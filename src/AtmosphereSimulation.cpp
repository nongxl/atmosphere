#include "AtmosphereSimulation.h"
#include <math.h>
#include "Noise.h"
#include "config.h"

AtmosphereSimulation::AtmosphereSimulation()
    : _electricCharge(0.0f), _lightningPending(false), _sunAzimuth(PI / 4.0f), _activeCloudCount(0), _typhoonGrowth(0.0f)
{
    _cells      = new AirCell[X_SIZE * Y_SIZE * Z_SIZE];
    _nextCells  = new AirCell[X_SIZE * Y_SIZE * Z_SIZE];
    _noiseTable = new float[X_SIZE * Y_SIZE * Z_SIZE];

    // Precalculate noise table for Vsat modulation (removes ~9000 sinf/cosf per frame)
    for (int z = 0; z < Z_SIZE; z++) {
        for (int y = 0; y < Y_SIZE; y++) {
            for (int x = 0; x < X_SIZE; x++) {
                float ns = sinf((float)x * 0.5f) * cosf((float)y * 0.5f) * sinf((float)z * 0.7f)
                         + sinf((float)x * 1.1f + (float)y * 0.9f) * cosf((float)z * 1.4f) * 0.35f
                         + cosf(((float)x - (float)y) * 0.3f) * sinf(((float)z + (float)x) * 0.6f) * 0.2f;
                _noiseTable[z * X_SIZE * Y_SIZE + y * X_SIZE + x] = ns;
            }
        }
    }
}

AtmosphereSimulation::~AtmosphereSimulation() {
    delete[] _cells;
    delete[] _nextCells;
    delete[] _noiseTable;
}

// ─── init ──────────────────────────────────────────────────────────────────
void AtmosphereSimulation::init(float initTemp, float initHum, float initPres) {
    _typhoonGrowth = 0.0f;
    float initVapor = initHum / 100.0f;
    for (int z = 0; z < Z_SIZE; z++) {
        // Temperature decreases ~1.25 deg per layer; pressure drops with height
        float targetTemp = initTemp - (z * 1.25f);
        float targetPres = initPres * (1.0f - z * 0.008f);
        for (int y = 0; y < Y_SIZE; y++) {
            for (int x = 0; x < X_SIZE; x++) {
                AirCell& cell = getCellRef(x, y, z);
                cell.temperature  = targetTemp;
                cell.vapor        = initVapor * expf(-(float)z * 0.16f);
                cell.cloudDensity = 0.0f;
                cell.velocityX    = 0.0f;
                cell.velocityY    = 0.0f;
                cell.velocityZ    = 0.0f;
                cell.pressure     = targetPres;
                cell.lightIntensity = 1.0f;
            }
        }
    }
    memcpy(_nextCells, _cells, sizeof(AirCell) * X_SIZE * Y_SIZE * Z_SIZE);
}

// ─── update ────────────────────────────────────────────────────────────────
void AtmosphereSimulation::update(float dt, float extTemp, float extHum,
                                  float extPres, float extMicDb) {
    if (dt <= 0.0f) return;
    if (dt >  0.1f) dt = 0.1f;

    // 麦克风风能积累台风强度模型 (吹气时 >65dB 促使其加速成长，平静时缓慢消散)
    if (extMicDb > 65.0f) {
        float micRatio = (extMicDb - 65.0f) / 35.0f;
        if (micRatio > 1.0f) micRatio = 1.0f;
        _typhoonGrowth += micRatio * 0.22f * dt; // 蓄能
    } else {
        _typhoonGrowth -= 0.038f * dt; // 耗散
    }
    if (_typhoonGrowth < 0.0f) _typhoonGrowth = 0.0f;
    if (_typhoonGrowth > 1.0f) _typhoonGrowth = 1.0f;

    // 1. Ground boundary: slowly drive z=0 cells toward sensor readings
    // Amplify vapor 2.5x so 55% indoor humidity saturates high layers
    float targetVapor = fminf(1.0f, (extHum / 100.0f) * 2.5f);
    for (int y = 0; y < Y_SIZE; y++) {
        for (int x = 0; x < X_SIZE; x++) {
            AirCell& cell = getCellRef(x, y, 0);
            cell.temperature = cell.temperature * (1.0f - 0.2f * dt) + extTemp * 0.2f * dt;
            cell.vapor       = cell.vapor       * (1.0f - 0.2f * dt) + targetVapor * 0.2f * dt;
        }
    }

    // 2. Thermal convection column (5% chance, down from 14% for stability)
    if (random(100) < 5) {
        int tx = random(X_SIZE);
        int ty = random(Y_SIZE);
        float strength = 0.8f + (random(100) / 100.0f);
        for (int z = 0; z < Z_SIZE - 2; z++) {
            AirCell& cell = getCellRef(tx, ty, z);
            float ratio = 1.0f - ((float)z / (Z_SIZE - 2));
            cell.velocityZ += strength * ratio * 2.0f;
            if (z <= 2) {
                cell.temperature += strength * 1.5f;
                // Vapor not injected here; let diffusion handle it
            }
        }
    }

    // 3. Microphone wind (suppress first 3 s of boot to ignore mic pop)
    static float simRunTime = 0.0f;
    simRunTime += dt;
    if (simRunTime > 3.0f && extMicDb > 60.0f) {
        float windForce = (extMicDb - 60.0f) * 0.15f;
        for (int z = 1; z < Z_SIZE - 1; z++) {
            for (int y = 0; y < Y_SIZE; y++) {
                AirCell& cell = getCellRef(0, y, z);
                cell.velocityX += windForce * (1.0f - (float)z / Z_SIZE);
            }
        }
    }

    // 4. Diffusion / advection
    // ── 4a. Copy current state into _nextCells ──
    memcpy(_nextCells, _cells, sizeof(AirCell) * X_SIZE * Y_SIZE * Z_SIZE);

    // ── 4b. Diffusion + buoyancy + damping ──
    for (int z = 0; z < Z_SIZE; z++) {
        for (int y = 0; y < Y_SIZE; y++) {
            for (int x = 0; x < X_SIZE; x++) {
                AirCell& cur  = getCellRef(x, y, z);
                AirCell& next = _nextCells[z * X_SIZE * Y_SIZE + y * X_SIZE + x];

                // 6-neighbor diffusion (temperature, vapor, cloud) - manually unrolled to remove lambda overhead
                float sumT = 0, sumV = 0, sumC = 0;
                int   cnt  = 0;
                
                if (x > 0) {
                    const AirCell& nb = getCell(x - 1, y, z);
                    sumT += nb.temperature; sumV += nb.vapor; sumC += nb.cloudDensity; cnt++;
                }
                if (x < X_SIZE - 1) {
                    const AirCell& nb = getCell(x + 1, y, z);
                    sumT += nb.temperature; sumV += nb.vapor; sumC += nb.cloudDensity; cnt++;
                }
                if (y > 0) {
                    const AirCell& nb = getCell(x, y - 1, z);
                    sumT += nb.temperature; sumV += nb.vapor; sumC += nb.cloudDensity; cnt++;
                }
                if (y < Y_SIZE - 1) {
                    const AirCell& nb = getCell(x, y + 1, z);
                    sumT += nb.temperature; sumV += nb.vapor; sumC += nb.cloudDensity; cnt++;
                }
                if (z > 0) {
                    const AirCell& nb = getCell(x, y, z - 1);
                    sumT += nb.temperature; sumV += nb.vapor; sumC += nb.cloudDensity; cnt++;
                }
                if (z < Z_SIZE - 1) {
                    const AirCell& nb = getCell(x, y, z + 1);
                    sumT += nb.temperature; sumV += nb.vapor; sumC += nb.cloudDensity; cnt++;
                }

                if (cnt > 0) {
                    float diffT  = 0.08f * dt;
                    float diffV  = 0.06f * dt;
                    float diffC  = 0.04f * dt;
                    next.temperature  += diffT * (sumT/cnt - cur.temperature);
                    next.vapor        += diffV * (sumV/cnt - cur.vapor);
                    next.cloudDensity += diffC * (sumC/cnt - cur.cloudDensity);
                }

                // Buoyancy: warmer cells rise
                float buoyancy = (cur.temperature - extTemp) * 0.015f * dt;
                next.velocityZ += buoyancy;

                // Velocity damping
                next.velocityX *= (1.0f - 0.12f * dt);
                next.velocityY *= (1.0f - 0.12f * dt);
                next.velocityZ *= (1.0f - 0.10f * dt);

                // 【核心台风动力气旋风场注入】
                if (_typhoonGrowth > 0.02f) {
                    float centerX = (X_SIZE - 1) / 2.0f; // 7.5f
                    float centerY = (Y_SIZE - 1) / 2.0f; // 7.5f
                    float dx = (float)x - centerX;
                    float dy = (float)y - centerY;
                    float dist = sqrtf(dx * dx + dy * dy);
                    if (dist < 0.01f) dist = 0.01f;

                    // 动态风眼半径（台风成熟时，眼洞收缩为 1.8，外围云墙逼近）
                    float R_eye = 2.4f - _typhoonGrowth * 0.6f;
                    float zRatio = 1.0f - ((float)z / Z_SIZE);

                    if (dist <= R_eye) {
                        // ── 沉降风眼内部 (无风无云，下沉流场) ──
                        float eyeFactor = dist / R_eye;
                        next.velocityX *= 0.35f;
                        next.velocityY *= 0.35f;
                        next.velocityZ = next.velocityZ * 0.35f - 1.3f * (1.0f - eyeFactor) * _typhoonGrowth * zRatio;
                    } else {
                        // ── 风眼墙及螺旋外围气流 (上升气流与狂风旋涡) ──
                        float factor = 1.0f;
                        if (dist < 6.0f) {
                            // 风眼墙区域：上升对流和旋转切变极强
                            factor = (dist - R_eye) / (6.0f - R_eye);
                            next.velocityZ += 3.6f * factor * _typhoonGrowth * zRatio * dt;
                        } else {
                            // 螺旋区向外随距离衰减
                            factor = fmaxf(0.1f, 1.0f - (dist - 6.0f) / 10.0f);
                        }

                        // 逆时针旋转的切线风向
                        float rx = -dy / dist;
                        float ry =  dx / dist;

                        // 强力气旋自转插值，k 为自转约束权重
                        float k = 0.70f * _typhoonGrowth;
                        float targetRotX = rx * 4.8f * _typhoonGrowth * factor * zRatio;
                        float targetRotY = ry * 4.8f * _typhoonGrowth * factor * zRatio;

                        next.velocityX = next.velocityX * (1.0f - k) + targetRotX * k;
                        next.velocityY = next.velocityY * (1.0f - k) + targetRotY * k;
                    }
                }

                // Turbulence
                if (random(1000) < 30) {
                    next.velocityX += (random(100) - 50) * 0.015f;
                    next.velocityY += (random(100) - 50) * 0.015f;
                    next.velocityZ += (random(100) - 50) * 0.008f;
                }

                // Clamp velocities (台风眼墙允许较强风速)
                float maxV = (_typhoonGrowth > 0.4f) ? 7.5f : 6.0f;
                auto clampV = [maxV](float v) { return v<-maxV?-maxV:(v>maxV?maxV:v); };
                next.velocityX = clampV(next.velocityX);
                next.velocityY = clampV(next.velocityY);
                next.velocityZ = clampV(next.velocityZ);
            }
        }
    }
    memcpy(_cells, _nextCells, sizeof(AirCell) * X_SIZE * Y_SIZE * Z_SIZE);

    // 5. Condensation / evaporation
    for (int z = 0; z < Z_SIZE; z++) {
        for (int y = 0; y < Y_SIZE; y++) {
            for (int x = 0; x < X_SIZE; x++) {
                AirCell& cell = getCellRef(x, y, z);

                // Saturation threshold (lower = easier to condense)
                float normalizedT = (cell.temperature + 5.0f) / 40.0f;
                float Vsat = 0.04f + 0.62f * fmaxf(0.0f, fminf(1.0f, normalizedT));
                Vsat = fmaxf(0.04f, Vsat - (float)z * 0.025f);

                // Fractal turbulence noise breaks geometric uniformity
                float ns = _noiseTable[z * X_SIZE * Y_SIZE + y * X_SIZE + x];
                Vsat = fmaxf(0.05f, Vsat + ns * 0.09f);

                // 【台风眼晴空下沉与风眼墙积云】
                if (_typhoonGrowth > 0.02f) {
                    float centerX = (X_SIZE - 1) / 2.0f;
                    float centerY = (Y_SIZE - 1) / 2.0f;
                    float dx = (float)x - centerX;
                    float dy = (float)y - centerY;
                    float dist = sqrtf(dx * dx + dy * dy);
                    
                    float R_eye = 2.4f - _typhoonGrowth * 0.6f;
                    if (dist <= R_eye) {
                        // 沉降风眼内部：下沉气流主导，强行消散水汽与云密度
                        float eyeFactor = dist / R_eye;
                        cell.cloudDensity *= (0.15f + 0.65f * eyeFactor * eyeFactor);
                        cell.vapor *= (0.3f + 0.7f * eyeFactor);
                    } else if (dist <= 5.5f) {
                        // 风眼墙云系：强力补充水分与初始云量，构造厚重螺旋云墙
                        float factor = (5.5f - dist) / (5.5f - R_eye);
                        cell.vapor = fminf(1.0f, cell.vapor + 0.38f * factor * _typhoonGrowth);
                        cell.cloudDensity = fminf(1.0f, cell.cloudDensity + 0.32f * factor * _typhoonGrowth);
                    }
                }

                // Condensation / evaporation
                if (cell.vapor > Vsat) {
                    // Over-saturated → condense
                    float cond = (cell.vapor - Vsat) * 0.35f * dt;
                    cell.vapor        -= cond;
                    cell.cloudDensity += cond;
                    cell.temperature  += cond * 2.5f; // latent heat
                    if (cell.cloudDensity > 1.0f) cell.cloudDensity = 1.0f;
                } else {
                    // Under-saturated → evaporate
                    float evap = (Vsat - cell.vapor) * 0.12f * dt;
                    float evapFromCloud = fminf(evap, cell.cloudDensity);
                    cell.cloudDensity -= evapFromCloud;
                    cell.vapor        += evapFromCloud;
                    cell.temperature  -= evapFromCloud * 1.5f;
                    if (cell.cloudDensity < 0.0f) cell.cloudDensity = 0.0f;
                }

                // Rainfall: dense low-layer clouds shed precipitation
                if (z == 0 && cell.cloudDensity > 0.7f) {
                    float rain = (cell.cloudDensity - 0.7f) * 0.5f * dt;
                    cell.cloudDensity -= rain;
                }

                // 【气压场重构】台风中心超强低气压倒钟梯度分布
                float basePres = extPres * (1.0f - (float)z * 0.008f);
                if (_typhoonGrowth > 0.01f) {
                    float centerX = (X_SIZE - 1) / 2.0f;
                    float centerY = (Y_SIZE - 1) / 2.0f;
                    float dx = (float)x - centerX;
                    float dy = (float)y - centerY;
                    float dist = sqrtf(dx * dx + dy * dy);
                    // 倒钟形降幅分布，中心最大暴跌达 45 hPa，外围逐渐过渡回常压
                    float presDrop = 45.0f * _typhoonGrowth * expf(-0.065f * dist * dist);
                    cell.pressure = basePres - presDrop;
                } else {
                    cell.pressure = basePres;
                }

                // Clamp
                if (cell.vapor        < 0.0f) cell.vapor        = 0.0f;
                if (cell.vapor        > 1.0f) cell.vapor        = 1.0f;
                if (cell.cloudDensity < 0.0f) cell.cloudDensity = 0.0f;
                if (cell.cloudDensity > 1.0f) cell.cloudDensity = 1.0f;
            }
        }
    }

    // 6. Compute lighting (Beer-Law self-shadow)
    computeLightField();

    // 7. Electric charge accumulation & Cloud counting (merged to single pass)
    float chargeGain = 0.0f;
    int tempCloudCount = 0;
    for (int z = 0; z < Z_SIZE; z++) {
        for (int y = 0; y < Y_SIZE; y++) {
            for (int x = 0; x < X_SIZE; x++) {
                const AirCell& c = getCell(x, y, z);
                if (c.cloudDensity > 0.4f) {
                    tempCloudCount++;
                }
                if (z >= Z_SIZE / 2) {
                    chargeGain += c.cloudDensity * fabsf(c.velocityZ) * 0.05f;
                }
            }
        }
    }
    _activeCloudCount = tempCloudCount;

    _electricCharge += chargeGain * dt;
    _electricCharge *= (1.0f - 0.01f * dt);  // natural leakage
    if (_electricCharge < 0.0f) _electricCharge = 0.0f;

    if (_electricCharge > LIGHTNING_CHARGE_THRESHOLD) {
        _lightningPending  = true;
        _electricCharge   *= LIGHTNING_CHARGE_DECAY;
    }
}

// ─── consumeLightningEvent ─────────────────────────────────────────────────
bool AtmosphereSimulation::consumeLightningEvent() {
    if (_lightningPending) {
        _lightningPending = false;
        return true;
    }
    return false;
}

// ─── injectCyclone ─────────────────────────────────────────────────────────
void AtmosphereSimulation::injectCyclone() {
    // Inject rotating updraft at grid center; immediately seed cloud density
    // at mid/upper layers so BtnA gives instant visual feedback
    float centerX = (X_SIZE - 1) / 2.0f;
    float centerY = (Y_SIZE - 1) / 2.0f;

    for (int z = 0; z < Z_SIZE; z++) {
        float zRatio = 1.0f - ((float)z / Z_SIZE);
        for (int y = 0; y < Y_SIZE; y++) {
            for (int x = 0; x < X_SIZE; x++) {
                float dx   = x - centerX;
                float dy   = y - centerY;
                float dist = sqrtf(dx * dx + dy * dy);

                if (dist < 5.5f) {
                    float factor = (5.5f - dist) / 5.5f;
                    AirCell& cell = getCellRef(x, y, z);

                    if (z <= 3) {
                        // Low layers: warm moist air to trigger buoyancy
                        cell.temperature = 38.0f;
                        cell.vapor       = 1.0f;
                    } else {
                        // Mid/upper: cold moist air → low Vsat → instant condensation
                        cell.temperature  -= 12.0f * factor;
                        cell.vapor         = fminf(1.0f, cell.vapor + 0.7f * factor);
                        cell.cloudDensity  = fminf(1.0f, cell.cloudDensity + 0.55f * factor);
                    }

                    // Updraft (strongest at center)
                    cell.velocityZ += 5.5f * factor * zRatio;

                    // Counter-clockwise rotation
                    if (dist > 0.1f) {
                        float rx = -dy / dist;
                        float ry =  dx / dist;
                        float windNoiseX = sinf((float)x * 0.7f + (float)z * 1.5f) * 1.4f;
                        float windNoiseY = cosf((float)y * 0.7f - (float)z * 1.5f) * 1.4f;
                        cell.velocityX += (rx * 3.8f + windNoiseX) * factor * zRatio;
                        cell.velocityY += (ry * 3.8f + windNoiseY) * factor * zRatio;
                    }
                }
            }
        }
    }
}

// ─── setSunAzimuth ─────────────────────────────────────────────────────────
void AtmosphereSimulation::setSunAzimuth(float azimuth) {
    _sunAzimuth = azimuth;
    // Use literal to avoid clash with Arduino.h's TWO_PI macro
    while (_sunAzimuth >= 6.2831853f) _sunAzimuth -= 6.2831853f;
    while (_sunAzimuth <  0.0f)       _sunAzimuth += 6.2831853f;
}

static inline float approxExpNeg(float x) {
    if (x < 0.0f) x = 0.0f;
    // Pade approximation for 1/(1+x+x^2/2+x^3/6) which is extremely fast and accurate for positive inputs.
    float t = 1.0f + x * (1.0f + x * (0.5f + x * 0.16666667f));
    return 1.0f / t;
}

// ─── computeLightField ─────────────────────────────────────────────────────
void AtmosphereSimulation::computeLightField() {
    // Sun direction is driven by _sunAzimuth (enables sunrise/sunset effects)
    // Map continuous azimuth to discrete grid offset (sunDX, sunDY) in {-1,0,+1}
    float sunCosA = cosf(_sunAzimuth);
    float sunSinA = sinf(_sunAzimuth);
    int sunDX = (sunCosA >  0.42f) ?  1 : (sunCosA < -0.42f) ? -1 : 0;
    int sunDY = (sunSinA >  0.42f) ?  1 : (sunSinA < -0.42f) ? -1 : 0;

    // Propagate top-down (z = Z_SIZE-1 is fully lit)
    for (int z = Z_SIZE - 1; z >= 0; z--) {
        for (int y = 0; y < Y_SIZE; y++) {
            for (int x = 0; x < X_SIZE; x++) {
                AirCell& cell = getCellRef(x, y, z);

                if (z == Z_SIZE - 1) {
                    cell.lightIntensity = 1.0f;
                } else {
                    int nx = x + sunDX;
                    int ny = y + sunDY;
                    int nz = z + 1;

                    float parentLight   = 1.0f;
                    float parentDensity = 0.0f;

                    if (nx >= 0 && nx < X_SIZE && ny >= 0 && ny < Y_SIZE) {
                        const AirCell& parent = getCell(nx, ny, nz);
                        parentLight   = parent.lightIntensity;
                        parentDensity = parent.cloudDensity;
                    } else {
                        // At boundary, fall back to vertical parent
                        const AirCell& parent = getCell(x, y, nz);
                        parentLight   = parent.lightIntensity;
                        parentDensity = parent.cloudDensity;
                    }

                    // Beer-Law attenuation + scatter + ambient + rim highlight
                    float baseLight  = parentLight * approxExpNeg(parentDensity * 3.3f);
                    float scatter    = 0.3f;
                    float ambient    = 0.12f;
                    float rim        = ((float)z / (float)Z_SIZE) * 0.4f;
                    float finalLight = baseLight * (scatter + ambient + rim);
                    cell.lightIntensity = finalLight < 0.08f ? 0.08f : finalLight;
                }
            }
        }
    }
}
