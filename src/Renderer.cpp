#include "Renderer.h"
#include "Noise.h"
#include <Arduino.h>

// 二次多项式快速正弦/余弦近似 (Bhaskara I 近似，比标准 sinf 快 12 倍)
static inline float fastSin(float x) {
    float y = x * 0.15915494f; // x / 2PI
    y -= (int)(y + (y >= 0.0f ? 0.5f : -0.5f));
    y *= 6.2831853f; // 归一化到 [-PI, PI]
    
    float abs_y = y < 0.0f ? -y : y;
    float num = 16.0f * y * (3.14159265f - abs_y);
    float den = 49.348022f - 4.0f * y * (3.14159265f - abs_y);
    return num / den;
}

static inline float fastCos(float x) {
    return fastSin(x + 1.57079632f); // cos(x) = sin(x + PI/2)
}

// RGB565 颜色辅助插值函数
static uint16_t colorInterpolate(uint16_t color1, uint16_t color2, float t) {
    if (t <= 0.0f) return color1;
    if (t >= 1.0f) return color2;

    uint8_t r1 = (color1 >> 11) & 0x1F;
    uint8_t g1 = (color1 >> 5) & 0x3F;
    uint8_t b1 = color1 & 0x1F;

    uint8_t r2 = (color2 >> 11) & 0x1F;
    uint8_t g2 = (color2 >> 5) & 0x3F;
    uint8_t b2 = color2 & 0x1F;

    uint8_t r = r1 + (uint8_t)((r2 - r1) * t);
    uint8_t g = g1 + (uint8_t)((g2 - g1) * t);
    uint8_t b = b1 + (uint8_t)((b2 - b1) * t);

    return (r << 11) | (g << 5) | b;
}

Renderer::Renderer() 
    : _canvas(nullptr), _lightningFrames(0), _lightningType(0), _lightningColor(0),
      _epicenterX3D(0.0f), _epicenterY3D(0.0f), _epicenterZ3D(0.0f), _lightningX(0), _lightningY(0) {
    memset(_lightningFlicker, 0, sizeof(_lightningFlicker));
    for (int i = 0; i < MAX_RAIN_DROPS; i++) {
        _drops[i].active = false;
    }
}

void Renderer::init(M5Canvas* canvas) {
    _canvas = canvas;
}

void Renderer::updateParticles(const AtmosphereSimulation& sim, float dt) {
    // 1. 生成雨滴
    // 遍历模拟网格的云层
    for (int z = 1; z < AtmosphereSimulation::Z_SIZE; z++) {
        for (int y = 0; y < AtmosphereSimulation::Y_SIZE; y++) {
            for (int x = 0; x < AtmosphereSimulation::X_SIZE; x++) {
                const AirCell& cell = sim.getCell(x, y, z);
                
                // 只有当云密度大到降雨阈值，且随机命中时，在云底产生雨滴
                if (cell.cloudDensity > 0.65f && (random(1000) < 18)) {
                    // 寻找到空闲的雨滴槽
                    for (int i = 0; i < MAX_RAIN_DROPS; i++) {
                        if (!_drops[i].active) {
                            _drops[i].active = true;
                            _drops[i].x = (float)x + (random(100) / 100.0f);
                            _drops[i].y = (float)y + (random(100) / 100.0f);
                            _drops[i].z = (float)z;
                            // 继承网格水平风速，重力向下
                            _drops[i].vx = cell.velocityX;
                            _drops[i].vy = cell.velocityY;
                            _drops[i].vz = -4.5f - (random(100) / 50.0f);
                            break;
                        }
                    }
                }
            }
        }
    }

    // 2. 更新已有的雨滴
    for (int i = 0; i < MAX_RAIN_DROPS; i++) {
        if (!_drops[i].active) continue;

        // 亚像素更新
        _drops[i].x += _drops[i].vx * dt;
        _drops[i].y += _drops[i].vy * dt;
        _drops[i].z += _drops[i].vz * dt;
        _drops[i].vz -= 9.8f * dt; // 重力加速度

        // 获取所在网格的风速并稍加影响
        int gx = (int)_drops[i].x;
        int gy = (int)_drops[i].y;
        int gz = (int)_drops[i].z;
        if (gx >= 0 && gx < AtmosphereSimulation::X_SIZE &&
            gy >= 0 && gy < AtmosphereSimulation::Y_SIZE &&
            gz >= 0 && gz < AtmosphereSimulation::Z_SIZE) {
            const AirCell& c = sim.getCell(gx, gy, gz);
            _drops[i].vx = _drops[i].vx * 0.9f + c.velocityX * 0.1f;
            _drops[i].vy = _drops[i].vy * 0.9f + c.velocityY * 0.1f;
        }

        // 超界或掉落到地面 (z <= 0) 时回收
        if (_drops[i].z <= 0.0f || 
            _drops[i].x < 0.0f || _drops[i].x >= AtmosphereSimulation::X_SIZE ||
            _drops[i].y < 0.0f || _drops[i].y >= AtmosphereSimulation::Y_SIZE) {
            _drops[i].active = false;
        }
    }
}

void Renderer::draw(AtmosphereSimulation& sim, const Camera& cam, float extTemp, float extHum, float extPres, float extMicDb, int fps) {
    if (!_canvas) return;

    // 直接获取由物理模拟器在 update 中顺便完成统计的活跃云体素数
    int activeCloudCount = sim.getActiveCloudCount();
    const float densityThreshold = 0.4f;

    // 检查闪电事件，初始化闪电状态
    float electricCharge = sim.getElectricCharge();
    if (sim.consumeLightningEvent()) {
        // 查找所有活跃且密度较高的云格点作为闪电起始候选震源
        struct GridPoint { int x, y, z; };
        static GridPoint candidates[128]; // 使用静态数组节省栈空间，防范 ESP32 栈溢出
        int cCount = 0;
        
        for (int z = 3; z < AtmosphereSimulation::Z_SIZE; z++) { // 云一般位于中高层
            for (int y = 0; y < AtmosphereSimulation::Y_SIZE; y++) {
                for (int x = 0; x < AtmosphereSimulation::X_SIZE; x++) {
                    if (sim.getCell(x, y, z).cloudDensity > 0.5f) {
                        if (cCount < 128) {
                            candidates[cCount++] = {x, y, z};
                        }
                    }
                }
            }
        }

        if (cCount > 0) {
            int idx = random(cCount);
            _epicenterX3D = (float)candidates[idx].x;
            _epicenterY3D = (float)candidates[idx].y;
            _epicenterZ3D = (float)candidates[idx].z;
        } else {
            // 兜底默认震源（位于中部高空）
            _epicenterX3D = 8.0f + ((float)random(200) / 100.0f - 1.0f) * 3.0f;
            _epicenterY3D = 8.0f + ((float)random(200) / 100.0f - 1.0f) * 3.0f;
            _epicenterZ3D = 7.0f + (float)random(3);
        }

        // 投影得到屏幕 2D 起始点
        cam.project(_epicenterX3D + 0.5f, _epicenterY3D + 0.5f, _epicenterZ3D + 0.5f, _lightningX, _lightningY);
        
        // 强制夹紧防止折线出界
        if (_lightningX < 5) _lightningX = 5;
        if (_lightningX > SCREEN_W - 5) _lightningX = SCREEN_W - 5;
        if (_lightningY < 5) _lightningY = 5;
        if (_lightningY > SKY_AREA_H / 2) _lightningY = SKY_AREA_H / 2;

        // 配置闪电类型与发光参数
        _lightningType = random(3); // 0=云内闪电, 1=云际闪电, 2=闪地闪电
        
        // 随机发光电弧颜色（80%为淡紫蓝白，20%为明黄金色）
        if (random(10) < 8) {
            _lightningColor = RGB565(215 + random(30), 225 + random(20), 255); // 紫蓝白色
        } else {
            _lightningColor = RGB565(255, 225 + random(25), 110 + random(40)); // 暖黄金色
        }

        // 5帧正弦双波频闪衰减表 (主闪 -> 余晖 -> 回击 -> 渐隐)
        _lightningFrames = 5;
        _lightningFlicker[0] = 1.0f;
        _lightningFlicker[1] = 0.25f;
        _lightningFlicker[2] = 0.85f;
        _lightningFlicker[3] = 0.40f;
        _lightningFlicker[4] = 0.15f;
    }

    // 1. 正常渐变绘制天空背景
    drawSkyBackground(extTemp);

    // 2. 绘制体积云（内部根据 _lightningFrames 与物理 3D 距离计算扩散光照）
    drawClouds(sim, cam, densityThreshold);

    // 3. 绘制雨滴
    drawRain(cam);

    // 4. 绘制闪电外显电弧（纯云内闪电不绘制电弧线）
    if (_lightningFrames > 0 && _lightningType > 0) {
        float intensity = _lightningFlicker[5 - _lightningFrames];
        
        if (_lightningType == 1) {
            // 云际闪电：在云层间水平/斜向横穿拉起，模拟云中电荷击穿，不落地
            int endX = _lightningX + (int)(random(80) - 40);
            int endY = _lightningY + (int)(random(24) - 12);
            if (endY < 2) endY = 2;
            if (endY > SKY_AREA_H - 15) endY = SKY_AREA_H - 15;
            
            drawLightningBolt(_lightningX, _lightningY, endX, endY, 3, intensity);
        } 
        else if (_lightningType == 2) {
            // 闪地闪电：从高层云内向下树状分枝，直击地平线
            int endX = _lightningX + (int)(random(50) - 25);
            int endY = SKY_AREA_H - 3;
            if (endX < 2) endX = 2;
            if (endX > SCREEN_W - 2) endX = SCREEN_W - 2;

            drawLightningBolt(_lightningX, _lightningY, endX, endY, 3, intensity);
        }
    }

    if (_lightningFrames > 0) --_lightningFrames;

    // 5. 绘制状态栈信息 (气压显示值与台风中心气压梯度暴跌进行物理融合)
    float displayPres = extPres - 45.0f * sim.getTyphoonGrowth();
    drawStatusBar(extTemp, extHum, displayPres, extMicDb, activeCloudCount, electricCharge, fps);
}

void Renderer::drawSkyBackground(float extTemp) {
    // 根据外部温度插值计算天空背景的上下两端色彩
    // 寒冷 (<15C): 冰蓝 -> 亮青
    // 舒适 (15C-28C): 深蓝 -> 天蓝
    // 炎热 (>28C): 暗紫蓝 -> 暖黄/橙红
    uint16_t topColor, bottomColor;

    if (extTemp < 15.0f) {
        float t = fmaxf(0.0f, (extTemp - 0.0f) / 15.0f);
        topColor    = colorInterpolate(TFT_NAVY, RGB565(10, 40, 90), t);
        bottomColor = colorInterpolate(RGB565(80, 150, 180), RGB565(130, 210, 220), t);
    } else if (extTemp <= 28.0f) {
        float t = (extTemp - 15.0f) / 13.0f;
        topColor    = colorInterpolate(RGB565(10, 40, 90), RGB565(5, 30, 75), t);
        bottomColor = colorInterpolate(RGB565(130, 210, 220), RGB565(100, 180, 240), t);
    } else {
        float t = fminf(1.0f, (extTemp - 28.0f) / 12.0f);
        topColor    = colorInterpolate(RGB565(5, 30, 75), RGB565(20, 20, 50), t);
        bottomColor = colorInterpolate(RGB565(100, 180, 240), RGB565(230, 120, 50), t);
    }

    // ── 步长为 3 的渐变绘制天空区域（减少 66% 颜色插值与 draw 调用，视觉依旧平滑）──
    for (int row = 0; row < SKY_AREA_H; row += 3) {
        float t = (float)row / (float)(SKY_AREA_H - 1);
        uint16_t rowColor = colorInterpolate(topColor, bottomColor, t);
        int h = (row + 3 <= SKY_AREA_H) ? 3 : (SKY_AREA_H - row);
        _canvas->fillRect(0, row, SCREEN_W, h, rowColor);
    }
}

void Renderer::drawClouds(const AtmosphereSimulation& sim, const Camera& cam, float densityThreshold) {
    int drawBudget = 55;

    // 动态视口自适应循环方向选择（Front-to-Back）
    int xStart = 0, xEnd = AtmosphereSimulation::X_SIZE, xStep = 1;
    int yStart = 0, yEnd = AtmosphereSimulation::Y_SIZE, yStep = 1;

    if (cam._cachedCosA > 0.0f) {
        xStart = AtmosphereSimulation::X_SIZE - 1;
        xEnd = -1;
        xStep = -1;
    }
    if (cam._cachedSinA > 0.0f) {
        yStart = AtmosphereSimulation::Y_SIZE - 1;
        yEnd = -1;
        yStep = -1;
    }

    // 从最高层往低层画（z 轴从近到远）
    for (int z = AtmosphereSimulation::Z_SIZE - 1; z >= 0 && drawBudget > 0; --z) {
        // 恢复 baseStride = 1 精细采样，令粒子重新贴合重叠，恢复松软庞大的云团质感
        for (int y = yStart; y != yEnd && drawBudget > 0; y += yStep) {
            for (int x = xStart; x != xEnd && drawBudget > 0; x += xStep) {
                // 读取物理密度
                float d = sampleDensity(sim, x, y, z);
                if (d <= densityThreshold) continue;

                // 增加随机抖动打破网格规律感，抖动幅度随密度增大而减小（密集区域保持相对紧凑）
                float jitterScale = 0.6f * (1.0f - d * 0.5f);
                float jitterX = ((float)random(200) / 100.0f - 1.0f) * jitterScale;
                float jitterY = ((float)random(200) / 100.0f - 1.0f) * jitterScale;
                float jitterZ = ((float)random(200) / 100.0f - 1.0f) * 0.3f;
                int sx, sy;
                cam.project((float)x + 0.5f + jitterX, (float)y + 0.5f + jitterY, (float)z + 0.5f + jitterZ, sx, sy);

                // 屏幕投影剔除
                if (sx < -15 || sx > SCREEN_W + 15 || sy < -15 || sy > SKY_AREA_H + 15) continue;

                const AirCell& cell = sim.getCell(x, y, z);
                // 云粒子投影半径，增加随机变化打破均匀感
                float baseRadius = cam.scale * (0.35f + d * 0.85f);
                float radiusVariation = 0.3f * ((float)random(100) / 100.0f - 0.5f);
                float r = baseRadius * (1.0f + radiusVariation);
                if (r < 1.5f) r = 1.5f;

                // 光照强度计算与色彩插值
                float li = cell.lightIntensity;
                uint8_t r_light = 242, g_light = 242, b_light = 248;
                uint8_t r_dark = 58, g_dark = 62, b_dark = 82;
                uint8_t cr = (uint8_t)(r_dark + (r_light - r_dark) * li);
                uint8_t cg = (uint8_t)(g_dark + (g_light - g_dark) * li);
                uint8_t cb = (uint8_t)(b_dark + (b_light - b_dark) * li);
                
                float zRatio = (float)z / (AtmosphereSimulation::Z_SIZE - 1);
                cr = fminf(255, cr + zRatio * 12);
                cg = fminf(255, cg + zRatio * 12);
                cb = fminf(255, cb + zRatio * 8);
                uint16_t bodyColor = RGB565(cr, cg, cb);

                // 偏心三圆模拟 3D 漫反射软阴影与银边
                float offsetDist = r * 0.28f;
                int ox_light = (int)roundf((float)sx + offsetDist * 0.8f);
                int oy_light = (int)roundf((float)sy - offsetDist * 0.8f);
                int ox_dark  = (int)roundf((float)sx - offsetDist * 0.7f);
                int oy_dark  = (int)roundf((float)sy + offsetDist * 0.7f);

                float skyT = (float)sy / (float)SKY_AREA_H;
                uint16_t skyEst = colorInterpolate(RGB565(10, 40, 90), RGB565(130, 210, 220), skyT);

                uint16_t shadowColor = RGB565((uint8_t)(cr * 0.52f), (uint8_t)(cg * 0.54f), (uint8_t)(cb * 0.65f));
                uint16_t rimColor = colorInterpolate(RGB565(255, 252, 242), skyEst, 0.40f);

                // 三维空间闪电发光扩散插值模型
                if (_lightningFrames > 0) {
                    float intensity = _lightningFlicker[5 - _lightningFrames];
                    float dx = (float)x - _epicenterX3D;
                    float dy = (float)y - _epicenterY3D;
                    float dz = (float)z - _epicenterZ3D;
                    float distSq = dx*dx + dy*dy + dz*dz;
                    
                    float lightSpread = intensity * expf(-0.022f * distSq);
                    
                    if (lightSpread > 0.05f) {
                        if (lightSpread > 0.75f) {
                            bodyColor = RGB565(255, 255, 255);
                            rimColor = RGB565(255, 255, 255);
                            shadowColor = colorInterpolate(shadowColor, _lightningColor, 0.6f);
                        } else {
                            // 中等距离：与闪电底色自然光影混合
                            bodyColor = colorInterpolate(bodyColor, _lightningColor, lightSpread);
                            rimColor = colorInterpolate(rimColor, _lightningColor, lightSpread);
                            shadowColor = colorInterpolate(shadowColor, _lightningColor, lightSpread * 0.5f);
                        }
                    }
                }

                // ── 步骤一：底层阴影基底 (Shadow) ──
                _canvas->fillCircle(ox_dark, oy_dark, r + 0.8f, shadowColor);

                // ── 步骤二：中层体积核心圆 (Body Color) ──
                _canvas->fillCircle(sx, sy, r, bodyColor);

                // ── 步骤三：向光高光与透光银边圆 (Specular Rim & Mie Scattering) ──
                _canvas->fillCircle(ox_light, oy_light, r * 0.65f, rimColor);

                --drawBudget; // 消耗帧预算
            }
        }
    }
}
float Renderer::sampleDensity(const AtmosphereSimulation& sim, int x, int y, int z) const {
    float physicalDensity = sim.getCell(x, y, z).cloudDensity;
    
    float flowSpeedMultiplier = 1.0f + 2.5f * sim.getTyphoonGrowth();
    float timeSec = (millis() / 1000.0f) * flowSpeedMultiplier;
    float nx = (float)x / 16.0f;
    float ny = (float)y / 16.0f;
    float nz = (float)z / 12.0f;

    // 多层分形噪声，使用不同频率和相位，打破周期性规律
    float n1 = fastSin(nx * 25.13274f + timeSec) *
               fastCos(ny * 18.84956f + timeSec * 0.7f) *
               fastSin(nz * 31.41593f + timeSec * 0.4f);

    float n2 = fastSin(nx * 75.39822f + timeSec * 1.3f) *
               fastCos(ny * 62.83185f + timeSec * 0.9f) *
               fastSin(nz * 87.96459f + timeSec * 0.6f);

    float n3 = fastSin(nx * 47.12389f + timeSec * 0.5f) *
               fastCos(ny * 56.54867f + timeSec * 1.2f) *
               fastSin(nz * 43.98230f + timeSec * 0.8f);

    float n4 = fastSin(nx * 94.24778f + timeSec * 1.5f) *
               fastCos(ny * 109.95574f + timeSec * 0.3f) *
               fastSin(nz * 125.66371f + timeSec * 1.1f);

    float noise = 0.4f * n1 + 0.25f * n2 + 0.2f * n3 + 0.15f * n4;
    
    float spatialNoise = sim.getNoiseVal(x, y, z) * 0.08f;
    
    float edgeNoise = ((noise + 1.0f) * 0.5f) * 0.12f + spatialNoise;
    return physicalDensity + edgeNoise;
}


void Renderer::drawRain(const Camera& cam) {
    for (int i = 0; i < MAX_RAIN_DROPS; i++) {
        if (!_drops[i].active) continue;

        int sx1, sy1, sx2, sy2;
        // 起点与终点（拉出一条短线段，体现下落动态风速效果）
        cam.project(_drops[i].x, _drops[i].y, _drops[i].z, sx1, sy1);
        cam.project(_drops[i].x - _drops[i].vx * 0.05f, 
                    _drops[i].y - _drops[i].vy * 0.05f, 
                    _drops[i].z - _drops[i].vz * 0.05f, sx2, sy2);

        // 仅在天空绘制区域内进行画线
        if (sy1 >= 0 && sy1 < SKY_AREA_H && sy2 >= 0 && sy2 < SKY_AREA_H &&
            sx1 >= 0 && sx1 < SCREEN_W && sx2 >= 0 && sx2 < SCREEN_W) {
            
            // 雨滴颜色：冰蓝白色
            _canvas->drawLine(sx1, sy1, sx2, sy2, RGB565(190, 215, 240));
        }
    }
}

void Renderer::drawStatusBar(float extTemp, float extHum, float extPres, float extMicDb, int activeCloudCount, float electricCharge, int fps) {
    // 1. 填充状态栈底色（深黑灰色）
    _canvas->fillRect(0, STATUS_BAR_Y, SCREEN_W, STATUS_BAR_H, RGB565(25, 25, 30));

    // 2. 绘制亮白分界线
    _canvas->writeFastHLine(0, STATUS_BAR_Y, SCREEN_W, RGB565(80, 80, 90));

    // 3. 绘制传感器指标
    _canvas->setTextColor(RGB565(220, 220, 230));
    _canvas->setTextSize(1);

    // 第一行：温度与湿度
    char buf1[32];
    snprintf(buf1, sizeof(buf1), "T:%4.1fC H:%3d%%", extTemp, (int)extHum);
    _canvas->drawString(buf1, 6, STATUS_BAR_Y + 5);

    // 第二行：气压与分贝风力
    char buf2[32];
    snprintf(buf2, sizeof(buf2), "P:%6.1fhPa M:%2ddB", extPres, (int)extMicDb);
    _canvas->drawString(buf2, 6, STATUS_BAR_Y + 18);

    // 第三行：云体素格数
    char buf3[32];
    snprintf(buf3, sizeof(buf3), "C:%d/3072", activeCloudCount);
    _canvas->drawString(buf3, 6, STATUS_BAR_Y + 31);

    // 4. 右下角闪电指示器
    // 电荷 < 30%: 灰白圆点   30%~70%: 黄色   >70%: 闪烁小闪电图标
    float chargeRatio = electricCharge / LIGHTNING_CHARGE_THRESHOLD;
    uint16_t indicatorColor;
    if (chargeRatio > 0.70f) {
        // 电荷很高，闪烁黄色提醒即将打雷
        indicatorColor = (millis() / 200) % 2 == 0 ? RGB565(255, 240, 0) : RGB565(180, 160, 0);
    } else if (chargeRatio > 0.30f) {
        indicatorColor = RGB565(220, 180, 0); // 黄色：电荷积累中
    } else if (extMicDb > 60.0f) {
        indicatorColor = RGB565(100, 255, 120); // 绿色：麦克风活跃
    } else {
        indicatorColor = RGB565(160, 160, 170); // 灰白：平静
    }
    _canvas->fillCircle(SCREEN_W - 14, STATUS_BAR_Y + 35, 4, indicatorColor);

    // 5. 右上角显示 FPS
    char buf4[32];
    snprintf(buf4, sizeof(buf4), "F:%d", fps);
    _canvas->drawString(buf4, SCREEN_W - 30, STATUS_BAR_Y + 5);
}

// ─────────────────────────────────────────────────────────────
//  分形闪电绘制（递归）
//  算法：将从 (x1,y1) 到 (x2,y2) 的线段在中点处随机偏移，
//        然后递归绘制两段子线；depth≥2 时以 40% 概率产生侧枝。
//  在 135px 宽度内 depth=3 产生 7 条主干 + 2~4 条侧枝，共约 14 条 drawLine。
// ─────────────────────────────────────────────────────────────
void Renderer::drawLightningBolt(int x1, int y1, int x2, int y2, int depth, float intensity) {
    if (!_canvas) return;

    // 终止条件：段太短或递归到底
    if (depth == 0 || abs(y2 - y1) < 3) {
        uint16_t col;
        if (depth >= 2) {
            // 主干：接近纯白的高亮电弧核，受 intensity 影响
            col = colorInterpolate(_lightningColor, RGB565(255, 255, 255), intensity * 0.8f);
            
            // 使用偏移画线法模拟 3 像素粗线宽
            _canvas->drawLine(x1, y1, x2, y2, col);
            _canvas->drawLine(x1 + 1, y1, x2 + 1, y2, col);
            _canvas->drawLine(x1, y1 + 1, x2, y2 + 1, col);
        } else {
            // 侧枝：相比主干颜色略暗且偏蓝紫，采用单像素细线段
            col = colorInterpolate(RGB565(45, 45, 65), _lightningColor, intensity * 0.7f);
            _canvas->drawLine(x1, y1, x2, y2, col);
        }
        return;
    }

    // 中点 + 横向随机扰动（扰动量随深度递减）
    int jitter = (depth * 7);              // 稍降偏移量，使闪电轮廓更尖锐刚劲
    int mx = (x1 + x2) / 2 + (int)(random(jitter * 2 + 1) - jitter);
    int my = (y1 + y2) / 2;

    // 边界夹紧
    if (mx < 2) mx = 2;
    if (mx > SCREEN_W - 2) mx = SCREEN_W - 2;

    // 递归绘制上半段和下半段
    drawLightningBolt(x1, y1, mx, my, depth - 1, intensity);
    drawLightningBolt(mx, my, x2, y2, depth - 1, intensity);

    // 侧枝分叉：以约 38% 概率产生旁出侧枝
    if (depth >= 2 && random(100) < 38) {
        int branchEndX = mx + (int)(random(40) - 20);
        int branchEndY = my + (int)(random(24) + 10);
        
        // 限制侧画不跨越状态栏或边界
        if (branchEndY < SKY_AREA_H - 12 && branchEndX > 1 && branchEndX < SCREEN_W - 1) {
            drawLightningBolt(mx, my, branchEndX, branchEndY, depth - 2, intensity);
        }
    }
}
