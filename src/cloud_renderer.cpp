#include "cloud_renderer.h"
#include <math.h>
#include <string.h>

CloudRenderer::CloudRenderer()
    : _debugMode(CLOUD_DEBUG == 1)
{
}

void CloudRenderer::init(M5Canvas& canvas, const CloudConfig& config) {
    _config = config;
    canvas.setColorDepth(16); // 16-bit RGB565 色彩
    canvas.createSprite(SCREEN_W, SCREEN_H);
    precomputeBackground();
}

void CloudRenderer::applyPreset(CloudLightPreset preset) {
    switch (preset) {
        case CLOUD_LIGHT_SOFT:
            _config.lightIntensity  = 0.65f;
            _config.ambientLight    = 0.45f;
            _config.shadowStrength  = 0.45f;
            _config.bottomShade     = 0.18f;
            break;
        case CLOUD_LIGHT_NATURAL:
            _config.lightIntensity  = 0.80f;
            _config.ambientLight    = 0.35f;
            _config.shadowStrength  = 0.65f;
            _config.bottomShade     = 0.26f;
            break;
        case CLOUD_LIGHT_DRAMATIC:
            _config.lightIntensity  = 0.95f;
            _config.ambientLight    = 0.25f;
            _config.shadowStrength  = 0.82f;
            _config.bottomShade     = 0.35f;
            break;
    }
}

// 快速字节序转换: M5Canvas 内部 Framebuffer 为大端序 (Big-Endian) RGB565
static inline uint16_t packRGB565_BE(uint8_t r, uint8_t g, uint8_t b) {
    uint16_t c = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
    return __builtin_bswap16(c);
}

void CloudRenderer::precomputeBackground() {
    int groundStart = SCREEN_H - _config.groundHeight;
    uint8_t r_top, g_top, b_top, r_bot, g_bot, b_bot, r_gnd, g_gnd, b_gnd;
    decodeRGB565(_config.skyTopColor, r_top, g_top, b_top);
    decodeRGB565(_config.skyBottomColor, r_bot, g_bot, b_bot);
    decodeRGB565(_config.groundColor, r_gnd, g_gnd, b_gnd);

    for (int y = 0; y < SCREEN_H; ++y) {
        if (y < groundStart) {
            float t = (float)y / (float)(groundStart > 0 ? groundStart : 1);
            uint8_t r = (uint8_t)lerpF(r_top, r_bot, t);
            uint8_t g = (uint8_t)lerpF(g_top, g_bot, t);
            uint8_t b = (uint8_t)lerpF(b_top, b_bot, t);
            _skyRowColors[y] = packRGB565_BE(r, g, b);
        } else {
            _skyRowColors[y] = packRGB565_BE(r_gnd, g_gnd, b_gnd);
        }
    }
}

void CloudRenderer::render(M5Canvas& canvas, const CloudPhysics& physics, const IMUController& imu, float fps, bool debugMode) {
    uint32_t renderStartUs = micros();

    const CloudBlob* blobs = physics.getBlobs();
    const int blobCount = physics.getBlobCount();
    const float cloudCenterX = physics.getCenterX();
    const float cloudCenterY = physics.getCenterY();

    // 0. 获取 Framebuffer 直接内存指针
    uint16_t* fb = (uint16_t*)canvas.getBuffer();
    if (!fb) return;

    // 1. 快速填充全屏预计算背景 (直接内存拷贝填充)
    for (int y = 0; y < SCREEN_H; ++y) {
        uint16_t rowColor = _skyRowColors[y];
        uint16_t* rowPtr = fb + y * SCREEN_W;
        for (int x = 0; x < SCREEN_W; ++x) {
            rowPtr[x] = rowColor;
        }
    }

    // 2. 紧凑计算云朵外接包围盒 (Bounding Box)
    float minX = SCREEN_W, maxX = 0;
    float minY = SCREEN_H, maxY = 0;

    for (int i = 0; i < blobCount; ++i) {
        float r = blobs[i].currentRadius;
        float bx0 = blobs[i].currentX - r;
        float bx1 = blobs[i].currentX + r;
        float by0 = blobs[i].currentY - r;
        float by1 = blobs[i].currentY + r;

        if (bx0 < minX) minX = bx0;
        if (bx1 > maxX) maxX = bx1;
        if (by0 < minY) minY = by0;
        if (by1 > maxY) maxY = by1;
    }

    const float edgeSoft = _config.edgeSoftness;
    const float k = _config.sminBlendK;
    const float occRadius = _config.occlusionRadius;
    const float padding = k + edgeSoft + 2.0f;

    // 偶数对齐包围盒边界
    int bbX0 = ((int)floorf(clampF(minX - padding, 0.0f, (float)(SCREEN_W - 1)))) & ~1;
    int bbX1 = ((int)ceilf(clampF(maxX + padding, 0.0f, (float)(SCREEN_W - 1)))) | 1;
    int bbY0 = ((int)floorf(clampF(minY - padding, 0.0f, (float)(SCREEN_H - 1)))) & ~1;
    int bbY1 = ((int)ceilf(clampF(maxY + padding, 0.0f, (float)(SCREEN_H - 1)))) | 1;

    if (bbX1 >= SCREEN_W) bbX1 = SCREEN_W - 1;
    if (bbY1 >= SCREEN_H) bbY1 = SCREEN_H - 1;

    // 3. 2x2 亚采样着色网格 (8-bit RGB 空间，杜绝颜色失真与字节序错误)
    const float noiseAmt = _config.noiseAmount;
    const float lightDirX = _config.lightDirX;
    const float lightDirY = _config.lightDirY;
    const float lightIntensity = _config.lightIntensity;
    const float ambientLight = _config.ambientLight;
    const float shadowStrength = _config.shadowStrength;
    const float bottomShade = _config.bottomShade;
    const float depthInfluence = _config.depthLightInfluence;

    float lightLen = sqrtf(lightDirX * lightDirX + lightDirY * lightDirY);
    float lnx = (lightLen > 0.0001f) ? (lightDirX / lightLen) : -0.6f;
    float lny = (lightLen > 0.0001f) ? (lightDirY / lightLen) : -0.8f;
    float occOffsetX = lnx * occRadius;
    float occOffsetY = lny * occRadius;

    float bx[CloudConfig::BLOB_COUNT], by[CloudConfig::BLOB_COUNT], br[CloudConfig::BLOB_COUNT], bdepth[CloudConfig::BLOB_COUNT];
    for (int i = 0; i < blobCount; ++i) {
        bx[i] = blobs[i].currentX;
        by[i] = blobs[i].currentY;
        br[i] = blobs[i].currentRadius;
        bdepth[i] = blobs[i].depth;
    }

    static constexpr int MAX_GW = (SCREEN_W / 2) + 2;
    static constexpr int MAX_GH = (SCREEN_H / 2) + 2;
    static uint8_t gridR[MAX_GH][MAX_GW];
    static uint8_t gridG[MAX_GH][MAX_GW];
    static uint8_t gridB[MAX_GH][MAX_GW];
    static uint8_t gridAlpha[MAX_GH][MAX_GW]; // 0~255

    int gridX0 = bbX0 / 2;
    int gridX1 = bbX1 / 2;
    int gridY0 = bbY0 / 2;
    int gridY1 = bbY1 / 2;

    for (int gy = gridY0; gy <= gridY1; ++gy) {
        float fpy = (float)(gy * 2);

        for (int gx = gridX0; gx <= gridX1; ++gx) {
            float fpx = (float)(gx * 2);

            // ── 单次 8-Blob 遍历求 SDF 与解析法线 ──
            float dx0 = fpx - bx[0];
            float dy0 = fpy - by[0];
            float dist0 = sqrtf(dx0 * dx0 + dy0 * dy0);
            float d = dist0 - br[0];

            float invDist0 = (dist0 > 0.001f) ? (1.0f / dist0) : 0.0f;
            float nx = dx0 * invDist0;
            float ny = dy0 * invDist0;

            float w0 = 1.0f / (dist0 + 0.1f);
            float totalWeight = w0;
            float weightedDepth = bdepth[0] * w0;

            for (int i = 1; i < blobCount; ++i) {
                float dx = fpx - bx[i];
                float dy = fpy - by[i];
                float dist_i = sqrtf(dx * dx + dy * dy);
                float di = dist_i - br[i];

                float h = clampF(0.5f + 0.5f * (di - d) / k, 0.0f, 1.0f);
                d = lerpF(di, d, h) - k * h * (1.0f - h);

                float invDist_i = (dist_i > 0.001f) ? (1.0f / dist_i) : 0.0f;
                float bnx = dx * invDist_i;
                float bny = dy * invDist_i;
                nx = lerpF(bnx, nx, h);
                ny = lerpF(bny, ny, h);

                float wi = 1.0f / (dist_i + 0.1f);
                totalWeight += wi;
                weightedDepth += bdepth[i] * wi;
            }

            if (d >= edgeSoft) {
                gridAlpha[gy][gx] = 0;
                continue;
            }

            float alphaF = smoothstepF(+edgeSoft, -edgeSoft, d);
            gridAlpha[gy][gx] = (uint8_t)(alphaF * 255.0f);

            float pixelDepth = weightedDepth / totalWeight;

            // 归一化解析法线
            float normLen = sqrtf(nx * nx + ny * ny) + 1e-4f;
            nx /= normLen;
            ny /= normLen;

            // 漫反射受光
            float NdotL = nx * lnx + ny * lny;
            float halfLambert = clampF(NdotL * 0.55f + 0.45f, 0.0f, 1.0f);

            // 向光单步快速 AO 探测 (探测左上方是否有遮挡)
            float spX = fpx + occOffsetX;
            float spY = fpy + occOffsetY;
            float spdx0 = spX - bx[0];
            float spdy0 = spY - by[0];
            float dLight = sqrtf(spdx0 * spdx0 + spdy0 * spdy0) - br[0];
            for (int i = 1; i < blobCount; ++i) {
                float spdx = spX - bx[i];
                float spdy = spY - by[i];
                float spdi = sqrtf(spdx * spdx + spdy * spdy) - br[i];
                dLight = smin(dLight, spdi, k);
            }

            float occDelta = (d - dLight);
            float occlusion = clampF(occDelta / 7.0f, 0.0f, 1.0f);
            float bottomFactor = smoothstepF(cloudCenterY - 4.0f, cloudCenterY + 28.0f, fpy) * bottomShade;

            // 细节噪声
            float noise = 0.0f;
            if (noiseAmt > 0.001f) {
                noise = sinf(fpx * 0.22f + fpy * 0.16f) * cosf(fpx * 0.14f - fpy * 0.24f) * noiseAmt;
            }

            // 综合光照合成
            float directLight = halfLambert * lightIntensity;
            float shadowFactor = (1.0f - occlusion * shadowStrength) * (1.0f - bottomFactor);
            float depthMod = pixelDepth * depthInfluence;

            float finalLight = (ambientLight + directLight) * shadowFactor + depthMod + noise;
            finalLight = clampF(finalLight, 0.0f, 1.0f);

            // 调色板颜色计算 (RGB888 空间，色彩纯净无失真)
            uint8_t cr, cg, cb;
            if (finalLight > 0.72f) {
                float t = (finalLight - 0.72f) / 0.28f;
                cr = (uint8_t)lerpF(246.0f, 255.0f, t);
                cg = (uint8_t)lerpF(248.0f, 255.0f, t);
                cb = (uint8_t)lerpF(255.0f, 255.0f, t);
            } else {
                float t = finalLight / 0.72f;
                cr = (uint8_t)lerpF(182.0f, 246.0f, t);
                cg = (uint8_t)lerpF(198.0f, 248.0f, t);
                cb = (uint8_t)lerpF(228.0f, 255.0f, t);
            }

            gridR[gy][gx] = cr;
            gridG[gy][gx] = cg;
            gridB[gy][gx] = cb;
        }
    }

    // 4. 2x2 高速整型双线性插值并写入 Framebuffer
    int groundStart = SCREEN_H - _config.groundHeight;
    uint8_t r_top, g_top, b_top, r_bot, g_bot, b_bot, r_gnd, g_gnd, b_gnd;
    decodeRGB565(_config.skyTopColor, r_top, g_top, b_top);
    decodeRGB565(_config.skyBottomColor, r_bot, g_bot, b_bot);
    decodeRGB565(_config.groundColor, r_gnd, g_gnd, b_gnd);

    for (int py = bbY0; py < bbY1; py += 2) {
        int gy = py / 2;
        int next_py = py + 1;
        uint16_t* row0 = fb + py * SCREEN_W;
        uint16_t* row1 = (next_py < SCREEN_H) ? (fb + next_py * SCREEN_W) : nullptr;

        // 当前行天空背景 RGB888
        uint8_t skyR0, skyG0, skyB0, skyR1, skyG1, skyB1;
        if (py < groundStart) {
            float t0 = (float)py / (float)(groundStart > 0 ? groundStart : 1);
            skyR0 = (uint8_t)lerpF(r_top, r_bot, t0);
            skyG0 = (uint8_t)lerpF(g_top, g_bot, t0);
            skyB0 = (uint8_t)lerpF(b_top, b_bot, t0);
        } else {
            skyR0 = r_gnd; skyG0 = g_gnd; skyB0 = b_gnd;
        }

        if (next_py < groundStart) {
            float t1 = (float)next_py / (float)(groundStart > 0 ? groundStart : 1);
            skyR1 = (uint8_t)lerpF(r_top, r_bot, t1);
            skyG1 = (uint8_t)lerpF(g_top, g_bot, t1);
            skyB1 = (uint8_t)lerpF(b_top, b_bot, t1);
        } else {
            skyR1 = r_gnd; skyG1 = g_gnd; skyB1 = b_gnd;
        }

        for (int px = bbX0; px < bbX1; px += 2) {
            int gx = px / 2;
            int next_px = px + 1;

            uint16_t a00 = gridAlpha[gy][gx];
            uint16_t a10 = gridAlpha[gy][gx + 1];
            uint16_t a01 = gridAlpha[gy + 1][gx];
            uint16_t a11 = gridAlpha[gy + 1][gx + 1];

            if (a00 == 0 && a10 == 0 && a01 == 0 && a11 == 0) {
                continue; // 完全在云外
            }

            uint16_t r00 = gridR[gy][gx], g00 = gridG[gy][gx], b00 = gridB[gy][gx];
            uint16_t r10 = gridR[gy][gx + 1], g10 = gridG[gy][gx + 1], b10 = gridB[gy][gx + 1];
            uint16_t r01 = gridR[gy + 1][gx], g01 = gridG[gy + 1][gx], b01 = gridB[gy + 1][gx];
            uint16_t r11 = gridR[gy + 1][gx + 1], g11 = gridG[gy + 1][gx + 1], b11 = gridB[gy + 1][gx + 1];

            // 像素 (0,0)
            if (a00 >= 250) {
                row0[px] = packRGB565_BE(r00, g00, b00);
            } else if (a00 > 2) {
                uint8_t r = (r00 * a00 + skyR0 * (255 - a00)) / 255;
                uint8_t g = (g00 * a00 + skyG0 * (255 - a00)) / 255;
                uint8_t b = (b00 * a00 + skyB0 * (255 - a00)) / 255;
                row0[px] = packRGB565_BE(r, g, b);
            }

            // 像素 (1,0)
            if (next_px < SCREEN_W) {
                uint16_t a_10 = (a00 + a10) >> 1;
                uint16_t r_10 = (r00 + r10) >> 1;
                uint16_t g_10 = (g00 + g10) >> 1;
                uint16_t b_10 = (b00 + b10) >> 1;
                if (a_10 >= 250) {
                    row0[next_px] = packRGB565_BE(r_10, g_10, b_10);
                } else if (a_10 > 2) {
                    uint8_t r = (r_10 * a_10 + skyR0 * (255 - a_10)) / 255;
                    uint8_t g = (g_10 * a_10 + skyG0 * (255 - a_10)) / 255;
                    uint8_t b = (b_10 * a_10 + skyB0 * (255 - a_10)) / 255;
                    row0[next_px] = packRGB565_BE(r, g, b);
                }
            }

            // 像素 (0,1) 与 (1,1)
            if (row1) {
                uint16_t a_01 = (a00 + a01) >> 1;
                uint16_t r_01 = (r00 + r01) >> 1;
                uint16_t g_01 = (g00 + g01) >> 1;
                uint16_t b_01 = (b00 + b01) >> 1;
                if (a_01 >= 250) {
                    row1[px] = packRGB565_BE(r_01, g_01, b_01);
                } else if (a_01 > 2) {
                    uint8_t r = (r_01 * a_01 + skyR1 * (255 - a_01)) / 255;
                    uint8_t g = (g_01 * a_01 + skyG1 * (255 - a_01)) / 255;
                    uint8_t b = (b_01 * a_01 + skyB1 * (255 - a_01)) / 255;
                    row1[px] = packRGB565_BE(r, g, b);
                }

                if (next_px < SCREEN_W) {
                    uint16_t a_11 = (a00 + a10 + a01 + a11) >> 2;
                    uint16_t r_11 = (r00 + r10 + r01 + r11) >> 2;
                    uint16_t g_11 = (g00 + g10 + g01 + g11) >> 2;
                    uint16_t b_11 = (b00 + b10 + b01 + b11) >> 2;
                    if (a_11 >= 250) {
                        row1[next_px] = packRGB565_BE(r_11, g_11, b_11);
                    } else if (a_11 > 2) {
                        uint8_t r = (r_11 * a_11 + skyR1 * (255 - a_11)) / 255;
                        uint8_t g = (g_11 * a_11 + skyG1 * (255 - a_11)) / 255;
                        uint8_t b = (b_11 * a_11 + skyB1 * (255 - a_11)) / 255;
                        row1[next_px] = packRGB565_BE(r, g, b);
                    }
                }
            }
        }
    }

    uint32_t renderTimeUs = micros() - renderStartUs;

    // 5. Debug 模式信息绘制
    bool showDebug = debugMode || _debugMode;
    if (showDebug) {
        canvas.drawRect(bbX0, bbY0, (bbX1 - bbX0 + 1), (bbY1 - bbY0 + 1), 0xF800);

        for (int i = 0; i < blobCount; ++i) {
            int bx_i = (int)blobs[i].currentX;
            int by_i = (int)blobs[i].currentY;
            uint16_t dotColor = (blobs[i].depth >= 0) ? 0x07E0 : 0x03FF;
            canvas.fillCircle(bx_i, by_i, 2, dotColor);
            canvas.drawCircle(bx_i, by_i, (int)blobs[i].currentRadius, 0x7BEF);
            canvas.setCursor(bx_i + 3, by_i - 3);
            canvas.setTextColor(0x0000);
            canvas.printf("%d", i);
        }

        canvas.fillCircle((int)cloudCenterX, (int)cloudCenterY, 3, 0xF81F);

        int gx0 = 118, gy0 = 18;
        canvas.drawCircle(gx0, gy0, 12, 0xFFFF);
        int gx1 = gx0 + (int)(imu.getGravityX() * 10.0f);
        int gy1 = gy0 + (int)(imu.getGravityY() * 10.0f);
        canvas.drawLine(gx0, gy0, gx1, gy1, 0xF800);
        canvas.fillCircle(gx1, gy1, 2, 0xF800);

        canvas.setTextColor(0xFFFF, 0x0000);
        canvas.setTextSize(1);
        canvas.setCursor(2, 2);
        canvas.printf("FPS:%.1f %dms", fps, (int)(renderTimeUs / 1000));
        canvas.setCursor(2, 12);
        canvas.printf("L:%.2f Sh:%.2f", lightIntensity, shadowStrength);
        canvas.setCursor(2, 22);
        canvas.printf("AO:%.2f N:%.2f", shadowStrength, noiseAmt);
    }

    // 6. 最终推屏呈现
    canvas.pushSprite(0, 0);
}
