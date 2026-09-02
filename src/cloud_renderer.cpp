#include "cloud_renderer.h"
#include <math.h>

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

void CloudRenderer::precomputeBackground() {
    int groundStart = SCREEN_H - _config.groundHeight;
    for (int y = 0; y < SCREEN_H; ++y) {
        if (y < groundStart) {
            // 天空垂直线性渐变 (顶部深蓝 -> 底部浅天蓝)
            float t = (float)y / (float)(groundStart > 0 ? groundStart : 1);
            _skyRowColors[y] = lerpColor565(_config.skyTopColor, _config.skyBottomColor, t);
        } else {
            // 地面草地基底
            _skyRowColors[y] = _config.groundColor;
        }
    }
}

void CloudRenderer::render(M5Canvas& canvas, const CloudPhysics& physics, const IMUController& imu, float fps, bool debugMode) {
    uint32_t renderStartUs = micros();

    const CloudBlob* blobs = physics.getBlobs();
    int blobCount = physics.getBlobCount();
    float cloudCenterX = physics.getCenterX();
    float cloudCenterY = physics.getCenterY();

    // 1. 快速绘制全屏预计算背景 (按行填充或双缓冲清屏)
    for (int y = 0; y < SCREEN_H; ++y) {
        canvas.drawFastHLine(0, y, SCREEN_W, _skyRowColors[y]);
    }

    // 2. 计算云朵外接包围盒 (Bounding Box)
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

    // 边距外扩 padding，用于 smooth union 融合区与柔化边缘
    float padding = _config.sminBlendK + _config.edgeSoftness + 4.0f;
    int bbX0 = (int)floorf(clampF(minX - padding, 0.0f, (float)(SCREEN_W - 1)));
    int bbX1 = (int)ceilf(clampF(maxX + padding, 0.0f, (float)(SCREEN_W - 1)));
    int bbY0 = (int)floorf(clampF(minY - padding, 0.0f, (float)(SCREEN_H - 1)));
    int bbY1 = (int)ceilf(clampF(maxY + padding, 0.0f, (float)(SCREEN_H - 1)));

    // 3. 在 Bounding Box 内部逐像素计算 SDF 密度场与光照着色
    float k = _config.sminBlendK;
    float edgeSoft = _config.edgeSoftness;
    float noiseAmt = _config.noiseAmount;

    // 阴影与高光基础色 (RGB565 分解)
    // 高光纯白: (255, 255, 255)
    // 云体柔白: (248, 250, 252)
    // 阴影淡蓝灰: (218, 228, 240)
    for (int py = bbY0; py <= bbY1; ++py) {
        float fpy = (float)py;
        uint16_t skyColor = _skyRowColors[py];

        for (int px = bbX0; px <= bbX1; ++px) {
            float fpx = (float)px;

            // a. 计算像素到每个 blob 的 SDF 距离，并用多项式 smin 平滑融合
            float dx0 = fpx - blobs[0].currentX;
            float dy0 = fpy - blobs[0].currentY;
            float d = sqrtf(dx0 * dx0 + dy0 * dy0) - blobs[0].currentRadius;

            for (int i = 1; i < blobCount; ++i) {
                float dx = fpx - blobs[i].currentX;
                float dy = fpy - blobs[i].currentY;
                float di = sqrtf(dx * dx + dy * dy) - blobs[i].currentRadius;
                d = smin(d, di, k);
            }

            // b. 边缘柔化判定 (Soft Edge Smoothstep)
            // d < -edgeSoft 为完全内部 (alpha = 1.0)
            // d > +edgeSoft 为完全外部 (alpha = 0.0)
            if (d >= edgeSoft) {
                continue; // 完全在云外，保持背景
            }

            float alpha = smoothstepF(+edgeSoft, -edgeSoft, d);

            // c. 柔和体积光照与立体着色 (Lighting)
            // 光源来自左上方 (-0.6, -0.8)
            // 相对云中心方向分量与高度占比 (适配竖屏云朵尺度)
            float relX = (fpx - cloudCenterX) / 42.0f;
            float relY = (fpy - cloudCenterY) / 22.0f;
            
            // 内部深度
            float depth = clampF(-d / 13.0f, 0.0f, 1.0f);

            // 光照强度计算: 上部向阳、左侧亮、底部阴影
            float light = 0.55f - (relY * 0.45f) - (relX * 0.18f) + (depth * 0.20f);
            light = clampF(light, 0.0f, 1.0f);

            // 微弱低频纹理 (打破纯白死板，增加柔软棉花纤维感)
            if (noiseAmt > 0.001f) {
                float n = sinf(fpx * 0.22f + fpy * 0.16f) * cosf(fpx * 0.14f - fpy * 0.24f);
                light += n * noiseAmt;
                light = clampF(light, 0.0f, 1.0f);
            }

            // 依据光照强度在淡冷蓝灰(阴影) -> 柔白(云身) -> 亮白(高光)之间渐变
            uint8_t cr, cg, cb;
            if (light > 0.6f) {
                // 亮部向高光白过渡
                float t = (light - 0.6f) / 0.4f;
                cr = (uint8_t)lerpF(246.0f, 255.0f, t);
                cg = (uint8_t)lerpF(248.0f, 255.0f, t);
                cb = (uint8_t)lerpF(252.0f, 255.0f, t);
            } else {
                // 暗部淡蓝灰向柔白过渡
                float t = light / 0.6f;
                cr = (uint8_t)lerpF(216.0f, 246.0f, t);
                cg = (uint8_t)lerpF(226.0f, 248.0f, t);
                cb = (uint8_t)lerpF(240.0f, 252.0f, t);
            }

            uint16_t cloudColor = encodeRGB565(cr, cg, cb);

            // d. 边缘与背景 Alpha 混合
            uint16_t finalColor;
            if (alpha >= 0.99f) {
                finalColor = cloudColor;
            } else {
                finalColor = lerpColor565(skyColor, cloudColor, alpha);
            }

            canvas.drawPixel(px, py, finalColor);
        }
    }

    uint32_t renderTimeUs = micros() - renderStartUs;

    // 4. Debug 模式信息绘制
    bool showDebug = debugMode || _debugMode;
    if (showDebug) {
        // 绘制 Bounding Box
        canvas.drawRect(bbX0, bbY0, (bbX1 - bbX0 + 1), (bbY1 - bbY0 + 1), 0xF800); // 红色线框

        // 绘制各 Blob 中心点及连线
        for (int i = 0; i < blobCount; ++i) {
            int bx = (int)blobs[i].currentX;
            int by = (int)blobs[i].currentY;
            canvas.fillCircle(bx, by, 2, 0x07E0); // 绿色圆点
            canvas.drawCircle(bx, by, (int)blobs[i].currentRadius, 0x7BEF); // 半径圆圈
            canvas.setCursor(bx + 3, by - 3);
            canvas.setTextColor(0x0000);
            canvas.printf("%d", i);
        }

        // 绘制云中心
        canvas.fillCircle((int)cloudCenterX, (int)cloudCenterY, 3, 0xF81F);

        // 绘制 IMU 重力方向指示器 (右上角小仪表, 适配竖屏 135 宽)
        int gx0 = 118, gy0 = 18;
        canvas.drawCircle(gx0, gy0, 12, 0xFFFF);
        int gx1 = gx0 + (int)(imu.getGravityX() * 10.0f);
        int gy1 = gy0 + (int)(imu.getGravityY() * 10.0f);
        canvas.drawLine(gx0, gy0, gx1, gy1, 0xF800);
        canvas.fillCircle(gx1, gy1, 2, 0xF800);

        // 打印遥测与性能文字 (适配竖屏宽度)
        canvas.setTextColor(0xFFFF, 0x0000);
        canvas.setTextSize(1);
        canvas.setCursor(2, 2);
        canvas.printf("FPS:%.1f %dms", fps, (int)(renderTimeUs / 1000));
        canvas.setCursor(2, 12);
        canvas.printf("G:(%+.2f,%+.2f)", imu.getGravityX(), imu.getGravityY());
        canvas.setCursor(2, 22);
        canvas.printf("Shake:%.2f", imu.getShakeImpulse());
    }

    // 5. 最终推屏呈现
    canvas.pushSprite(0, 0);
}
