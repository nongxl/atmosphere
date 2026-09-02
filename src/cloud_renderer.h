#pragma once
#include <M5Unified.h>
#include "config.h"
#include "cloud_blob.h"
#include "cloud_physics.h"
#include "imu_controller.h"

// ═══════════════════════════════════════════════════════════════
//  Atmosphere Phase 1.5 — 增强阴影与体积感软云渲染器
// ═══════════════════════════════════════════════════════════════

class CloudRenderer {
public:
    CloudRenderer();

    void init(M5Canvas& canvas, const CloudConfig& config);
    void render(M5Canvas& canvas, const CloudPhysics& physics, const IMUController& imu, float fps, bool debugMode);

    // 切换调试显示
    void toggleDebug() { _debugMode = !_debugMode; }
    bool isDebug() const { return _debugMode; }

    // 光照预设切换
    void applyPreset(CloudLightPreset preset);

private:
    CloudConfig _config;
    bool        _debugMode;
    uint16_t    _skyRowColors[SCREEN_H]; // 预计算每行天空/地面背景色以极大提升渲染性能

    void precomputeBackground();
    
    // 多项式平滑最小融合函数 (Polynomial Smooth Minimum)
    static inline float smin(float a, float b, float k) {
        float h = clampF(0.5f + 0.5f * (b - a) / k, 0.0f, 1.0f);
        return lerpF(b, a, h) - k * h * (1.0f - h);
    }

    // 连续 SDF 距离场估算
    static inline float evalSDF(float px, float py, const CloudBlob* blobs, int blobCount, float k) {
        float dx0 = px - blobs[0].currentX;
        float dy0 = py - blobs[0].currentY;
        float d = sqrtf(dx0 * dx0 + dy0 * dy0) - blobs[0].currentRadius;
        for (int i = 1; i < blobCount; ++i) {
            float dx = px - blobs[i].currentX;
            float dy = py - blobs[i].currentY;
            float di = sqrtf(dx * dx + dy * dy) - blobs[i].currentRadius;
            d = smin(d, di, k);
        }
        return d;
    }
};
