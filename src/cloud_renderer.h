#pragma once
#include <M5Unified.h>
#include "config.h"
#include "cloud_blob.h"
#include "cloud_physics.h"
#include "imu_controller.h"

// ═══════════════════════════════════════════════════════════════
//  Atmosphere Phase 1 — 软云渲染器 (SDF / Metaball + 光照)
// ═══════════════════════════════════════════════════════════════

class CloudRenderer {
public:
    CloudRenderer();

    void init(M5Canvas& canvas, const CloudConfig& config);
    void render(M5Canvas& canvas, const CloudPhysics& physics, const IMUController& imu, float fps, bool debugMode);

    // 切换调试显示
    void toggleDebug() { _debugMode = !_debugMode; }
    bool isDebug() const { return _debugMode; }

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
};
