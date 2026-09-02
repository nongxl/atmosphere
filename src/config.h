#pragma once
#include <Arduino.h>

// ═══════════════════════════════════════════════════════════════
//  Atmosphere Phase 1 — 全局集中配置与参数管理
// ═══════════════════════════════════════════════════════════════

// 调试模式编译期开关: 1 为默认开启调试信息, 0 为默认关闭
#ifndef CLOUD_DEBUG
#define CLOUD_DEBUG 0
#endif

// 屏幕配置 (M5StickS3 竖屏 135×240)
static constexpr int SCREEN_W            = 135;
static constexpr int SCREEN_H            = 240;
static constexpr uint8_t SCREEN_ROTATION = 0; // 竖屏模式 (逆时针旋转 90°)

// 主循环配置
static constexpr uint32_t TARGET_FPS     = 40;
static constexpr uint32_t FRAME_INTERVAL_MS = 1000 / TARGET_FPS;

// 默认屏幕背光
static constexpr uint8_t SYSTEM_BRIGHTNESS = 60;

// 云物理与视觉集中配置
struct CloudConfig {
    // 基础结构
    static constexpr int BLOB_COUNT = 8;
    float cloudCenterX        = 67.5f;      // 初始云中心 X (屏幕中央 135/2)
    float cloudCenterY        = 90.0f;      // 初始云中心 Y (偏上)
    
    // 弹簧阻尼物理系统
    float stiffness           = 36.0f;      // 弹簧刚度系数
    float damping             = 0.85f;      // 速度阻尼 (0~1)
    
    // 呼吸动画
    float breathingSpeed      = 1.6f;       // 呼吸频率 (rad/s)
    float breathingAmplitude  = 1.8f;       // 半径波动幅度 (像素)
    
    // IMU 重力与形变
    float gravityInfluenceX   = 24.0f;      // 重力对云整体位移的最大像素偏移 (X轴)
    float gravityInfluenceY   = 42.0f;      // 重力对云整体位移的最大像素偏移 (Y轴)
    float deformationAmount   = 0.55f;      // 各 Blob 差异化拉伸形变强度
    float shakeThreshold      = 1.5f;       // 晃动判定阈值 (G)
    float shakeImpulse        = 22.0f;      // 晃动注入的最大扰动冲量
    
    // 渲染与视觉表现
    float sminBlendK          = 13.0f;      // Smooth Union 平滑融合半径
    float edgeSoftness        = 2.0f;       // 边缘抗锯齿与过渡宽度 (像素)
    float noiseAmount         = 0.035f;     // 内部微弱低频纹理扰动
    
    // 光照参数 (来自左上方)
    float lightDirX           = -0.6f;
    float lightDirY           = -0.8f;
    
    // 背景色彩 (RGB565)
    uint16_t skyTopColor      = 0x22DC;     // 顶部深邃天蓝
    uint16_t skyBottomColor   = 0x9E7F;     // 底部柔和浅天蓝
    uint16_t groundColor      = 0x7E0B;     // 底部草地基底
    int groundHeight          = 26;         // 底部地面高度 (像素)
};

// 辅助数学函数
inline float clampF(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

inline float lerpF(float a, float b, float t) {
    return a + (b - a) * t;
}

// 柔和 Smoothstep 计算
inline float smoothstepF(float edge0, float edge1, float x) {
    float t = clampF((x - edge0) / (edge1 - edge0), 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}

// 颜色转换工具: RGB565 分量解构与重建
inline void decodeRGB565(uint16_t c, uint8_t& r, uint8_t& g, uint8_t& b) {
    r = (c >> 8) & 0xF8;
    r |= (r >> 5);
    g = (c >> 3) & 0xFC;
    g |= (g >> 6);
    b = (c << 3) & 0xF8;
    b |= (b >> 5);
}

inline uint16_t encodeRGB565(uint8_t r, uint8_t g, uint8_t b) {
    return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

// 两个 RGB565 颜色线性插值
inline uint16_t lerpColor565(uint16_t c1, uint16_t c2, float t) {
    uint8_t r1, g1, b1, r2, g2, b2;
    decodeRGB565(c1, r1, g1, b1);
    decodeRGB565(c2, r2, g2, b2);
    uint8_t r = (uint8_t)clampF(r1 + (r2 - r1) * t, 0.0f, 255.0f);
    uint8_t g = (uint8_t)clampF(g1 + (g2 - g1) * t, 0.0f, 255.0f);
    uint8_t b = (uint8_t)clampF(b1 + (b2 - b1) * t, 0.0f, 255.0f);
    return encodeRGB565(r, g, b);
}
