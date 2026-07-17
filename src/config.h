#pragma once
#include <Arduino.h>

//── 屏幕尺寸 ─────────────────────────────────────────────────────
static constexpr int SCREEN_W     = 135;
static constexpr int SCREEN_H     = 240;

//── 布局分区 ─────────────────────────────────────────────────────
static constexpr int SKY_AREA_H   = 195;  // 天空+云区高度
static constexpr int STATUS_BAR_Y = 195;  // 状态栏起始 Y
static constexpr int STATUS_BAR_H = 45;   // 状态栏高度

//── I2C — ENV HAT III 引脚 ────────────────────────────────────────
//  I2C_SCL → G0,  I2C_SDA → G8
static constexpr int HAT_SCL      = 0;    // G0
static constexpr int HAT_SDA      = 8;    // G8

//── 传感器采样 ───────────────────────────────────────────────────
static constexpr uint32_t SENSOR_INTERVAL_MS   = 20000;  // 20 秒一次
static constexpr int      PRESSURE_HISTORY_LEN = 72;    // 72×5s = 6 min

//── CloudBlob 系统 ────────────────────────────────────────────────
static constexpr int MAX_CLOUD_BLOBS  = 20;   // 云核数量上限
static constexpr int MAX_RAIN_DROPS   = 100;  // 雨滴（来自云底）

//── 2D 风场网格 ───────────────────────────────────────────────────
static constexpr int WIND_FIELD_W     = 16;   // 横向格数
static constexpr int WIND_FIELD_H     = 24;   // 纵向格数
// 每格约 8.4 × 8.1 px

//── 时间压缩（现实 10 min ≈ 模拟 24 h） ─────────────────────────
static constexpr float TIME_SCALE     = 144.0f;

//── 显示 ─────────────────────────────────────────────────────────
static constexpr uint8_t SYSTEM_BRIGHTNESS = 60;

//── 帧率目标（~30 fps）──────────────────────────────────────────
static constexpr uint32_t FRAME_INTERVAL_MS = 33;

//── 工具函数 ─────────────────────────────────────────────────────
inline float clampF(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

//── 闪电系统 ──────────────────────────────────────────────────────
// 电荷超过此阈值时触发闪电，之后保留 40% 电荷
static constexpr float LIGHTNING_CHARGE_THRESHOLD = 35.0f;  // 降低阈值，更容易触发闪电
static constexpr float LIGHTNING_CHARGE_DECAY     = 0.40f; // 放电后保留比例提高
