#pragma once
#include <Arduino.h>
#include "config.h"

// ═══════════════════════════════════════════════════════════════
//  传感器模块 — SHT30（温湿度）+ QMP6988（气压）+ 内置麦克风（声压分贝）
//  内部维护气压历史环形缓冲区，提供气压变化趋势（线性回归）
//  麦克风以 8kHz 采样 64 帧（≈8ms），计算 RMS 转 dBFS
// ═══════════════════════════════════════════════════════════════

namespace Sensors {

bool  init();
void  update();           // 每帧调用，内部节流到 SENSOR_INTERVAL_MS

float getTemperature();   // °C
float getHumidity();      // % RH
float getPressure();      // hPa
float getPressureTrend(); // hPa/min（负值 = 气压下降）

bool  isReady();

// ── 麦克风 ────────────────────────────────────────────────────
void  initMic();          // 在 setup() 中调用一次
void  updateMic();        // 每帧调用，内部节流（每 6 帧采样一次）
float getMicDb();         // 返回平滑后的分贝值（典型 30~90 dB）

} // namespace Sensors
