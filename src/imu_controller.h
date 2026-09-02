#pragma once
#include <M5Unified.h>
#include "config.h"

// ═══════════════════════════════════════════════════════════════
//  Atmosphere Phase 1 — IMU 控制与重力滤波模块
// ═══════════════════════════════════════════════════════════════

class IMUController {
public:
    IMUController();

    bool begin();
    void update(float dt);

    // 获取归一化滤波重力向量 (-1.0 ~ +1.0)
    float getGravityX() const { return _filteredGravityX; }
    float getGravityY() const { return _filteredGravityY; }

    // 获取当前晃动/冲击冲量强度
    float getShakeImpulse() const { return _shakeImpulse; }

    // 手动触发一次冲量扰动 (用于按键测试)
    void triggerImpulse(float strength = 1.0f);

    // 传感器是否就绪
    bool isReady() const { return _ready; }

private:
    bool  _ready;
    float _filteredGravityX;
    float _filteredGravityY;
    
    float _prevRawX;
    float _prevRawY;
    float _prevRawZ;
    
    float _shakeImpulse;
};
