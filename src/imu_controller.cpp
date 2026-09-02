#include "imu_controller.h"

IMUController::IMUController()
    : _ready(false)
    , _filteredGravityX(0.0f)
    , _filteredGravityY(0.0f)
    , _prevRawX(0.0f)
    , _prevRawY(0.0f)
    , _prevRawZ(0.0f)
    , _shakeImpulse(0.0f)
{
}

bool IMUController::begin() {
    _ready = M5.Imu.isEnabled();
    return _ready;
}

void IMUController::update(float dt) {
    if (!_ready) {
        // 如果硬件 IMU 未就绪，尝试重新检测
        _ready = M5.Imu.isEnabled();
        if (!_ready) return;
    }

    float ax = 0.0f, ay = 0.0f, az = 0.0f;
    if (!M5.Imu.getAccel(&ax, &ay, &az)) {
        return;
    }

    // 坐标映射适配：在 M5StickS3 竖屏模式下：
    // 短边水平横向对应 ay，长边垂直纵向对应 ax
    float screenRawX = ay;
    float screenRawY = ax;

    // 低通滤波 (一阶 IIR 滤波器)，平滑滤除高频抖动
    // 响应时间常数约为 0.12s
    float alpha = clampF(dt * 8.0f, 0.01f, 1.0f);
    _filteredGravityX += (screenRawX - _filteredGravityX) * alpha;
    _filteredGravityY += (screenRawY - _filteredGravityY) * alpha;

    // 限制在 -1.0 ~ +1.0 G 范围内
    _filteredGravityX = clampF(_filteredGravityX, -1.0f, 1.0f);
    _filteredGravityY = clampF(_filteredGravityY, -1.0f, 1.0f);

    // 晃动检测 (Jerk / 加速度变化率)
    float deltaX = ax - _prevRawX;
    float deltaY = ay - _prevRawY;
    float deltaZ = az - _prevRawZ;
    float jerkSq = deltaX * deltaX + deltaY * deltaY + deltaZ * deltaZ;

    _prevRawX = ax;
    _prevRawY = ay;
    _prevRawZ = az;

    // 阈值判定
    if (jerkSq > (1.3f * 1.3f)) {
        float strength = sqrtf(jerkSq) - 1.3f;
        _shakeImpulse = clampF(_shakeImpulse + strength * 4.0f, 0.0f, 2.5f);
    }

    // 冲量衰减
    _shakeImpulse *= powf(0.08f, dt);
    if (_shakeImpulse < 0.01f) {
        _shakeImpulse = 0.0f;
    }
}

void IMUController::triggerImpulse(float strength) {
    _shakeImpulse = clampF(_shakeImpulse + strength, 0.0f, 3.0f);
}
