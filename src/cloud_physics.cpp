#include "cloud_physics.h"
#include <math.h>

CloudPhysics::CloudPhysics()
    : _timeAccumulator(0.0f)
    , _currentCenterX(120.0f)
    , _currentCenterY(56.0f)
    , _centerVx(0.0f)
    , _centerVy(0.0f)
{
}

void CloudPhysics::init(const CloudConfig& config) {
    _config = config;
    _currentCenterX = _config.cloudCenterX;
    _currentCenterY = _config.cloudCenterY;
    _centerVx = 0.0f;
    _centerVy = 0.0f;
    _timeAccumulator = 0.0f;

    // 经典 8-Blob 蓬松立体卡通云朵拓扑布局 (含前后景 Depth 层次)
    struct BlobInit {
        float bx, by;
        float radius;
        float phase;
        float depth;
        float factorX, factorY;
    };

    static const BlobInit initLayout[CloudConfig::BLOB_COUNT] = {
        {   0.0f,   1.5f, 15.0f, 0.00f,  0.60f, 1.00f, 1.00f }, // 0: 云心主核 (前景)
        { -17.0f,   3.0f, 12.0f, 1.15f,  0.85f, 1.12f, 0.95f }, // 1: 左中主凸起 (向阳最前景)
        {  18.0f,   3.5f, 12.5f, 2.30f,  0.40f, 1.18f, 1.05f }, // 2: 右中主凸起 (前中景)
        {  -5.0f,  -9.5f, 13.0f, 3.45f,  0.75f, 0.92f, 1.20f }, // 3: 顶部主峰 (向阳光照前景)
        {  10.0f,  -6.5f, 11.0f, 4.60f, -0.20f, 1.05f, 1.10f }, // 4: 右上次峰 (中后景)
        { -30.0f,   5.0f,  9.0f, 5.75f, -0.45f, 1.30f, 0.85f }, // 5: 最左侧软翼 (侧后景)
        {  31.0f,   5.5f,  9.5f, 0.80f, -0.65f, 1.35f, 0.90f }, // 6: 最右侧软翼 (背阳后景)
        {   1.0f,   8.5f, 11.0f, 2.05f, -0.80f, 1.00f, 0.80f }  // 7: 底部平坦支撑垫 (后景底垫)
    };

    for (int i = 0; i < CloudConfig::BLOB_COUNT; ++i) {
        _blobs[i].baseX = initLayout[i].bx;
        _blobs[i].baseY = initLayout[i].by;
        _blobs[i].baseRadius = initLayout[i].radius;
        _blobs[i].currentRadius = initLayout[i].radius;
        _blobs[i].phase = initLayout[i].phase;
        _blobs[i].depth = initLayout[i].depth;
        _blobs[i].gravityFactorX = initLayout[i].factorX;
        _blobs[i].gravityFactorY = initLayout[i].factorY;

        _blobs[i].currentX = _currentCenterX + _blobs[i].baseX;
        _blobs[i].currentY = _currentCenterY + _blobs[i].baseY;
        _blobs[i].targetX  = _blobs[i].currentX;
        _blobs[i].targetY  = _blobs[i].currentY;
        _blobs[i].vx = 0.0f;
        _blobs[i].vy = 0.0f;
    }
}

void CloudPhysics::update(float dt, float gravityX, float gravityY, float shakeImpulse) {
    if (dt <= 0.0001f) return;
    if (dt > 0.1f) dt = 0.1f; // 限制单步最大 dt 防止积分发散

    _timeAccumulator += dt;

    // 1. 计算云整体中心的目标位置 (受重力倾斜驱动)
    float targetCenterX = _config.cloudCenterX + gravityX * _config.gravityInfluenceX;
    float targetCenterY = _config.cloudCenterY + gravityY * _config.gravityInfluenceY;

    // 限制整体中心在安全屏幕区域内
    targetCenterX = clampF(targetCenterX, 42.0f, SCREEN_W - 42.0f);
    targetCenterY = clampF(targetCenterY, 35.0f, SCREEN_H - 50.0f);

    // 云中心二阶弹簧-阻尼系统模拟
    float centerForceX = (targetCenterX - _currentCenterX) * _config.stiffness;
    float centerForceY = (targetCenterY - _currentCenterY) * _config.stiffness;

    // 注入晃动冲量扰动
    if (shakeImpulse > 0.01f) {
        float shakeAngle = _timeAccumulator * 12.0f;
        centerForceX += cosf(shakeAngle) * shakeImpulse * _config.shakeImpulse;
        centerForceY += sinf(shakeAngle) * shakeImpulse * _config.shakeImpulse * 0.7f;
    }

    _centerVx += centerForceX * dt;
    _centerVy += centerForceY * dt;

    // 阻尼衰减
    float dampingFactor = powf(_config.damping, dt * 60.0f);
    _centerVx *= dampingFactor;
    _centerVy *= dampingFactor;

    _currentCenterX += _centerVx * dt;
    _currentCenterY += _centerVy * dt;

    // 2. 更新每个单独 Blob 的呼吸动画与局部软体形变
    for (int i = 0; i < CloudConfig::BLOB_COUNT; ++i) {
        CloudBlob& b = _blobs[i];

        // 异步呼吸: 独立 phase 使得每个 blob 差异化伸缩
        float breathCycle = sinf(_timeAccumulator * _config.breathingSpeed + b.phase);
        b.currentRadius = b.baseRadius + breathCycle * _config.breathingAmplitude;

        // 局部重力形变: 处于重力下风向的 blob 会轻微被拉伸延展
        float deformX = gravityX * (b.gravityFactorX - 1.0f) * _config.gravityInfluenceX * _config.deformationAmount;
        float deformY = gravityY * (b.gravityFactorY - 1.0f) * _config.gravityInfluenceY * _config.deformationAmount;

        // 晃动的微小相位抖动
        if (shakeImpulse > 0.01f) {
            float blobShake = sinf(_timeAccumulator * 16.0f + b.phase) * shakeImpulse * 3.0f;
            deformY += blobShake;
        }

        // 该 Blob 的世界目标坐标
        b.targetX = _currentCenterX + b.baseX + deformX;
        b.targetY = _currentCenterY + b.baseY + deformY;

        // Blob 独立的局部弹簧跟踪 (增加柔软弹性滞后感)
        float blobStiffness = _config.stiffness * 1.25f;
        float blobForceX = (b.targetX - b.currentX) * blobStiffness;
        float blobForceY = (b.targetY - b.currentY) * blobStiffness;

        b.vx += blobForceX * dt;
        b.vy += blobForceY * dt;

        b.vx *= dampingFactor;
        b.vy *= dampingFactor;

        b.currentX += b.vx * dt;
        b.currentY += b.vy * dt;
    }
}
