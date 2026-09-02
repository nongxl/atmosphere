#pragma once
#include "cloud_blob.h"
#include "config.h"

// ═══════════════════════════════════════════════════════════════
//  Atmosphere Phase 1 — 云朵物理动力学与软体形变引擎
// ═══════════════════════════════════════════════════════════════

class CloudPhysics {
public:
    CloudPhysics();

    void init(const CloudConfig& config);
    void update(float dt, float gravityX, float gravityY, float shakeImpulse);

    // 获取 Blob 数组指针及数量
    const CloudBlob* getBlobs() const { return _blobs; }
    int getBlobCount() const { return _config.BLOB_COUNT; }

    // 获取当前云整体中心
    float getCenterX() const { return _currentCenterX; }
    float getCenterY() const { return _currentCenterY; }

    // 获取配置引用以便动态调节
    CloudConfig& getConfig() { return _config; }
    const CloudConfig& getConfig() const { return _config; }

private:
    CloudConfig _config;
    CloudBlob   _blobs[CloudConfig::BLOB_COUNT];
    
    float _timeAccumulator;
    float _currentCenterX;
    float _currentCenterY;
    float _centerVx;
    float _centerVy;
};
