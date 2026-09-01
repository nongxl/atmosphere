#pragma once
#include <math.h>

class Camera {
public:
    float centerX;
    float centerY;
    float scale;
    float heightScale;

    // IMU 驱动的全息视窗参数
    float azimuth;    // 水平旋转角 (弧度)，0 = 默认等轴视角
    float elevation;  // 仰角偏移 (弧度)，0 = 默认 30° 俯视
    float panX;       // 水平视差平移量 (像素)
    float panY;       // 垂直视差平移量 (像素)

    mutable float _lastAzimuth = -999.0f;
    mutable float _cachedCosA = 1.0f;
    mutable float _cachedSinA = 0.0f;

    Camera(float cx = 67.5f, float cy = 90.0f, float s = 4.3f, float hs = 5.0f)
        : centerX(cx), centerY(cy), scale(s), heightScale(hs),
          azimuth(0.0f), elevation(0.0f), panX(0.0f), panY(0.0f) {}

    // 将 3D 物理网格坐标 (x, y, z) 转换为 2D 屏幕坐标 (screenX, screenY)
    // 全息视窗模型：结合 3D 旋转 + 深度透视 + 视差平移
    void project(float x, float y, float z, int& screenX, int& screenY) const {
        const float gridCenterX = 7.5f;
        const float gridCenterY = 7.5f;
        
        float dx = x - gridCenterX;
        float dy = y - gridCenterY;

        // 绕网格中心连续平滑旋转 azimuth
        if (azimuth != _lastAzimuth) {
            _lastAzimuth = azimuth;
            _cachedCosA = cosf(azimuth);
            _cachedSinA = sinf(azimuth);
        }
        float rx = dx * _cachedCosA - dy * _cachedSinA + gridCenterX;
        float ry = dx * _cachedSinA + dy * _cachedCosA + gridCenterY;

        // 等轴仰角系数（基准 0.5，根据 elevation 平滑微调）
        float elevFactor = 0.5f + elevation * 0.25f;
        if (elevFactor < 0.20f) elevFactor = 0.20f;
        if (elevFactor > 0.80f) elevFactor = 0.80f;

        // 结合等轴测投影与 IMU 全息视差平移 (panX, panY)
        screenX = (int)roundf(centerX + panX + (rx - ry) * 0.866025f * scale);
        screenY = (int)roundf(centerY + panY + (rx + ry) * elevFactor * scale - z * heightScale);
    }
};
