#pragma once
#include <math.h>

class Camera {
public:
    float centerX;
    float centerY;
    float scale;
    float heightScale;

    // IMU 驱动的视角参数
    float azimuth;    // 水平旋转角 (弧度)，0 = 默认等轴视角
    float elevation;  // 仰角偏移 (弧度)，0 = 默认 30° 俯视

    mutable float _lastAzimuth = -999.0f;
    mutable float _cachedCosA = 1.0f;
    mutable float _cachedSinA = 0.0f;

    Camera(float cx = 67.5f, float cy = 90.0f, float s = 4.3f, float hs = 5.0f)
        : centerX(cx), centerY(cy), scale(s), heightScale(hs),
          azimuth(0.0f), elevation(0.0f) {}

    // 将 3D 物理网格坐标 (x, y, z) 转换为 2D 屏幕坐标 (screenX, screenY)
    // 透过窗户观察模型：azimuth旋转视角，elevation调整仰角
    void project(float x, float y, float z, int& screenX, int& screenY) const {
        float dx = x - 7.5f;
        float dy = y - 7.5f;

        // 绕网格中心旋转 azimuth
        if (azimuth != _lastAzimuth) {
            _lastAzimuth = azimuth;
            _cachedCosA = cosf(azimuth);
            _cachedSinA = sinf(azimuth);
        }
        float rx = dx * _cachedCosA - dy * _cachedSinA + 7.5f;
        float ry = dx * _cachedSinA + dy * _cachedCosA + 7.5f;

        // 等轴投影：elevation 调整仰角系数（默认 0.5，范围约 0.15~0.85）
        float elevFactor = 0.5f + elevation * 0.35f;
        if (elevFactor < 0.15f) elevFactor = 0.15f;
        if (elevFactor > 0.85f) elevFactor = 0.85f;

        screenX = (int)(centerX + (rx - ry) * 0.866025f * scale);
        screenY = (int)(centerY + (rx + ry) * elevFactor * scale - z * heightScale);
    }
};
