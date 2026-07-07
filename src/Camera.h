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
    // 以网格的中心 (7.5, 7.5) 为轴旋转，实现真正的摄像机环绕观察，而不是平移
    void project(float x, float y, float z, int& screenX, int& screenY) const {
        // 1. 绕网格中心 (7.5, 7.5) 旋转 azimuth
        float dx = x - 7.5f;
        float dy = y - 7.5f;

        if (azimuth != _lastAzimuth) {
            _lastAzimuth = azimuth;
            _cachedCosA = cosf(azimuth);
            _cachedSinA = sinf(azimuth);
        }
        float rx = dx * _cachedCosA - dy * _cachedSinA + 7.5f;
        float ry = dx * _cachedSinA + dy * _cachedCosA + 7.5f;

        // 2. 等轴投影：elevation 调整仰角系数（默认 0.5，范围约 0.15~0.85）
        //    elevation>0 = 视角更俯视（Y贡献更大），elevation<0 = 更侧视
        float elevFactor = 0.5f + elevation * 0.35f;
        if (elevFactor < 0.15f) elevFactor = 0.15f;
        if (elevFactor > 0.85f) elevFactor = 0.85f;

        screenX = (int)(centerX + (rx - ry) * 0.866025f * scale);
        screenY = (int)(centerY + (rx + ry) * elevFactor * scale - z * heightScale);
    }
};
