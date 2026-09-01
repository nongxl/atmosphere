#pragma once
#include <math.h>

class Camera {
public:
    float centerX;
    float centerY;
    float scale;
    float heightScale;

    // IMU 驱动的 3D 视角旋转参数
    float azimuth;    // 水平旋转角偏移 (弧度)，0 = 正中
    float elevation;  // 俯仰角偏移 (弧度)，0 = 默认 32° 俯视

    // 默认观察方位角
    static constexpr float BASE_AZIMUTH = 0.785398f; // 45度 等轴基准方位角

    Camera(float cx = 67.5f, float cy = 100.0f, float s = 4.3f, float hs = 5.0f)
        : centerX(cx), centerY(cy), scale(s), heightScale(hs),
          azimuth(0.0f), elevation(0.60f) {}

    // 将 3D 物理网格坐标 (x, y, z) 转换为 2D 屏幕坐标 (screenX, screenY)
    // 采用标准 3D 环绕相机模型 (Orbit Camera)：绕网格物理中心纯 3D 空间旋转
    void project(float x, float y, float z, int& screenX, int& screenY) const {
        const float gridCenterX = 7.5f;
        const float gridCenterY = 7.5f;
        const float gridCenterZ = 5.5f;
        
        // 1. 相对中心相对坐标
        float dx = x - gridCenterX;
        float dy = y - gridCenterY;
        float dz = (z - gridCenterZ) * 1.15f; // 纵向高度比例调节

        // 2. 第一步：绕垂直轴旋转方位角 (Base + Azimuth)
        float totalAzimuth = BASE_AZIMUTH + azimuth;
        float cosA = cosf(totalAzimuth);
        float sinA = sinf(totalAzimuth);
        float x1 = dx * cosA - dy * sinA;
        float y1 = dx * sinA + dy * cosA;
        float z1 = dz;

        // 3. 第二步：绕水平轴旋转俯仰角 (elevation: 0.05 纯平视 -> 1.25 高空俯视)
        float elev = elevation;
        if (elev < 0.05f) elev = 0.05f; // 允许水平平视 (3°)
        if (elev > 1.25f) elev = 1.25f; // 防止垂直朝下 (72°)
        float cosE = cosf(elev);
        float sinE = sinf(elev);

        // 正统 3D 观察投影：
        // 平视时 (elev ≈ 0): sinE=0, cosE=1 -> projY = -dz (纯高度投影，与云水平平视！)
        // 俯视时 (elev > 0): 远景 y1 向上展开，展现立体深度
        float projY = -(y1 * sinE + dz * cosE);

        // 4. 映射到屏幕中心
        screenX = (int)roundf(centerX + x1 * 0.92f * scale);
        screenY = (int)roundf(centerY + projY * scale);
    }
};
