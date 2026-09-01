#pragma once
#include <math.h>

class Camera {
public:
    float centerX;
    float centerY;
    float scale;
    float heightScale;

    // 3D 轨道球相机视角参数 (绕云中心纯旋转观察)
    float azimuth;    // 水平环视旋转角 (弧度): 0 = 正面, 负值 = 观察左侧, 正值 = 观察右侧
    float elevation;  // 垂直俯仰角 (弧度): 0 = 正平视, 正值 = 前倾俯视看云顶, 负值 = 后倾仰视看云底

    mutable float _lastAzimuth = -999.0f;
    mutable float _lastElevation = -999.0f;
    mutable float _cachedCosA = 1.0f;
    mutable float _cachedSinA = 0.0f;
    mutable float _cachedCosE = 1.0f;
    mutable float _cachedSinE = 0.0f;

    Camera(float cx = 67.5f, float cy = 92.0f, float s = 4.8f, float hs = 5.2f)
        : centerX(cx), centerY(cy), scale(s), heightScale(hs),
          azimuth(0.0f), elevation(0.0f) {}

    // 将 3D 物理空间网格坐标 (x, y, z) 严格转换为 3D 轨道球透视屏幕坐标 (screenX, screenY)
    // 纯旋转无平移模型：
    // - elevation = 0: 正侧面平视 (用户平视设备，屏幕中也是平视云侧面)
    // - elevation > 0: 俯视观察云顶 (向前倾斜设备)
    // - elevation < 0: 仰视观察云底 (向后倾斜设备)
    // - azimuth 变化: 左右环绕观察云的左侧与右侧
    void project(float x, float y, float z, int& screenX, int& screenY) const {
        const float gridCenterX = 7.5f;
        const float gridCenterY = 7.5f;
        const float gridCenterZ = 5.5f;
        
        float dx = x - gridCenterX;
        float dy = y - gridCenterY;
        float dz = (z - gridCenterZ) * (heightScale / scale); // 统一几何尺度

        // 1. 缓存三角函数
        if (azimuth != _lastAzimuth) {
            _lastAzimuth = azimuth;
            _cachedCosA = cosf(azimuth);
            _cachedSinA = sinf(azimuth);
        }
        if (elevation != _lastElevation) {
            _lastElevation = elevation;
            _cachedCosE = cosf(elevation);
            _cachedSinE = sinf(elevation);
        }

        // 2. 绕世界垂直轴 Z 旋转 azimuth (左右转动视角)
        float x1 = dx * _cachedCosA - dy * _cachedSinA;
        float y1 = dx * _cachedSinA + dy * _cachedCosA;
        float z1 = dz;

        // 3. 绕相机水平横轴 X 旋转 elevation (前后俯仰视角)
        // Y_view 向上为正: 平视(E=0)时 Y_view = z1; 俯视(E>0)时看到近处 y1<0 偏下方，远处的 y1>0 偏上方
        float xView = x1;
        float yView = z1 * _cachedCosE - y1 * _cachedSinE;

        // 4. 投影至 2D 屏幕 (屏幕 Y 向下为正，因此减去 yView * scale)
        screenX = (int)roundf(centerX + xView * scale);
        screenY = (int)roundf(centerY - yView * scale);
    }
};
