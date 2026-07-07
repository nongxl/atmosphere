#pragma once
#include <M5GFX.h>
#include "AtmosphereSimulation.h"
#include "Camera.h"
#include "config.h"

#define RGB565(r, g, b) (((uint16_t)((r) & 0xF8) << 8) | ((uint16_t)((g) & 0xFC) << 3) | ((uint16_t)(b) >> 3))

struct RainDrop {
    float x, y, z;
    float vx, vy, vz;
    bool active;
};

class Renderer {
public:
    Renderer();
    
    // 初始化渲染器并设置画布
    void init(M5Canvas* canvas);

    // 更新雨滴粒子系统
    void updateParticles(const AtmosphereSimulation& sim, float dt);

    // 绘制整个世界（背景、体积云、降雨、闪电、状态栈）
    // 注意: sim 为非 const 引用，以便内部消耗闪电事件标志
    void draw(AtmosphereSimulation& sim, const Camera& cam, float extTemp, float extHum, float extPres, float extMicDb, int fps);

private:
    M5Canvas* _canvas;
    RainDrop _drops[MAX_RAIN_DROPS];

    // 绘制天空渐变背景
    void drawSkyBackground(float extTemp);

    // 绘制体积云（基于画家算法）
    void drawClouds(const AtmosphereSimulation& sim, const Camera& cam, float densityThreshold = 0.4f);
    // Helper to smooth density (implemented in .cpp)
    float sampleDensity(const AtmosphereSimulation& sim, int x, int y, int z) const;

    // 绘制雨滴
    void drawRain(const Camera& cam);

    // 绘制状态栈信息
    void drawStatusBar(float extTemp, float extHum, float extPres, float extMicDb, int activeCloudCount, float electricCharge, int fps);

    // 闪电物理渲染引擎定义
    int      _lightningFrames;          // 剩余闪电生命帧数 (>0 = 激活)
    int      _lightningType;            // 闪电分类：0=云内(Sheet), 1=云际(Cloud-to-Cloud), 2=闪地(Cloud-to-Ground)
    uint16_t _lightningColor;           // 闪电物理电弧色
    float    _lightningFlicker[5];      // 5帧正弦双波频闪衰减查找表
    
    // 3D 物理格点震源
    float    _epicenterX3D;
    float    _epicenterY3D;
    float    _epicenterZ3D;
    
    // 2D 投影屏幕起点
    int      _lightningX;
    int      _lightningY;

    // 分形分枝闪电绘制（递归）
    void drawLightningBolt(int x1, int y1, int x2, int y2, int depth, float intensity);
};
