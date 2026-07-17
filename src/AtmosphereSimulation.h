#pragma once
#include <Arduino.h>

struct AirCell {
    float temperature;   // 温度 (degC)
    float vapor;         // 水汽 (0.0 ~ 1.0)
    float cloudDensity;  // 云密度 (0.0 ~ 1.0)
    float velocityX;     // 风速 X
    float velocityY;     // 风速 Y
    float velocityZ;     // 风速 Z
    float pressure;      // 气压 (hPa)
    float lightIntensity;// 光照照度 (0.0 ~ 1.0)
    float posCharge;     // 正电荷密度
    float negCharge;     // 负电荷密度
};

class AtmosphereSimulation {
public:
    static constexpr int X_SIZE = 16;
    static constexpr int Y_SIZE = 16;
    static constexpr int Z_SIZE = 12;

    AtmosphereSimulation();
    ~AtmosphereSimulation();

    void init(float initTemp, float initHum, float initPres);
    void update(float dt, float extTemp, float extHum, float extPres, float extMicDb);
    void injectCyclone();

    // 闪电系统接口
    float getElectricCharge() const { return _electricCharge; }
    // 消耗闪电事件：返回 true 表示本帧应触发闪电，同时清除标志
    bool consumeLightningEvent();

    // 活跃云体素计数（在物理引擎更新时顺便统计，供渲染器使用，避免重复遍历）
    int getActiveCloudCount() const { return _activeCloudCount; }

    // 太阳方位角接口（用于动态光照）
    void  setSunAzimuth(float azimuth);             // 设置太阳方位角 (0~2PI)
    float getSunAzimuth() const { return _sunAzimuth; }
    
    // 台风生命周期接口
    float getTyphoonGrowth() const { return _typhoonGrowth; }

    inline const AirCell& getCell(int x, int y, int z) const {
        return _cells[z * X_SIZE * Y_SIZE + y * X_SIZE + x];
    }

    inline float getNoiseVal(int x, int y, int z) const {
        return _noiseTable[z * X_SIZE * Y_SIZE + y * X_SIZE + x];
    }

private:
    AirCell* _cells;
    AirCell* _nextCells;
    float*   _noiseTable;

    // 闪电系统状态
    float _electricCharge;   // 当前电荷（超过阈值则触发闪电）
    bool  _lightningPending; // 是否待触发闪电

    // 动态光照
    float _sunAzimuth;       // 太阳方位角 (0~2PI)，默认来自右前方
    int   _activeCloudCount; // 活跃云粒子统计
    float _typhoonGrowth;    // 台风成长进度 (0.0~1.0)

    // 温度逆温层参数
    float _inversionBaseZ;   // 逆温层起始高度
    float _inversionStrength;// 逆温强度

    // 随机风参数
    float _globalWindDir;    // 全局风向 (0~2PI)
    float _globalWindSpeed;  // 全局风速
    float _windShearX;       // X方向垂直风切变
    float _windShearY;       // Y方向垂直风切变

    // 电荷分离相关
    float _maxChargeSeparation; // 最大电荷分离度

    void computeLightField();
    void computeTemperatureProfile(float initTemp);
    void computeChargeSeparation(float dt);

    inline AirCell& getCellRef(int x, int y, int z) {
        return _cells[z * X_SIZE * Y_SIZE + y * X_SIZE + x];
    }

    inline AirCell& getNextCellRef(int x, int y, int z) {
        return _nextCells[z * X_SIZE * Y_SIZE + y * X_SIZE + x];
    }
};
