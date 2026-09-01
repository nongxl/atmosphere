#include <Arduino.h>
#include <M5Unified.h>
#include "config.h"
#include "sensors.h"
#include "AtmosphereSimulation.h"
#include "Camera.h"
#include "Renderer.h"

// ─────────────────────────────────────────────────────────────
//  全局变量
// ─────────────────────────────────────────────────────────────
static M5Canvas canvas(&M5.Display);

static AtmosphereSimulation* sim = nullptr;
static Camera* camera = nullptr;
static Renderer* renderer = nullptr;

static uint32_t lastFrameMicros = 0;
static int fps = 0;

void setup() {
    auto cfg = M5.config();
    cfg.internal_imu = true; // 显式开启内部 IMU 的硬件探测与初始化，必不可少
    M5.begin(cfg);
    M5.Imu.init(); // 强行对 IMU 进行硬件级初始化以开始数据采集

    // 开启外接插槽/HAT 接口的 5V/3.3V 外部电源输出，为 ENV HAT 传感器通电
    M5.Power.setExtOutput(true);

    // 设置屏幕方向（竖屏）与亮度
    M5.Display.setRotation(0);
    M5.Display.setBrightness(SYSTEM_BRIGHTNESS);

    // 在 SRAM 中创建主画布（移除 PSRAM：PSRAM 写速 ~10MB/s 远慢于 SRAM ~400MB/s，
    // setPsram(true) 导致 fillCircle 极慢是 FPS=4 的主要原因）
    canvas.setPsram(false);
    canvas.createSprite(SCREEN_W, SCREEN_H);

    Serial.begin(115200);
    Serial.println("[System] Initializing Micro Atmosphere...");

    // 初始化传感器与内置麦克风
    Sensors::init();
    Sensors::initMic();

    // 动态分配模拟器和辅助模块以防栈溢出
    sim = new AtmosphereSimulation();
    camera = new Camera(67.5f, 90.0f, 4.3f, 5.0f); // 针对 135x195 区域微调
    renderer = new Renderer();

    renderer->init(&canvas);

    // 获取初值初始化大气世界
    Sensors::update();
    float t = Sensors::getTemperature();
    float h = Sensors::getHumidity();
    float p = Sensors::getPressure();
    sim->init(t, h, p);

    lastFrameMicros = micros();
    Serial.println("[System] Initialization complete.");
}

void loop() {
    M5.update();

    // ── BtnA: 短按注入气旋，长按时循环旋转太阳方位角 ──
    if (M5.BtnA.wasPressed()) {
        sim->injectCyclone();
    }
    if (M5.BtnA.isPressed()) {
        // 持按 BtnA：每 80ms 旋转太阳约 ~5°，约 70s 完成一圈
        static uint32_t lastSunRotMs = 0;
        uint32_t nowMs = millis();
        if (nowMs - lastSunRotMs >= 80) {
            lastSunRotMs = nowMs;
            sim->setSunAzimuth(sim->getSunAzimuth() + 0.08f);  // +4.6°/step
        }
    }

    // 1. 获取传感器与声音数据
    Sensors::update();
    Sensors::updateMic();

    float extTemp  = Sensors::getTemperature();
    float extHum   = Sensors::getHumidity();
    float extPres  = Sensors::getPressure();
    float extMicDb = Sensors::getMicDb();

    // 2. 真实 FPS 精确测量器（基于每 500ms 实际完成帧数统计，杜绝微秒抖动与截断死锁）
    static uint32_t frameCounter = 0;
    static uint32_t lastFpsCalcMs = 0;
    static int realFps = 60;

    frameCounter++;
    uint32_t currentMs = millis();
    if (currentMs - lastFpsCalcMs >= 500) {
        realFps = (int)((frameCounter * 1000.0f) / (float)(currentMs - lastFpsCalcMs) + 0.5f);
        frameCounter = 0;
        lastFpsCalcMs = currentMs;
    }
    fps = realFps;

    // 物理步长计算（限制单步仿真安全上限）
    uint32_t now = micros();
    float dt = (float)(now - lastFrameMicros) / 1000000.0f;
    lastFrameMicros = now;
    if (dt < 0.001f) dt = 0.001f;
    if (dt > 0.05f)  dt = 0.05f;

    // 2.5 IMU 相机控制（透过窗户观察：设备倾斜控制视角旋转）
    {
        M5.Imu.update();

        static float imuX = 0.0f;
        static float imuY = 0.0f;
        static float imuZ = 1.0f;

        float ax = 0.0f, ay = 0.0f, az = 0.0f;
        if (M5.Imu.getAccel(&ax, &ay, &az)) {
            // 竖屏（Rotation 0，135x240）物理轴映射（对调轴向）：
            // ay 对应屏幕左右侧倾 (Roll)
            // ax 对应屏幕前后俯仰 (Pitch)，补偿正常手持倾角基准偏置 ~0.55G
            float rawRoll = ay;
            float rawPitch = ax + 0.55f;

            // EMA 低通滤波（平滑视差运动）
            const float IMU_LPF_ALPHA = 0.85f;
            imuX = imuX * IMU_LPF_ALPHA + rawRoll * (1.0f - IMU_LPF_ALPHA);
            imuY = imuY * IMU_LPF_ALPHA + rawPitch * (1.0f - IMU_LPF_ALPHA);
            imuZ = imuZ * IMU_LPF_ALPHA + az * (1.0f - IMU_LPF_ALPHA);

            // 平滑死区处理：避免微小手抖产生的扰动
            const float IMU_DEADZONE = 0.04f;
            float effectiveX = fabsf(imuX) > IMU_DEADZONE ? (imuX > 0 ? imuX - IMU_DEADZONE : imuX + IMU_DEADZONE) : 0.0f;
            float effectiveY = fabsf(imuY) > IMU_DEADZONE ? (imuY > 0 ? imuY - IMU_DEADZONE : imuY + IMU_DEADZONE) : 0.0f;

            // 纯 3D 空间环绕视角映射（中心绝对固定，转动设备旋转 3D 观察视角）
            // 左右侧倾控制水平方位角 (azimuth)，可大范围环视云团四周侧面
            // 前后俯仰控制俯仰仰角 (elevation)，可从俯视云顶到平视云腰自由观察
            const float MAX_AZIMUTH   = 1.15f; // 约 66 度水平环视范围
            const float MAX_ELEVATION = 0.55f; // 约 32 度俯仰仰角范围

            camera->azimuth   = clampF(-effectiveX * MAX_AZIMUTH, -MAX_AZIMUTH, MAX_AZIMUTH);
            camera->elevation = clampF(effectiveY * MAX_ELEVATION, -MAX_ELEVATION, MAX_ELEVATION);

            static uint32_t lastPrintMs = 0;
            if (millis() - lastPrintMs >= 1000) {
                lastPrintMs = millis();
                Serial.printf("[IMU 3D Orbit] Accel: %.2f,%.2f,%.2f | Azimuth: %.2f rad | Elev: %.2f rad\n",
                              ax, ay, az, camera->azimuth, camera->elevation);
            }
        }
    }

    // 3. 物理模拟演化 (累积步长达到 80ms 时才执行流体引擎更新，将流体计算降频至 12Hz)
    static float accumulatedSimTime = 0.0f;
    accumulatedSimTime += dt;
    uint32_t simStart = micros();
    if (accumulatedSimTime >= 0.08f) {
        float simDt = accumulatedSimTime * 1.5f; 
        if (simDt > 0.15f) simDt = 0.15f;
        sim->update(simDt, extTemp, extHum, extPres, extMicDb);
        accumulatedSimTime = 0.0f;
    }
    uint32_t simTime = micros() - simStart;

    // 4. 更新雨滴粒子系统
    uint32_t particleStart = micros();
    renderer->updateParticles(*sim, dt);
    uint32_t particleTime = micros() - particleStart;

    // 5. 渲染画面
    uint32_t renderStart = micros();
    canvas.clear();
    renderer->draw(*sim, *camera, extTemp, extHum, extPres, extMicDb, fps);
    uint32_t renderTime = micros() - renderStart;

    // 6. 推送显示
    uint32_t pushStart = micros();
    canvas.pushSprite(0, 0);
    uint32_t pushTime = micros() - pushStart;

    // 7. 精确微秒级 60 FPS 帧率控制
    uint32_t frameEndMicros = micros();
    uint32_t frameDurationMicros = frameEndMicros - now;
    const uint32_t TARGET_FRAME_MICROS = 1000000UL / 60UL; // ~16.6ms @ 60 FPS
    if (frameDurationMicros < TARGET_FRAME_MICROS) {
        delayMicroseconds(TARGET_FRAME_MICROS - frameDurationMicros);
    } else {
        yield();
    }

    // 性能日志（每秒输出一次）
    static uint32_t lastPerfLog = 0;
    if (millis() - lastPerfLog >= 1000) {
        lastPerfLog = millis();
        Serial.printf("[Perf] FPS:%d | Sim:%dµs | Particle:%dµs | Render:%dµs | Push:%dµs\n",
                      fps, simTime, particleTime, renderTime, pushTime);
    }
}
