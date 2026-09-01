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

    // 2.5 IMU 3D 轨道球相机视角控制（纯旋转观察：平视侧面、前俯看顶、后仰看底、左右看侧）
    {
        M5.Imu.update();

        static float imuRoll = 0.0f;
        static float imuPitch = 0.0f;

        float ax = 0.0f, ay = 0.0f, az = 0.0f;
        if (M5.Imu.getAccel(&ax, &ay, &az)) {
            // 竖屏（Rotation 0）：
            // 左右转动由横向分量 ay 驱动
            // 前后俯仰转动由垂直屏幕分量 az 驱动（平视竖直时 az≈0，前倾看顶 az>0，后仰看底 az<0）
            float rawRoll = ay;
            float rawPitch = az;

            // EMA 低通滤波（平滑旋转视角）
            const float IMU_LPF_ALPHA = 0.85f;
            imuRoll = imuRoll * IMU_LPF_ALPHA + rawRoll * (1.0f - IMU_LPF_ALPHA);
            imuPitch = imuPitch * IMU_LPF_ALPHA + rawPitch * (1.0f - IMU_LPF_ALPHA);

            // 平滑死区
            const float DEADZONE = 0.03f;
            float effRoll = fabsf(imuRoll) > DEADZONE ? (imuRoll > 0 ? imuRoll - DEADZONE : imuRoll + DEADZONE) : 0.0f;
            float effPitch = fabsf(imuPitch) > DEADZONE ? (imuPitch > 0 ? imuPitch - DEADZONE : imuPitch + DEADZONE) : 0.0f;

            // 映射至 3D 轨道球视角（无平移）：
            // 左右转动设备 -> 观察云的左侧与右侧 (Azimuth 范围 ±1.4 弧度 ≈ ±80°)
            // 前后转动设备 -> 观察云的顶部与底部 (Elevation 范围 ±1.1 弧度 ≈ ±63°)
            camera->azimuth = clampF(-effRoll * 1.5f, -1.4f, 1.4f);
            camera->elevation = clampF(effPitch * 1.3f, -1.1f, 1.1f);

            static uint32_t lastPrintMs = 0;
            if (millis() - lastPrintMs >= 1000) {
                lastPrintMs = millis();
                Serial.printf("[IMU] Accel: %.2f,%.2f,%.2f | Azimuth(Yaw): %.2f | Elevation(Pitch): %.2f\n",
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
