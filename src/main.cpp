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

    // 2. 帧间隔计算（使用micros提高精度，避免高帧率时dt=0）
    uint32_t now = micros();
    float dt = (float)(now - lastFrameMicros) / 1000000.0f;
    lastFrameMicros = now;
    
    // 防止极端调试或断点等导致 dt 异常
    if (dt < 0.00001f) dt = 0.00001f;
    if (dt > 0.1f)     dt = 0.1f;
    
    // FPS滤波：使用更强的EMA低通滤波，避免数字抖动
    static float smoothFps = 0.0f;
    float rawFps = 1.0f / dt;
    if (smoothFps <= 0.0f) {
        smoothFps = rawFps;
    } else {
        smoothFps = smoothFps * 0.92f + rawFps * 0.08f;
    }
    fps = (int)(smoothFps + 0.5f);

    // 2.5 IMU 相机控制（透过窗户观察：设备倾斜控制视角旋转）
    {
        M5.Imu.update();

        static float imuX = 0.0f;
        static float imuY = 0.0f;
        static float imuZ = 1.0f;

        float ax = 0.0f, ay = 0.0f, az = 0.0f;
        if (M5.Imu.getAccel(&ax, &ay, &az)) {
            // 竖屏（Rotation 0，135x240）物理轴直接映射：
            // ax 对应屏幕左右侧倾 (Roll)
            // ay 对应屏幕前后俯仰 (Pitch)，补偿正常手持倾角基准偏置 ~0.55G
            float rawRoll = ax;
            float rawPitch = ay + 0.55f;

            // EMA 低通滤波（平滑视差运动）
            const float IMU_LPF_ALPHA = 0.85f;
            imuX = imuX * IMU_LPF_ALPHA + rawRoll * (1.0f - IMU_LPF_ALPHA);
            imuY = imuY * IMU_LPF_ALPHA + rawPitch * (1.0f - IMU_LPF_ALPHA);
            imuZ = imuZ * IMU_LPF_ALPHA + az * (1.0f - IMU_LPF_ALPHA);

            // 平滑死区处理：避免微小手抖产生的扰动
            const float IMU_DEADZONE = 0.04f;
            float effectiveX = fabsf(imuX) > IMU_DEADZONE ? (imuX > 0 ? imuX - IMU_DEADZONE : imuX + IMU_DEADZONE) : 0.0f;
            float effectiveY = fabsf(imuY) > IMU_DEADZONE ? (imuY > 0 ? imuY - IMU_DEADZONE : imuY + IMU_DEADZONE) : 0.0f;

            // 全息视窗映射（视差平移 + 微视角旋转）
            const float MAX_PAN_X = 22.0f;      // 水平视差平移 ±22 像素
            const float MAX_PAN_Y = 18.0f;      // 垂直视差平移 ±18 像素
            const float MAX_AZIMUTH = 0.35f;    // 水平旋转角 ±20 度 (产生立体侧视)
            const float MAX_ELEVATION = 0.30f;  // 仰角偏移 ±17 度 (产生俯视/侧视)

            camera->panX = clampF(-effectiveX * MAX_PAN_X, -MAX_PAN_X, MAX_PAN_X);
            camera->panY = clampF(effectiveY * MAX_PAN_Y, -MAX_PAN_Y, MAX_PAN_Y);
            camera->azimuth = clampF(-effectiveX * MAX_AZIMUTH, -MAX_AZIMUTH, MAX_AZIMUTH);
            camera->elevation = clampF(effectiveY * MAX_ELEVATION, -MAX_ELEVATION, MAX_ELEVATION);

            static uint32_t lastPrintMs = 0;
            if (millis() - lastPrintMs >= 1000) {
                lastPrintMs = millis();
                Serial.printf("[IMU] Accel: %.2f,%.2f,%.2f | Pan: %.1f,%.1f | Az/Elev: %.2f,%.2f\n",
                              ax, ay, az, camera->panX, camera->panY, camera->azimuth, camera->elevation);
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

    // 7. 帧率控制
    uint32_t elapsed = millis() - now;
    if (elapsed < FRAME_INTERVAL_MS) {
        delay(FRAME_INTERVAL_MS - elapsed);
    } else {
        delay(2);
    }

    // 性能日志（每秒输出一次）
    static uint32_t lastPerfLog = 0;
    if (millis() - lastPerfLog >= 1000) {
        lastPerfLog = millis();
        Serial.printf("[Perf] FPS:%d | Sim:%dµs | Particle:%dµs | Render:%dµs | Push:%dµs\n",
                      fps, simTime, particleTime, renderTime, pushTime);
    }
}
