#include <M5Unified.h>
#include "config.h"
#include "imu_controller.h"
#include "cloud_physics.h"
#include "cloud_renderer.h"

// ═══════════════════════════════════════════════════════════════
//  Atmosphere Phase 1 — Soft Cloud Renderer Prototype
// ═══════════════════════════════════════════════════════════════

static M5Canvas       canvas(&M5.Display);
static IMUController  imu;
static CloudPhysics   physics;
static CloudRenderer  renderer;
static CloudConfig    cloudConfig;

static uint8_t  brightnessLevels[] = {30, 60, 120, 200};
static uint8_t  brightnessIdx      = 1;
static bool     debugMode          = (CLOUD_DEBUG == 1);
static uint8_t  presetIdx          = 1; // 默认 1: CLOUD_LIGHT_NATURAL
static const char* presetNames[]   = {"SOFT", "NATURAL", "DRAMATIC"};

static uint32_t lastFrameMs        = 0;
static float    currentFps         = 30.0f;
static uint32_t frameCount         = 0;
static uint32_t fpsTimerMs         = 0;

void setup() {
    // 1. 初始化 M5Unified 硬件驱动
    auto cfg = M5.config();
    M5.begin(cfg);

    // 2. 初始化串口
    Serial.begin(115200);
    delay(50);
    Serial.println("\n================================================");
    Serial.println("  Atmosphere Phase 1 — Soft Cloud Prototype     ");
    Serial.println("================================================");

    // 3. 配置横屏 LCD (240×135)
    M5.Display.setRotation(SCREEN_ROTATION);
    M5.Display.setBrightness(brightnessLevels[brightnessIdx]);
    M5.Display.fillScreen(0x0000);

    // 4. 初始化 IMU
    bool imuOk = imu.begin();
    Serial.printf("[System] IMU Initialized: %s\n", imuOk ? "SUCCESS" : "FAILED/NOT_FOUND");

    // 5. 初始化云物理引擎与参数
    physics.init(cloudConfig);

    // 6. 初始化渲染引擎与画布置
    renderer.init(canvas, cloudConfig);

    lastFrameMs = millis();
    fpsTimerMs  = millis();

    Serial.println("[System] Cloud Renderer Engine Ready.");
}

void loop() {
    uint32_t nowMs = millis();
    float dt = (nowMs - lastFrameMs) / 1000.0f;
    if (dt < 0.001f) dt = 0.001f;
    lastFrameMs = nowMs;

    // 1. 硬件输入轮询
    M5.update();

    // 按键 A: 短按切换 Debug 模式并触发冲量；长按切换光照预设 (SOFT/NATURAL/DRAMATIC)
    if (M5.BtnA.wasHold()) {
        presetIdx = (presetIdx + 1) % 3;
        renderer.applyPreset((CloudLightPreset)presetIdx);
        Serial.printf("[BtnA Hold] Light Preset -> %s\n", presetNames[presetIdx]);
    } else if (M5.BtnA.wasClicked()) {
        debugMode = !debugMode;
        imu.triggerImpulse(1.2f);
        Serial.printf("[BtnA] Debug mode %s, impulse triggered\n", debugMode ? "ON" : "OFF");
    }

    // 按键 B: 调节屏幕背光亮度
    if (M5.BtnB.wasPressed()) {
        brightnessIdx = (brightnessIdx + 1) % 4;
        M5.Display.setBrightness(brightnessLevels[brightnessIdx]);
        Serial.printf("[BtnB] Brightness: %u\n", brightnessLevels[brightnessIdx]);
    }

    // 2. IMU 采样与重力/冲量滤波
    imu.update(dt);

    // 3. 云物理动力学与呼吸形变更新
    physics.update(dt, imu.getGravityX(), imu.getGravityY(), imu.getShakeImpulse());

    // 4. 软云 SDF 渲染并推屏
    renderer.render(canvas, physics, imu, currentFps, debugMode);

    // 5. 帧率统计计算
    frameCount++;
    if (nowMs - fpsTimerMs >= 500) {
        currentFps = (frameCount * 1000.0f) / (float)(nowMs - fpsTimerMs);
        frameCount = 0;
        fpsTimerMs = nowMs;
    }

    // 6. 帧率限速节流 (保持在目标帧率附近，避免空转过载)
    uint32_t frameTimeMs = millis() - nowMs;
    if (frameTimeMs < FRAME_INTERVAL_MS) {
        delay(FRAME_INTERVAL_MS - frameTimeMs);
    }
}
