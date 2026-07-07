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

static uint32_t lastFrameTimeMs = 0;
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

    lastFrameTimeMs = millis();
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

    // 2. 帧间隔计算
    uint32_t now = millis();
    float dt = (float)(now - lastFrameTimeMs) / 1000.0f;
    lastFrameTimeMs = now;
    fps = (int)(1.0f / dt + 0.5f);

    // 防止极端调试或断点等导致 dt 异常
    if (dt < 0.001f) dt = 0.001f;
    if (dt > 0.1f)   dt = 0.1f;

    // 2.5 IMU 相机控制（带低通滤波防抖，绝对角度绑定模型，手势与视角绝对同步）
    {
        // 必须周期性更新 IMU 状态寄存器，否则底层 Accel 缓存永远为 0
        M5.Imu.update();

        float ax = 0.0f, ay = 0.0f, az = 0.0f;
        // 使用 if 结构对获取函数做安全防卡死校验，当且仅当硬件返回成功时更新视角
        if (M5.Imu.getAccel(&ax, &ay, &az)) {
            // 估计 roll/pitch：ax, ay 已经是 G (1G ≈ 1.0f) 为单位，平放时基准本就是 0.0，故直接使用
            float rawRoll  = ax;          // 左右倾斜
            float rawPitch = ay;          // 前后倾斜

            // EMA 低通滤波防抖
            static float filtRoll  = 0.0f;
            static float filtPitch = 0.0f;
            const float  ALPHA     = 0.18f; // 对齐 venom 最佳滤波器平滑参数，降低跟手延迟
            filtRoll  = filtRoll  * (1.0f - ALPHA) + rawRoll  * ALPHA;
            filtPitch = filtPitch * (1.0f - ALPHA) + rawPitch * ALPHA;

            // 绝对角度映射绑定（全息透视视口模型：手势倾斜与视角反向运动，构成三维深度镜面）
            camera->azimuth   = -filtRoll * 1.8f;   // 左右倾斜 (Roll) 控制水平旋转
            float tempElev    = -filtPitch * 0.7f;  // 前后倾斜 (Pitch) 控制仰角偏移
            camera->elevation = tempElev < -0.7f ? -0.7f : (tempElev > 0.7f ? 0.7f : tempElev);

            // 调试打印：检查 IMU 运行数据与映射角度
            static uint32_t lastPrintMs = 0;
            if (millis() - lastPrintMs >= 1000) {
                lastPrintMs = millis();
                Serial.printf("[IMU Debug] Enabled: %d | Accel X/Y/Z: %.2f, %.2f, %.2f | Filt R/P: %.2f, %.2f | Cam Azimuth: %.2f, Elev: %.2f\n",
                              M5.Imu.isEnabled(), ax, ay, az, filtRoll, filtPitch, camera->azimuth, camera->elevation);
            }
        }
    }

    // 3. 物理模拟演化 (累积步长达到 50ms 时才执行流体引擎更新，将流体计算降频至 20Hz)
    // 这样做在渲染高帧率时，可省去大部分物理算力，使 IMU 相机控制和粒子降雨保持极高帧率
    static float accumulatedSimTime = 0.0f;
    accumulatedSimTime += dt;
    if (accumulatedSimTime >= 0.05f) {
        float simDt = accumulatedSimTime * 1.5f; 
        if (simDt > 0.1f) simDt = 0.1f;
        sim->update(simDt, extTemp, extHum, extPres, extMicDb);
        accumulatedSimTime = 0.0f;
    }

    // 4. 更新雨滴粒子系统 (使用真实帧步长，使雨滴降落速度流畅)
    renderer->updateParticles(*sim, dt);

    // 5. 渲染画面
    canvas.clear();
    renderer->draw(*sim, *camera, extTemp, extHum, extPres, extMicDb, fps);

    // 6. 推送显示
    canvas.pushSprite(0, 0);

    // 7. 帧率控制 (约 30 FPS，包含合理空闲礼让，防止 CPU 100% 挂载)
    uint32_t elapsed = millis() - now;
    if (elapsed < FRAME_INTERVAL_MS) {
        delay(FRAME_INTERVAL_MS - elapsed);
    } else {
        delay(2); // 即使帧渲染超时，也强制礼让 2ms 给系统后台任务调度
    }
}
