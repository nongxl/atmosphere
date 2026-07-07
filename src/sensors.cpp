#include "sensors.h"
#include <Wire.h>
#include <M5UnitENV.h>   // m5stack/M5Unit-ENV v1.5+
#include <M5Unified.h>   // M5.Mic
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// ═══════════════════════════════════════════════════════════════
//  实现：SHT3X 温湿度 + QMP6988 气压
//  M5Unit-ENV v1.5 API:
//    SHT3X::begin(wire, addr, sda, scl, freq)
//    SHT3X::update() → .cTemp / .humidity
//    QMP6988::begin(wire, addr, sda, scl, freq)
//    QMP6988::calcPressure() → Pa（需除以100转换为hPa）
// ═══════════════════════════════════════════════════════════════

namespace Sensors {

static SHT3X   _sht;
static QMP6988 _qmp;
static bool    _sReady = false;
static bool    _qReady = false;

// 当前读数（初始设合理默认值，传感器失败时作为回退）
static float _temp  = 25.0f;
static float _hum   = 55.0f;
static float _pres  = 1013.25f;

// 气压历史环形缓冲
static float    _presHist[PRESSURE_HISTORY_LEN];
static int      _presIdx   = 0;
static int      _presCount = 0;

static uint32_t _lastMs = 0;

static TaskHandle_t _sensorsTaskHandle = NULL;

// FreeRTOS 异步传感器读取任务，运行于 Core 0
static void sensorsTask(void* pvParameters) {
    while (true) {
        // 使用 vTaskDelay 阻塞 20 秒，绝不霸占主循环 CPU 时间
        vTaskDelay(pdMS_TO_TICKS(SENSOR_INTERVAL_MS));
        
        if (_sReady && _sht.update()) {
            float t = _sht.cTemp;
            float h = _sht.humidity;
            if (t > -15.0f && t < 55.0f && h > 0.5f && h <= 100.0f && !(fabsf(t) < 0.001f && fabsf(h) < 0.001f)) {
                _temp = t;
                _hum  = h;
            }
        }

        if (_qReady) {
            float p = _qmp.calcPressure() / 100.0f;  // Pa -> hPa
            if (p > 850.0f && p < 1100.0f) {
                _pres = p;
                _presHist[_presIdx] = p;
                _presIdx = (_presIdx + 1) % PRESSURE_HISTORY_LEN;
                if (_presCount < PRESSURE_HISTORY_LEN) _presCount++;
            }
        }
    }
}

// ──────────────────────────────────────────────────────────────
bool init() {
    // 延迟 120ms 等待 AXP PMIC 供电输出在外部电容完成充电、传感器芯片上电稳定
    delay(120);

    // 完美复现 venom_esp32 对全局 Wire 的总线复用及外部引脚重映射
    Wire.end();
    Wire.begin(HAT_SDA, HAT_SCL, 100000UL); // 外接 ENV HAT 传感器使用更稳定的 100kHz 频率
    delay(50);
    
    // 设置 I2C 默认总线超时时间为极其紧凑的 3ms
    Wire.setTimeOut(3);

    // SHT3X: 显式传入引脚参数与默认 Wire
    _sReady = _sht.begin(&Wire, SHT3X_I2C_ADDR, HAT_SDA, HAT_SCL, 100000UL);
    
    // QMP6988: 同样显式传入引脚参数与默认 Wire
    _qReady = _qmp.begin(&Wire, QMP6988_SLAVE_ADDRESS_L, HAT_SDA, HAT_SCL, 100000UL);

    // 重新强制写入 3ms 超时，确保覆盖库内部 begin 重置的影响
    Wire.setTimeOut(3);

    if (!_sReady) Serial.println("[Sensors] SHT3X not found, using defaults");
    if (!_qReady) Serial.println("[Sensors] QMP6988 not found, using defaults");

    // 初始读取
    if (_sReady && _sht.update()) {
        float t = _sht.cTemp;
        float h = _sht.humidity;
        if (t > -15.0f && t < 55.0f && h > 0.5f && h <= 100.0f && !(fabsf(t) < 0.001f && fabsf(h) < 0.001f)) {
            _temp = t;
            _hum  = h;
        }
    }
    if (_qReady) {
        float p = _qmp.calcPressure() / 100.0f;  // Pa → hPa
        if (p > 850.0f && p < 1100.0f) _pres = p;
    }

    // 预填气压历史
    for (int i = 0; i < PRESSURE_HISTORY_LEN; i++) _presHist[i] = _pres;
    _presCount = PRESSURE_HISTORY_LEN;

    // 成功检测到任意传感器后，在 Core 0 启动 FreeRTOS 异步读取任务，彻底释放主渲染循环 (Core 1) 的 CPU 时间片
    if (_sReady || _qReady) {
        xTaskCreatePinnedToCore(
            sensorsTask,
            "SensorsTask",
            4096,
            NULL,
            1,
            &_sensorsTaskHandle,
            0
        );
    }

    return _sReady || _qReady;
}

// ──────────────────────────────────────────────────────────────
void update() {
    // 异步 FreeRTOS 任务模式下，温湿度/压强由后台任务自动刷新，前台 loop 调用不执行阻塞操作
}

// ──────────────────────────────────────────────────────────────
float getTemperature()   { return _temp; }
float getHumidity()      { return _hum;  }
float getPressure()      { return _pres; }

// 线性回归：hPa/min（负值 = 气压持续下降）
float getPressureTrend() {
    int n = _presCount;
    if (n < 3) return 0.0f;

    float sx = 0, sy = 0, sxx = 0, sxy = 0;
    for (int i = 0; i < n; i++) {
        int   idx = (_presIdx - n + i + PRESSURE_HISTORY_LEN) % PRESSURE_HISTORY_LEN;
        float x   = (float)i * SENSOR_INTERVAL_MS / 60000.0f;  // 分钟
        float y   = _presHist[idx];
        sx  += x;  sy  += y;
        sxx += x * x;
        sxy += x * y;
    }
    float d = (float)n * sxx - sx * sx;
    if (fabsf(d) < 1e-6f) return 0.0f;
    return ((float)n * sxy - sx * sy) / d;  // hPa/min
}

bool isReady() { return _sReady || _qReady; }

// ══════════════════════════════════════════════════════════════
//  麦克风（M5StickS3 内置 SPM1423 PDM Mic）
//  原理：每 6 帧采集 64 个 16-bit 样本（8kHz ≈ 8ms）
//        RMS → dBFS（0~96 dB 满量程）
//        0.3 权重指数平滑，避免数值抖动
// ══════════════════════════════════════════════════════════════
static float   _micDb    = 30.0f;   // 初始假设环境底噪 ≈30 dB
static uint8_t _micFrame = 0;

void initMic() {
    auto cfg = M5.Mic.config();
    cfg.sample_rate   = 8000;
    cfg.magnification = 16;   // 硬件增益，提升小声音灵敏度
    M5.Mic.config(cfg);
    M5.Mic.begin();
    Serial.println("[Sensors] Mic initialized");
}

void updateMic() {
    if (++_micFrame < 6) return;   // 每 6 帧采样一次（~200ms @30fps）
    _micFrame = 0;

    static int16_t buf[64];
    // blocking=true，64 采样 @8kHz ≈ 8ms
    if (!M5.Mic.record(buf, 64, 8000, true)) return;

    // RMS 计算
    int64_t sum = 0;
    for (int i = 0; i < 64; i++) {
        int32_t s = buf[i];
        sum += (int64_t)s * s;
    }
    float rms = sqrtf((float)(sum / 64));

    // dBFS：满量程 32767 → 90.3 dB；静音 → ~30 dB
    float db = (rms > 1.0f) ? 20.0f * log10f(rms) : 0.0f;
    db = clampF(db, 0.0f, 96.0f);

    // 指数平滑（上升快/下降慢，更自然地模拟风起风落）
    float alpha = (db > _micDb) ? 0.45f : 0.20f;
    _micDb = _micDb * (1.0f - alpha) + db * alpha;
}

float getMicDb() { return _micDb; }

} // namespace Sensors
