#include "ble_server.h"

#if defined(ARDUINO)
#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// ============================================================================
// 内部跨核共享状态（Core0 BLE 回调写入，Core1 400Hz 控制环读取）
// ============================================================================
static portMUX_TYPE     s_cmd_mux        = portMUX_INITIALIZER_UNLOCKED;
static ControlCommand_t s_latest_cmd     = { 0.0f, 0.0f, 0.0f, 0.0f, false };
static uint32_t         s_last_rx_ms     = 0U;
static bool             s_has_valid_rx   = false;
static bool             s_ble_connected  = false;

static uint32_t         s_rx_ok_count    = 0U;
static uint32_t         s_rx_err_count   = 0U;
static uint8_t          s_last_rx_seq    = 0U;
static uint8_t          s_telem_seq      = 0U;

static BLEServer         *s_pServer      = nullptr;
static BLECharacteristic *s_pTelemChar   = nullptr;

// ============================================================================
// 1. BLE 连接/断开事件回调
// ============================================================================
class UavServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer *pServer, esp_ble_gatts_cb_param_t *param) override {
        portENTER_CRITICAL(&s_cmd_mux);
        s_ble_connected = true;
        portEXIT_CRITICAL(&s_cmd_mux);

        // 主动向主机（手机/PC）请求低延迟 BLE 连接参数：
        // min_int = 0x0006 (6 * 1.25ms = 7.5ms)
        // max_int = 0x000C (12 * 1.25ms = 15.0ms)
        // latency = 0 (不跳过任何连接事件)
        // timeout = 200 (200 * 10ms = 2000ms 监督超时)
        pServer->updateConnParams(param->connect.remote_bda, 0x0006, 0x000C, 0, 200);
    }

    void onDisconnect(BLEServer *pServer) override {
        portENTER_CRITICAL(&s_cmd_mux);
        s_ble_connected       = false;
        // 物理断链瞬间强制让指令失效，触发飞控安全模块保护
        s_latest_cmd.throttle = 0.0f;
        s_has_valid_rx        = false;
        portEXIT_CRITICAL(&s_cmd_mux);

        // 立即重新开启广播，允许飞手随时重连
        BLEDevice::startAdvertising();
    }
};

// ============================================================================
// 2. BLE 控制特征值写入回调（处理 50Hz Write Without Response 帧）
// ============================================================================
class UavCtrlCharCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) override {
        uint8_t *raw_data = pCharacteristic->getData();
        size_t   raw_len  = pCharacteristic->getLength();

        ControlCommand_t parsed_cmd;
        uint8_t          seq = 0U;

        ProtocolStatus_t st = protocol_parse_control(raw_data, raw_len, &parsed_cmd, &seq);
        uint32_t now_ms = millis();

        portENTER_CRITICAL(&s_cmd_mux);
        if (st == PROTOCOL_OK) {
            s_latest_cmd   = parsed_cmd;
            s_last_rx_ms   = now_ms;
            s_has_valid_rx = true;
            s_last_rx_seq  = seq;
            s_rx_ok_count++;
        } else {
            s_rx_err_count++;
        }
        portEXIT_CRITICAL(&s_cmd_mux);
    }
};

// ============================================================================
// 3. 公共接口实现
// ============================================================================
void ble_comm_init(const char *device_name) {
    const char *name = (device_name != nullptr && device_name[0] != '\0')
                       ? device_name
                       : UAV_BLE_DEVICE_NAME;

    BLEDevice::init(name);
    // 提升 ESP32-S3 BLE 发射功率至 +9dBm，增强 10~20m 室内外抗干扰能力
    BLEDevice::setPower(ESP_PWR_LVL_P9);

    s_pServer = BLEDevice::createServer();
    s_pServer->setCallbacks(new UavServerCallbacks());

    BLEService *pService = s_pServer->createService(UAV_BLE_SERVICE_UUID);

    // 下行控制特征值：支持 Write Without Response（低延迟主力）与普通 Write（兼容调试）
    BLECharacteristic *pCtrlChar = pService->createCharacteristic(
        UAV_BLE_CHAR_CTRL_UUID,
        BLECharacteristic::PROPERTY_WRITE_NR | BLECharacteristic::PROPERTY_WRITE
    );
    pCtrlChar->setCallbacks(new UavCtrlCharCallbacks());

    // 上行遥测特征值：支持 Notify 订阅与主动 Read
    s_pTelemChar = pService->createCharacteristic(
        UAV_BLE_CHAR_TELEM_UUID,
        BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ
    );
    s_pTelemChar->addDescriptor(new BLE2902());

    pService->start();

    // 配置低延迟广播参数，让手机/电脑秒级扫到设备
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(UAV_BLE_SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06); // 提示主机优先采用 7.5ms 连接间隔
    pAdvertising->setMaxPreferred(0x0C); // 15ms
    BLEDevice::startAdvertising();
}

void ble_comm_get_command(ControlCommand_t *out_cmd, uint32_t *out_cmd_age_ms) {
    uint32_t now_ms = millis();

    portENTER_CRITICAL(&s_cmd_mux);
    bool     valid   = s_has_valid_rx && s_ble_connected;
    uint32_t last_ms = s_last_rx_ms;
    if (out_cmd != nullptr) {
        *out_cmd = s_latest_cmd;
    }
    portEXIT_CRITICAL(&s_cmd_mux);

    if (out_cmd_age_ms != nullptr) {
        if (!valid) {
            // 未连接或从未收到有效包时，返回一个远超 500ms 的超时值（10000ms）
            *out_cmd_age_ms = 10000U;
        } else {
            *out_cmd_age_ms = (uint32_t)(now_ms - last_ms);
        }
    }
}

bool ble_comm_send_telemetry(FlightState_t state,
                             float vbat_v,
                             EulerAngle_t att_rad,
                             MotorOutput_t motors,
                             uint16_t loop_us) {
    if (!s_ble_connected || s_pTelemChar == nullptr) {
        return false;
    }

    BleTelemetryPacket_t pkt;
    protocol_pack_telemetry(s_telem_seq++, state, vbat_v, att_rad, motors, loop_us, &pkt);

    s_pTelemChar->setValue((uint8_t *)&pkt, sizeof(BleTelemetryPacket_t));
    s_pTelemChar->notify();
    return true;
}

void ble_comm_get_stats(BleCommStats_t *out_stats) {
    if (out_stats == nullptr) return;
    portENTER_CRITICAL(&s_cmd_mux);
    out_stats->rx_ok_count  = s_rx_ok_count;
    out_stats->rx_err_count = s_rx_err_count;
    out_stats->last_seq     = s_last_rx_seq;
    out_stats->connected    = s_ble_connected;
    portEXIT_CRITICAL(&s_cmd_mux);
}

#endif // defined(ARDUINO)
