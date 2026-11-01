#include <stdio.h>
#include <math.h>
#include "../src/comm/protocol.h"

#define ASSERT_TRUE(cond, msg) \
    do { \
        if (!(cond)) { \
            printf("[FAIL] %s (line %d)\n", msg, __LINE__); \
            return 1; \
        } else { \
            printf("[PASS] %s\n", msg); \
        } \
    } while (0)

int main(void) {
    printf("============================= BLE Protocol Unit Test =============================\n");

    // 1. 测试正常控制帧打包与解析闭环
    ControlCommand_t cmd_in = {
        .throttle        = 0.45f,
        .roll_target     =  10.0f * (float)M_PI / 180.0f,
        .pitch_target    = -12.5f * (float)M_PI / 180.0f,
        .yaw_rate_target =  90.0f * (float)M_PI / 180.0f,
        .armed           = true
    };

    BleControlPacket_t pkt;
    protocol_pack_control(&cmd_in, 42U, 0U, &pkt);

    ControlCommand_t cmd_out = { 0 };
    uint8_t seq_out = 0U;
    ProtocolStatus_t st = protocol_parse_control((const uint8_t *)&pkt, sizeof(pkt), &cmd_out, &seq_out);

    ASSERT_TRUE(st == PROTOCOL_OK, "1. Normal control packet pack & parse OK");
    ASSERT_TRUE(seq_out == 42U && cmd_out.armed == true, "2. Sequence & armed flag preserved");
    ASSERT_TRUE(fabsf(cmd_out.throttle - 0.45f) < 0.002f, "3. Throttle accuracy within 0.1%");
    ASSERT_TRUE(fabsf(cmd_out.roll_target - cmd_in.roll_target) < 0.002f, "4. Roll target accuracy");
    ASSERT_TRUE(fabsf(cmd_out.pitch_target - cmd_in.pitch_target) < 0.002f, "5. Pitch target accuracy");
    ASSERT_TRUE(fabsf(cmd_out.yaw_rate_target - cmd_in.yaw_rate_target) < 0.002f, "6. Yaw rate target accuracy");

    // 2. 测试紧急急停位 (PROTOCOL_FLAG_KILL) 强制上锁且油门归零
    protocol_pack_control(&cmd_in, 43U, PROTOCOL_FLAG_KILL, &pkt);
    st = protocol_parse_control((const uint8_t *)&pkt, sizeof(pkt), &cmd_out, &seq_out);
    ASSERT_TRUE(st == PROTOCOL_OK && cmd_out.armed == false && cmd_out.throttle == 0.0f,
                "7. Kill switch flag overrides armed=false & throttle=0");

    // 3. 测试 CRC 校验拦截比特翻转
    pkt.roll ^= 0x0010; // 人为制造 1 bit 翻转
    st = protocol_parse_control((const uint8_t *)&pkt, sizeof(pkt), &cmd_out, &seq_out);
    ASSERT_TRUE(st == PROTOCOL_ERR_CRC, "8. Corrupted packet rejected by CRC-8");

    // 4. 测试越界数据拦截（例如畸形油门 1500 > 1000）
    protocol_pack_control(&cmd_in, 44U, 0U, &pkt);
    pkt.throttle = 1500U;
    pkt.crc8 = protocol_crc8((const uint8_t *)&pkt, PROTOCOL_CTRL_FRAME_SIZE - 1U);
    st = protocol_parse_control((const uint8_t *)&pkt, sizeof(pkt), &cmd_out, &seq_out);
    ASSERT_TRUE(st == PROTOCOL_ERR_BOUNDS, "9. Out-of-bounds throttle (>1000) rejected");

    // 5. 测试 19 字节遥测帧打包与 CRC
    EulerAngle_t att = { 5.5f * (float)M_PI / 180.0f, -3.2f * (float)M_PI / 180.0f, 120.0f * (float)M_PI / 180.0f };
    MotorOutput_t motors = { 0.42f, 0.45f, 0.41f, 0.44f };
    BleTelemetryPacket_t telem;
    protocol_pack_telemetry(7U, FLIGHT_STATE_ARMED, 3.85f, att, motors, 1120U, &telem);

    uint8_t telem_crc = protocol_crc8((const uint8_t *)&telem, PROTOCOL_TELEM_FRAME_SIZE - 1U);
    ASSERT_TRUE(telem.crc8 == telem_crc && telem.vbat_mv == 3850U && telem.roll_deg10 == 55 && telem.pitch_deg10 == -32,
                "10. Telemetry 19-byte frame packing & CRC verified");

    printf("==================================================================================\n");
    return 0;
}
