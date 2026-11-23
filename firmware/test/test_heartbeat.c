#include <stdio.h>
#include "../src/flight/heartbeat.h"

static int g_failures = 0;

#define CHECK(cond, msg) do { \
    if (!(cond)) { \
        printf("[FAIL] %s (line %d)\n", msg, __LINE__); \
        g_failures++; \
    } else { \
        printf("[PASS] %s\n", msg); \
    } \
} while (0)

int main(void) {
    printf("================ Control Heartbeat Watchdog Test (F-01) ================\n");

    CHECK(heartbeat_stale(1000U, 1100U, 200U) == false,
          "1. Normal tick within timeout (100ms <= 200ms) is alive");
    CHECK(heartbeat_stale(1000U, 1300U, 200U) == true,
          "2. Stalled task beyond timeout (300ms > 200ms) is stale");
    CHECK(heartbeat_stale(0xFFFFFF00U, 0x00000100U, 200U) == true,
          "3. uint32 millis() wraparound stale check (512ms > 200ms) is stale");
    CHECK(heartbeat_stale(0xFFFFFF00U, 0xFFFFFF10U, 200U) == false,
          "4. uint32 millis() near overflow alive check (16ms <= 200ms) is alive");
    CHECK(heartbeat_stale(1000U, 1200U, 200U) == false,
          "5. Exact timeout boundary (200ms == 200ms) stays alive");
    CHECK(heartbeat_stale(1000U, 1201U, 200U) == true,
          "6. One millisecond past timeout (201ms > 200ms) triggers stale");
    CHECK(heartbeat_stale(1001U, 1000U, 200U) == false,
          "7. Cross-core 1ms read/write skew does not underflow to false stale");
    CHECK(heartbeat_stale(0x00000001U, 0xFFFFFFFFU, 200U) == false,
          "8. Cross-core wraparound skew does not underflow to false stale");

    bool fault = heartbeat_fault_latched(false, 1000U, 1300U, 200U, false);
    CHECK(fault, "9. Timeout enters latched motor inhibit");
    fault = heartbeat_fault_latched(fault, 1300U, 1301U, 200U, false);
    CHECK(fault, "10. Fresh heartbeat alone cannot restart motors");
    fault = heartbeat_fault_latched(fault, 1300U, 1302U, 200U, true);
    CHECK(!fault, "11. Healthy explicit disarm clears motor inhibit");
    CHECK(heartbeat_fault_latched(fault, 1302U, 1600U, 200U, false),
          "12. Subsequent stall latches again after reset");

    printf("========================================================================\n");
    if (g_failures == 0) {
        printf("ALL HEARTBEAT TESTS PASSED!\n");
        return 0;
    }
    printf("HEARTBEAT TESTS FAILED: %d\n", g_failures);
    return 1;
}
