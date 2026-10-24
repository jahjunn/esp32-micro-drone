#include <stdio.h>
#include "../src/flight/mixer.h"

// 辅助函数：调用混控器并打印结果
// name 为场景名称，其余参数为混控器输入
static void show(const char *name, float throttle, float roll, float pitch, float yaw) {
    MotorOutput_t out;                                  // 存放 4 个电机的输出
    mixer_quad_x(throttle, roll, pitch, yaw, &out);     // 调用混控算法

    // %-14s 表示名称左对齐、占 14 个字符宽，让输出整齐
    printf("%-14s | M1=%.2f M2=%.2f M3=%.2f M4=%.2f\n",
           name, out.m1, out.m2, out.m3, out.m4);
}

int main() {
    printf("================ Mixer Test ================\n");

    // 场景 1：悬停，没有任何修正量
    // 预期：4 个电机完全相同，都是 0.50
    show("hover",      0.50f,  0.00f,  0.00f,  0.00f);

    // 场景 2：横滚修正 +0.1
    // 预期：右侧 M1、M2 增大到 0.60；左侧 M3、M4 减小到 0.40
    show("roll right", 0.50f,  0.10f,  0.00f,  0.00f);

    // 场景 3：俯仰修正 +0.1
    // 预期：后侧 M2、M3 增大到 0.60；前侧 M1、M4 减小到 0.40
    show("pitch fwd",  0.50f,  0.00f,  0.10f,  0.00f);

    // 场景 4：偏航修正 +0.1
    // 预期：CCW 对角 M1、M3 增大到 0.60；CW 对角 M2、M4 减小到 0.40
    show("yaw",        0.50f,  0.00f,  0.00f,  0.10f);

    // 场景 5：高油门 + 大修正量，测试上限截断
    // 预期：M1、M2 原本会超过 1.0，应被截断为 1.00
    show("overflow",   0.95f,  0.20f,  0.00f,  0.00f);

    // 场景 6：低油门 + 大修正量，测试下限截断
    // 预期：M3、M4 原本会小于 0，应被截断为 0.00
    show("underflow",  0.05f,  0.20f,  0.00f,  0.00f);

    printf("============================================\n");
    return 0;
}