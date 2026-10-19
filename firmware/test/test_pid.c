#include <stdio.h>
// 1. 引用我们上面写好的 PID 算法头文件
#include "../src/flight/pid.h"

int main() {
    // 2. 声明一个 PID 控制器结构体变量
    PID_t pid;

    // 3. 初始化这套 PID 参数：Kp=2.0, Ki=0.5, Kd=0.1，限幅分别设为 10.0 和 20.0
    pid_init(&pid, 2.0f, 0.5f, 0.1f, 10.0f, 20.0f);

    // 4. 设置仿真初始场景：
    float current_angle = 10.0f;  // 假设一阵风把飞机吹歪了 10 度
    float target_angle  = 0.0f;   // 飞控的目标是让飞机保持 0 度水平
    float dt = 0.01f;             // 模拟控制循环每 10 毫秒（0.01秒）计算一次

    printf("================ PID 算法仿真验证 ================\n");

    // 5. 用一个 for 循环模拟飞控连续运转 15 拍（即 0.15 秒内的控制过程）
    for (int step = 1; step <= 15; step++) {
        // 第一小步：计算当前误差 = 目标 - 实际
        float error = target_angle - current_angle;

        // 第二小步：将误差丢进 PID 算法，算出力矩纠偏输出
        float output = pid_update(&pid, error, dt);

        // 第三小步：把每一步的数值打印在屏幕上观察
       printf("Step %2d | Angle: %6.2f deg | Error: %6.2f | Output: %6.2f\n",
       step, current_angle, error, output);

        // 第四小步：模拟物理世界电机的推力把角度拉回来：
        // 纠偏输出越大，飞机角度被回正得越快
        current_angle += output * 0.15f;
    }
    printf("==================================================\n");

    return 0;
}