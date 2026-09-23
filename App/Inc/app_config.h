#ifndef APP_CONFIG_H
#define APP_CONFIG_H

/* 控制周期。当前测试代码每 10 ms 调用一次控制任务。 */
#define ROBOT_CONTROL_PERIOD_MS       10U

/* 方向修正：先测编码器，再测电机，确认真实方向后只改这里的符号。
   不要一上来就改接线的正负，软件改符号更安全、也更好回退。 */
#define ENCODER_LEFT_SIGN             1
#define ENCODER_RIGHT_SIGN            -1
/* TIM 编码器输入滤波，范围 0x0 到 0xF；数值越大滤波越强。 */
#define ENCODER_IC_FILTER             0x0FU
#define MOTOR_LEFT_FORWARD_INVERT     0U
#define MOTOR_RIGHT_FORWARD_INVERT    0U

/* PID 输出的 PWM 范围是 -MOTOR_MAX_DUTY 到 +MOTOR_MAX_DUTY。 */
#define MOTOR_MAX_DUTY                1000U
/* 电机克服静摩擦所需的最小 PWM，0 表示暂时不做死区补偿。 */
#define MOTOR_LEFT_MIN_DUTY           0U
#define MOTOR_RIGHT_MIN_DUTY          0U

/* 常见数字循迹模块检测到黑线时输出低电平，因此默认用低电平有效。 */
#define TRACKING_ACTIVE_LEVEL         0U
/* 如果传感器 1 到 4 的左右顺序接反了，把这里改成 1U。 */
#define TRACKING_REVERSE_ORDER        0U

/* 左右轮速度环初始 PID 参数。第一次调参时 Ki 和 Kd 保持 0。
   这里的初值取得比较保守，按 Kp -> Ki -> Kd 的顺序一点点加。 */
#define WHEEL_PID_KP                  2.00f
#define WHEEL_PID_KI                  0.30f
#define WHEEL_PID_KD                  0.00f
/* 速度环输出限幅，对应电机 PWM -1000 到 +1000。 */
#define WHEEL_PID_OUTPUT_MIN         -1000.0f
#define WHEEL_PID_OUTPUT_MAX          1000.0f
/* 积分限幅，防止积分项无限增大。 */
#define WHEEL_PID_INTEGRAL_MIN       -1000.0f
#define WHEEL_PID_INTEGRAL_MAX        1000.0f
/* 速度环积分分离比例。误差绝对值超过目标速度的该比例时暂停积分。
   例如目标是 600，比例 0.80，实际速度低于 120 时才暂停积分，
   可以避免轮子被完全堵住后松手时速度冲高。 */
#define WHEEL_PID_INTEGRAL_SEPARATION_RATIO 0.80f
/* 允许设置的最大、最小目标速度，单位是 encoder counts/s。 */
#define WHEEL_SPEED_MIN              -3000.0f
#define WHEEL_SPEED_MAX               3000.0f

/* 循迹位置环 PID。输入误差范围是 -3 到 +3。 */
#define LINE_PID_KP                   120.0f
#define LINE_PID_KI                   0.00f
#define LINE_PID_KD                   0.05f
#define LINE_PID_OUTPUT_MIN          -700.0f
#define LINE_PID_OUTPUT_MAX           700.0f
#define LINE_PID_INTEGRAL_MIN        -200.0f
#define LINE_PID_INTEGRAL_MAX         200.0f

/* 循迹基础速度，单位同样是 encoder counts/s。 */
#define LINE_BASE_SPEED_COUNTS_S      1800.0f
/* 如果循迹修正方向相反，把 1.0f 改成 -1.0f。 */
#define LINE_STEERING_SIGN            -1.0f
/* 四路传感器都丢线后，超过这个时间就停车。 */
#define LINE_LOST_TIMEOUT_MS          400U

/* USART1 调试串口波特率，必须和电脑/HC-05 设置一致。 */
#define DEBUG_UART_BAUD               115200U

/* MPU6050 使用 I2C2：SCL=PB10，SDA=PB11。
   AD0 悬空或接 GND 时，7 位地址为 0x68。 */
#define MPU6050_I2C_ADDRESS           0x68U
/* 量程设为正负 2000 度每秒时，灵敏度为 16.4 LSB/(度每秒)。 */
#define MPU6050_GYRO_LSB_PER_DPS      16.4f

#endif
