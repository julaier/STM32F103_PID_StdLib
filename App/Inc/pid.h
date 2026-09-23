#ifndef PID_H
#define PID_H

#include <stdint.h>

typedef struct
{
  /* 比例、积分、微分三个可调参数。 */
  float kp;
  float ki;
  float kd;
  /* 积分累计值，会被 integral_min/integral_max 限制。 */
  float integral;
  /* 上一次测量值，用于避免目标值突变时产生微分冲击。 */
  float previous_measurement;
  /* PID 输出的最小值和最大值。 */
  float output_min;
  float output_max;
  /* 积分项最小值和最大值，用于抗积分饱和。 */
  float integral_min;
  float integral_max;
  /* 积分分离比例。误差绝对值大于目标绝对值乘该比例时暂停积分；0 表示关闭。 */
  float integral_separation_ratio;
  /* 第一次计算时记录测量值，避免首周期微分值异常。 */
  uint8_t initialized;
} PID_t;

/* 初始化 PID 参数和限幅范围。 */
void PID_Init(PID_t *pid,
              float kp,
              float ki,
              float kd,
              float output_min,
              float output_max,
              float integral_min,
              float integral_max);
/* 清空积分和微分历史，保留 Kp/Ki/Kd。 */
void PID_Reset(PID_t *pid);
/* 运行时修改 Kp/Ki/Kd，方便后续扩展在线调参。 */
void PID_SetTunings(PID_t *pid, float kp, float ki, float kd);
/* 设置积分分离比例，范围 0.0 到 1.0；0 表示关闭积分分离。 */
void PID_SetIntegralSeparation(PID_t *pid, float ratio);
/* 计算一次 PID：setpoint 是目标值，measurement 是实际值，dt_s 是周期秒数。 */
float PID_Update(PID_t *pid, float setpoint, float measurement, float dt_s);

#endif
