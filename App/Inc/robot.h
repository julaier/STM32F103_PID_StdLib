#ifndef ROBOT_H
#define ROBOT_H

#include "encoder.h"
#include "motor.h"
#include "pid.h"
#include "tracking.h"
#include <stdint.h>

typedef struct
{
  /* 左右编码器和左右电机对象。 */
  Encoder_t left_encoder;
  Encoder_t right_encoder;
  Motor_t left_motor;
  Motor_t right_motor;
  /* 四路循迹传感器。 */
  TrackingSensor_t tracking;
  /* 左轮速度环、右轮速度环和循迹位置环。 */
  PID_t left_speed_pid;
  PID_t right_speed_pid;
  PID_t line_pid;

  /* 当前循迹基础速度和左右轮目标速度，单位 counts/s。 */
  float base_speed_counts_s;
  float left_target_counts_s;
  float right_target_counts_s;
  /* 最近一次循迹误差，负值表示线偏左，正值表示线偏右。 */
  float line_error;
  /* 速度环最终输出的 PWM 命令。 */
  int16_t left_command;
  int16_t right_command;
  /* 连续丢线时间和是否检测到线的状态。 */
  uint32_t line_lost_ms;
  uint8_t line_found;
} Robot_t;

/* 初始化所有硬件模块和 PID，返回 1 表示成功。 */
uint8_t Robot_Init(Robot_t *robot);
void Robot_SetBaseSpeed(Robot_t *robot, float speed_counts_s);
void Robot_SetWheelPid(Robot_t *robot, float kp, float ki, float kd);
void Robot_SetLinePid(Robot_t *robot, float kp, float ki, float kd);
/* 只更新编码器，用于第 1 步确认方向和读数。 */
void Robot_UpdateEncoders(Robot_t *robot, uint32_t dt_ms);
/* 给定左右轮目标速度，速度环自动计算 PWM。 */
void Robot_UpdateSpeedControl(Robot_t *robot,
                              float left_target_counts_s,
                              float right_target_counts_s,
                              uint32_t dt_ms);
/* 循迹位置环和左右轮速度环组合运行。 */
void Robot_UpdateLineFollowing(Robot_t *robot, uint32_t dt_ms);
/* 停止电机并复位所有 PID。 */
void Robot_Stop(Robot_t *robot);

#endif
