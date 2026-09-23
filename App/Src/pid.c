#include "pid.h"
#include <stddef.h>

static float ClampFloat(float value, float minimum, float maximum)
{
  /* 限幅函数：把 value 限制在 minimum 到 maximum 之间。 */
  if (value > maximum)
  {
    return maximum;
  }
  if (value < minimum)
  {
    return minimum;
  }
  return value;
}

void PID_Init(PID_t *pid,
              float kp,
              float ki,
              float kd,
              float output_min,
              float output_max,
              float integral_min,
              float integral_max)
{
  if ((pid == NULL) || (output_min >= output_max) || (integral_min > integral_max))
  {
    return;
  }

  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
  pid->output_min = output_min;
  pid->output_max = output_max;
  pid->integral_min = integral_min;
  pid->integral_max = integral_max;
  pid->integral_separation_ratio = 0.0f;
  PID_Reset(pid);
}

void PID_Reset(PID_t *pid)
{
  if (pid == NULL)
  {
    return;
  }

  pid->integral = 0.0f;
  pid->previous_measurement = 0.0f;
  pid->initialized = 0U;
}

void PID_SetTunings(PID_t *pid, float kp, float ki, float kd)
{
  if (pid == NULL)
  {
    return;
  }

  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
  PID_Reset(pid);
}

void PID_SetIntegralSeparation(PID_t *pid, float ratio)
{
  if (pid == NULL)
  {
    return;
  }

  /* 比例限制在 0.0 到 1.0；0 表示关闭积分分离。 */
  if (ratio < 0.0f)
  {
    ratio = 0.0f;
  }
  else if (ratio > 1.0f)
  {
    ratio = 1.0f;
  }

  pid->integral_separation_ratio = ratio;
}

float PID_Update(PID_t *pid, float setpoint, float measurement, float dt_s)
{
  float error;
  float proportional;
  float derivative;
  float next_integral;
  float separation_threshold;
  float unclamped_output;
  float output;

  if ((pid == NULL) || (!(dt_s > 0.0f)))
  {
    return 0.0f;
  }

  if (pid->initialized == 0U)
  {
    /* 第一次运行只记录测量值，不计算微分，避免首周期跳变。 */
    pid->previous_measurement = measurement;
    pid->initialized = 1U;
  }

  /* 误差 = 目标值 - 实际值。 */
  error = setpoint - measurement;
  /* 比例项：误差越大，输出越大。 */
  proportional = pid->kp * error;
  /* 微分项使用测量值变化率，避免目标值突变时产生微分冲击。 */
  derivative = -pid->kd * (measurement - pid->previous_measurement) / dt_s;

  /* 积分分离：误差超过目标速度的一定比例时暂停积分，防止堵转或大幅超调时积分饱和。 */
  separation_threshold = setpoint * pid->integral_separation_ratio;
  if (separation_threshold < 0.0f)
  {
    separation_threshold = -separation_threshold;
  }

  if (pid->integral_separation_ratio > 0.0f)
  {
    if ((error > separation_threshold) && (pid->integral > 0.0f))
    {
      /* 正向误差很大，积分也是正的，暂停积分，防止继续饱和。 */
      next_integral = pid->integral;
    }
    else if ((error < -separation_threshold) && (pid->integral < 0.0f))
    {
      /* 反向误差很大，积分也是负的，暂停积分。 */
      next_integral = pid->integral;
    }
    else if ((error > separation_threshold) && (pid->integral < 0.0f))
    {
      /* 目标方向突然反转，快速释放负积分。 */
      next_integral = pid->integral * 0.8f;
    }
    else if ((error < -separation_threshold) && (pid->integral > 0.0f))
    {
      /* 目标方向突然反转，快速释放正积分。 */
      next_integral = pid->integral * 0.8f;
    }
    else
    {
      next_integral = pid->integral + pid->ki * error * dt_s;
    }
  }
  else
  {
    next_integral = pid->integral + pid->ki * error * dt_s;
  }

  /* 先限制积分项，防止长时间误差导致积分无限增长。 */
  next_integral = ClampFloat(next_integral, pid->integral_min, pid->integral_max);

  unclamped_output = proportional + next_integral + derivative;
  /* 输出限幅：速度环对应 PWM -1000 到 +1000。 */
  output = ClampFloat(unclamped_output, pid->output_min, pid->output_max);

  /* 条件积分抗饱和：输出已经到达上限且误差仍同向时，不再继续积分。 */
  if ((unclamped_output > pid->output_max) && (error > 0.0f))
  {
    next_integral = pid->integral;
  }
  else if ((unclamped_output < pid->output_min) && (error < 0.0f))
  {
    next_integral = pid->integral;
  }

  pid->integral = next_integral;
  pid->previous_measurement = measurement;
  return output;
}
