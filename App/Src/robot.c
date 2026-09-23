#include "robot.h"
#include "app_config.h"
#include <stddef.h>

static int16_t CommandFromFloat(float value)
{
  /* PID 输出是浮点数，最终要转换成 -1000 到 +1000 的电机命令。 */
  if (value > (float)MOTOR_MAX_DUTY)
  {
    return (int16_t)MOTOR_MAX_DUTY;
  }
  if (value < -(float)MOTOR_MAX_DUTY)
  {
    return -(int16_t)MOTOR_MAX_DUTY;
  }
  return (int16_t)value;
}

static float ClampSpeed(float value)
{
  if (value > WHEEL_SPEED_MAX)
  {
    return WHEEL_SPEED_MAX;
  }
  if (value < WHEEL_SPEED_MIN)
  {
    return WHEEL_SPEED_MIN;
  }
  return value;
}

uint8_t Robot_Init(Robot_t *robot)
{
  /* 四个循迹传感器从左到右对应 PB12、PB13、PB14、PB15。 */
  static const uint16_t tracking_pins[TRACKING_SENSOR_COUNT] =
  {
    GPIO_Pin_12, GPIO_Pin_13, GPIO_Pin_14, GPIO_Pin_15
  };

  /* 左电机：TIM3_CH1(PA6)，方向 PA2/PA3。 */
  Motor_Config_t left_config =
  {
    TIM3,
    1U,
    GPIOA,
    GPIO_Pin_6,
    GPIOA,
    GPIO_Pin_2,
    GPIO_Pin_3,
    MOTOR_LEFT_MIN_DUTY,
    MOTOR_LEFT_FORWARD_INVERT
  };

  /* 右电机：TIM3_CH2(PA7)，方向 PA4/PA5。 */
  Motor_Config_t right_config =
  {
    TIM3,
    2U,
    GPIOA,
    GPIO_Pin_7,
    GPIOA,
    GPIO_Pin_4,
    GPIO_Pin_5,
    MOTOR_RIGHT_MIN_DUTY,
    MOTOR_RIGHT_FORWARD_INVERT
  };

  if (robot == NULL)
  {
    return 0U;
  }

  /* 左轮使用 TIM2，右轮使用 TIM4。 */
  if (Encoder_Init(&robot->left_encoder, TIM2, ENCODER_LEFT_SIGN) == 0U)
  {
    return 0U;
  }
  if (Encoder_Init(&robot->right_encoder, TIM4, ENCODER_RIGHT_SIGN) == 0U)
  {
    return 0U;
  }
  /* 初始化两路 L298N 电机输出。 */
  if (Motor_Init(&robot->left_motor, &left_config) == 0U)
  {
    return 0U;
  }
  if (Motor_Init(&robot->right_motor, &right_config) == 0U)
  {
    return 0U;
  }

  Tracking_Init(&robot->tracking,
                GPIOB,
                tracking_pins,
                (TRACKING_ACTIVE_LEVEL == 0U) ? Bit_RESET : Bit_SET,
                TRACKING_REVERSE_ORDER);
  Tracking_ConfigPins(GPIOB, tracking_pins);

  /* 速度环输入是 counts/s，输出是 PWM。 */
  PID_Init(&robot->left_speed_pid,
           WHEEL_PID_KP,
           WHEEL_PID_KI,
           WHEEL_PID_KD,
           WHEEL_PID_OUTPUT_MIN,
           WHEEL_PID_OUTPUT_MAX,
           WHEEL_PID_INTEGRAL_MIN,
           WHEEL_PID_INTEGRAL_MAX);
  PID_Init(&robot->right_speed_pid,
           WHEEL_PID_KP,
           WHEEL_PID_KI,
           WHEEL_PID_KD,
           WHEEL_PID_OUTPUT_MIN,
           WHEEL_PID_OUTPUT_MAX,
           WHEEL_PID_INTEGRAL_MIN,
           WHEEL_PID_INTEGRAL_MAX);
  /* 速度环启用积分分离：误差很大时暂停积分，避免堵转后松手冲高。 */
  PID_SetIntegralSeparation(&robot->left_speed_pid,
                            WHEEL_PID_INTEGRAL_SEPARATION_RATIO);
  PID_SetIntegralSeparation(&robot->right_speed_pid,
                            WHEEL_PID_INTEGRAL_SEPARATION_RATIO);
  /* 循迹环输入是线位置误差，输出是左右轮目标速度的差值。 */
  PID_Init(&robot->line_pid,
           LINE_PID_KP,
           LINE_PID_KI,
           LINE_PID_KD,
           LINE_PID_OUTPUT_MIN,
           LINE_PID_OUTPUT_MAX,
           LINE_PID_INTEGRAL_MIN,
           LINE_PID_INTEGRAL_MAX);

  robot->base_speed_counts_s = LINE_BASE_SPEED_COUNTS_S;
  robot->left_target_counts_s = 0.0f;
  robot->right_target_counts_s = 0.0f;
  robot->line_error = 0.0f;
  robot->left_command = 0;
  robot->right_command = 0;
  robot->line_lost_ms = 0U;
  robot->line_found = 0U;

  Motor_Stop(&robot->left_motor);
  Motor_Stop(&robot->right_motor);
  return 1U;
}

void Robot_SetBaseSpeed(Robot_t *robot, float speed_counts_s)
{
  if (robot == NULL)
  {
    return;
  }

  robot->base_speed_counts_s = ClampSpeed(speed_counts_s);
}

void Robot_SetWheelPid(Robot_t *robot, float kp, float ki, float kd)
{
  if (robot == NULL)
  {
    return;
  }

  PID_SetTunings(&robot->left_speed_pid, kp, ki, kd);
  PID_SetTunings(&robot->right_speed_pid, kp, ki, kd);
}

void Robot_SetLinePid(Robot_t *robot, float kp, float ki, float kd)
{
  if (robot == NULL)
  {
    return;
  }

  PID_SetTunings(&robot->line_pid, kp, ki, kd);
}

void Robot_UpdateEncoders(Robot_t *robot, uint32_t dt_ms)
{
  if (robot == NULL)
  {
    return;
  }

  (void)Encoder_Update(&robot->left_encoder, dt_ms);
  (void)Encoder_Update(&robot->right_encoder, dt_ms);
}

void Robot_UpdateSpeedControl(Robot_t *robot,
                              float left_target_counts_s,
                              float right_target_counts_s,
                              uint32_t dt_ms)
{
  float left_pwm;
  float right_pwm;
  float dt_s;

  if ((robot == NULL) || (dt_ms == 0U))
  {
    return;
  }

  Robot_UpdateEncoders(robot, dt_ms);
  robot->left_target_counts_s = ClampSpeed(left_target_counts_s);
  robot->right_target_counts_s = ClampSpeed(right_target_counts_s);
  dt_s = (float)dt_ms / 1000.0f;

  /* 内外环都是速度环，误差来自编码器实际速度。 */
  left_pwm = PID_Update(&robot->left_speed_pid,
                        robot->left_target_counts_s,
                        robot->left_encoder.speed_counts_s,
                        dt_s);
  right_pwm = PID_Update(&robot->right_speed_pid,
                         robot->right_target_counts_s,
                         robot->right_encoder.speed_counts_s,
                         dt_s);

  robot->left_command = CommandFromFloat(left_pwm);
  robot->right_command = CommandFromFloat(right_pwm);
  Motor_SetCommand(&robot->left_motor, robot->left_command);
  Motor_SetCommand(&robot->right_motor, robot->right_command);
}

void Robot_UpdateLineFollowing(Robot_t *robot, uint32_t dt_ms)
{
  float correction;
  float dt_s;
  float left_pwm;
  float right_pwm;

  if ((robot == NULL) || (dt_ms == 0U))
  {
    return;
  }

  Robot_UpdateEncoders(robot, dt_ms);
  Tracking_Update(&robot->tracking);
  robot->line_found = robot->tracking.line_found;
  robot->line_error = robot->tracking.error;
  dt_s = (float)dt_ms / 1000.0f;

  if (robot->line_found != 0U)
  {
    robot->line_lost_ms = 0U;
    /* 修正量为正时左轮加速、右轮减速；修正量为负时相反。 */
    correction = LINE_STEERING_SIGN *
                 PID_Update(&robot->line_pid, 0.0f, robot->line_error, dt_s);

    robot->left_target_counts_s =
        ClampSpeed(robot->base_speed_counts_s + correction);
    robot->right_target_counts_s =
        ClampSpeed(robot->base_speed_counts_s - correction);
  }
  else
  {
    /* 短暂丢线时保留上次目标速度；超过阈值则停车。 */
    if (robot->line_lost_ms < LINE_LOST_TIMEOUT_MS)
    {
      robot->line_lost_ms += dt_ms;
      /* 丢线期间保持上一条控制周期的左右目标速度，不做额外转向。 */
    }

    if (robot->line_lost_ms >= LINE_LOST_TIMEOUT_MS)
    {
      Robot_Stop(robot);
      return;
    }
  }

  left_pwm = PID_Update(&robot->left_speed_pid,
                        robot->left_target_counts_s,
                        robot->left_encoder.speed_counts_s,
                        dt_s);
  right_pwm = PID_Update(&robot->right_speed_pid,
                         robot->right_target_counts_s,
                         robot->right_encoder.speed_counts_s,
                         dt_s);

  robot->left_command = CommandFromFloat(left_pwm);
  robot->right_command = CommandFromFloat(right_pwm);
  Motor_SetCommand(&robot->left_motor, robot->left_command);
  Motor_SetCommand(&robot->right_motor, robot->right_command);
}

void Robot_Stop(Robot_t *robot)
{
  if (robot == NULL)
  {
    return;
  }

  Motor_Stop(&robot->left_motor);
  Motor_Stop(&robot->right_motor);
  PID_Reset(&robot->left_speed_pid);
  PID_Reset(&robot->right_speed_pid);
  PID_Reset(&robot->line_pid);
  robot->left_target_counts_s = 0.0f;
  robot->right_target_counts_s = 0.0f;
  robot->left_command = 0;
  robot->right_command = 0;
}
