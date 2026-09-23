#include "user_code.h"
#include "app_config.h"
#include "bsp_time.h"
#include "bsp_uart.h"
#include "robot.h"
#include "mpu6050.h"
#include <stdarg.h>
#include <stdio.h>

/* 测试模式选择，每次只改这一个宏：
     1 = 编码器测试
     2 = 电机开环测试
     3 = 左右轮速度 PID
     4 = 循迹原始值测试
     5 = 完整循迹
     6 = 90 度转弯脉冲测试
     7 = MPU6050 陀螺仪 Z 轴测试 */
#define ROBOT_TEST_MODE              6U
/* 模式 3 的左右轮目标速度，单位是 encoder counts/s。 */
#define WHEEL_TEST_TARGET_COUNTS_S   2000.0f
/* 模式 2 的开环 PWM 命令，建议先从 200 开始。 */
#define MOTOR_TEST_COMMAND           800

/* 模式 6：90 度转弯脉冲测试。
   1 表示左转，-1 表示右转，测试时每次只改这个方向。 */
#define TURN_TEST_DIRECTION          1
/* 上电后等待时间，先把小车摆到赛道起点。 */
#define TURN_TEST_START_DELAY_MS     3000U
/* 转弯时两轮目标速度，单位 counts/s。
   内轮先反向转，外轮正向转，形成一个原地转弯。 */
#define TURN_TEST_INNER_COUNTS_S     500.0f
#define TURN_TEST_OUTER_COUNTS_S      1600.0f
/* 最长转弯时间，防止找不到线时一直旋转。 */
#define TURN_TEST_MAX_MS              7000U
/* 中间两路是 bit1(-1) 和 bit2(+1)，两路同时检测到线才算到中心。 */
#define TURN_TEST_CENTER_MASK         0x06U

/* 模式 7：MPU6050 静止校准和 Z 轴角速度测试。
   校准时小车必须保持静止，500 次乘 2 ms 约需要 1 秒。 */
#define GYRO_TEST_CALIBRATION_SAMPLES 500U
#define GYRO_TEST_CALIBRATION_DELAY_MS 2U
/* 模式 7 每个循环读取一次，主循环后面的延时也是 10 ms。 */
#define GYRO_TEST_DT_S                 0.01f

static Robot_t g_robot;
static uint32_t g_loop_count;

#if (ROBOT_TEST_MODE == 6U)
/* 模式 6 状态：等待启动、等待丢线、等待重新找到线、完成。 */
#define TURN6_STATE_WAIT_START      0U
#define TURN6_STATE_WAIT_LOST       1U
#define TURN6_STATE_WAIT_FOUND      2U
#define TURN6_STATE_DONE            3U

static uint8_t g_turn_state;
static uint8_t g_turn_confirm_count;
static uint8_t g_turn_timeout;
static uint32_t g_turn_elapsed_ms;
static int32_t g_turn_start_left;
static int32_t g_turn_start_right;
static int32_t g_turn_end_left;
static int32_t g_turn_end_right;
static int32_t g_turn_diff_left;
static int32_t g_turn_diff_right;
#endif

#if (ROBOT_TEST_MODE == 7U)
/* 模式 7 保存最近一次陀螺仪读数、零偏和积分角度。 */
static int16_t g_gyro_raw_z;
static float g_gyro_bias_z_raw;
static float g_gyro_rate_z_dps;
static float g_gyro_angle_z_deg;
static uint8_t g_gyro_read_ok;
#endif

/* 简单的阻塞式串口打印函数，只用于测试和调参。 */
static void Debug_Printf(const char *format, ...)
{
  char buffer[128];
  va_list args;
  int length;

  va_start(args, format);
  length = vsnprintf(buffer, sizeof(buffer), format, args);
  va_end(args);

  if (length < 0)
  {
    return;
  }
  if (length >= (int)sizeof(buffer))
  {
    length = (int)sizeof(buffer) - 1;
  }

  /* 把格式化后的字符串逐字节发送到串口。 */
  BSP_UART_SendBuffer((const uint8_t *)buffer, (uint16_t)length);
}

#if (ROBOT_TEST_MODE == 6U)
static void Turn6_Finish(uint8_t timed_out)
{
  int32_t end_left;
  int32_t end_right;

  /* 先刹车，再读取最后一次编码器增量，避免停车后继续滑行造成误差。 */
  Robot_Stop(&g_robot);
  Motor_Brake(&g_robot.left_motor);
  Motor_Brake(&g_robot.right_motor);
  Robot_UpdateEncoders(&g_robot, ROBOT_CONTROL_PERIOD_MS);

  end_left = Encoder_GetTotal(&g_robot.left_encoder);
  end_right = Encoder_GetTotal(&g_robot.right_encoder);
  g_turn_end_left = end_left;
  g_turn_end_right = end_right;
  g_turn_diff_left = end_left - g_turn_start_left;
  g_turn_diff_right = end_right - g_turn_start_right;
  g_turn_timeout = timed_out;
  g_turn_state = TURN6_STATE_DONE;
}
#endif

void Robot_UserSetup(void)
{
  /* 先初始化调试串口，方便初始化失败时看到提示。 */
  BSP_UART_Init();

  /* 初始化 TIM2/TIM4、TIM3、电机方向引脚、循迹 GPIO 和 PID。 */
  if (Robot_Init(&g_robot) == 0U)
  {
    BSP_UART_SendString("Robot init failed.\r\n");
    while (1)
    {
    }
  }

  g_loop_count = 0U;

#if (ROBOT_TEST_MODE == 6U)
  /* 模式 6 每次上电都从等待开始，并清空上一次的记录。 */
  g_turn_state = TURN6_STATE_WAIT_START;
  g_turn_confirm_count = 0U;
  g_turn_timeout = 0U;
  g_turn_elapsed_ms = 0U;
  g_turn_start_left = 0;
  g_turn_start_right = 0;
  g_turn_end_left = 0;
  g_turn_end_right = 0;
  g_turn_diff_left = 0;
  g_turn_diff_right = 0;
#endif

#if (ROBOT_TEST_MODE == 7U)
  /* AD0 不接时地址是 0x68。初始化失败时直接停在提示信息这里。 */
  g_gyro_raw_z = 0;
  g_gyro_bias_z_raw = 0.0f;
  g_gyro_rate_z_dps = 0.0f;
  g_gyro_angle_z_deg = 0.0f;
  g_gyro_read_ok = 0U;

  if (MPU6050_Init() == 0U)
  {
    BSP_UART_SendString("MPU6050 no ACK at 0x68 or 0x69. Check VCC/GND/SCL=PB10/SDA=PB11.\r\n");
    while (1)
    {
    }
  }

  Debug_Printf("MPU6050 address=0x%02X\r\n",
               (unsigned int)MPU6050_GetAddress());
  BSP_UART_SendString("MPU6050 init OK. Keep the car still...\r\n");
  if (MPU6050_CalibrateGyroZ(GYRO_TEST_CALIBRATION_SAMPLES,
                            GYRO_TEST_CALIBRATION_DELAY_MS,
                            &g_gyro_bias_z_raw) == 0U)
  {
    BSP_UART_SendString("MPU6050 calibration failed.\r\n");
    while (1)
    {
    }
  }

  Debug_Printf("GYRO calib done bias_raw_x100=%ld\r\n",
               (long)(g_gyro_bias_z_raw * 100.0f));
#endif
}

void Robot_UserLoop(void)
{
  g_loop_count++;

#if (ROBOT_TEST_MODE == 1U)
  /* 模式 1：只读编码器，轮子不动也可以观察计数。 */
  Robot_UpdateEncoders(&g_robot, ROBOT_CONTROL_PERIOD_MS);

#elif (ROBOT_TEST_MODE == 2U)
  /* 模式 2：直接给固定 PWM，每约 2 秒换一次正反转。 */
  {
    int16_t command = ((g_loop_count / 200U) % 2U == 0U) ?
                      MOTOR_TEST_COMMAND : -MOTOR_TEST_COMMAND;
    Motor_SetCommand(&g_robot.left_motor, command);
    Motor_SetCommand(&g_robot.right_motor, command);
  }

#elif (ROBOT_TEST_MODE == 3U)
  /* 模式 3：先不要上循迹，只让速度 PID 控制左右轮。 */
  Robot_UpdateSpeedControl(&g_robot,
                           WHEEL_TEST_TARGET_COUNTS_S,
                           WHEEL_TEST_TARGET_COUNTS_S,
                           ROBOT_CONTROL_PERIOD_MS);

#elif (ROBOT_TEST_MODE == 4U)
  /* 模式 4：只观察循迹模块原始值，不驱动电机。 */
  Robot_UpdateEncoders(&g_robot, ROBOT_CONTROL_PERIOD_MS);
  Tracking_Update(&g_robot.tracking);

#elif (ROBOT_TEST_MODE == 5U)
  /* 模式 5：速度环 + 循迹环一起运行。 */
  Robot_UpdateLineFollowing(&g_robot, ROBOT_CONTROL_PERIOD_MS);

#elif (ROBOT_TEST_MODE == 6U)
  /* 模式 6：固定方向原地转弯，记录 90 度附近左右轮脉冲。 */
  Tracking_Update(&g_robot.tracking);

  if (g_turn_state == TURN6_STATE_WAIT_START)
  {
    /* 等待启动时间结束，期间只更新编码器，不驱动电机。 */
    Robot_UpdateEncoders(&g_robot, ROBOT_CONTROL_PERIOD_MS);
    g_turn_elapsed_ms += ROBOT_CONTROL_PERIOD_MS;
    if (g_turn_elapsed_ms >= TURN_TEST_START_DELAY_MS)
    {
      g_turn_start_left = Encoder_GetTotal(&g_robot.left_encoder);
      g_turn_start_right = Encoder_GetTotal(&g_robot.right_encoder);
      g_turn_elapsed_ms = 0U;
      g_turn_confirm_count = 0U;
      g_turn_state = TURN6_STATE_WAIT_LOST;
    }
  }
  else if ((g_turn_state == TURN6_STATE_WAIT_LOST) ||
           (g_turn_state == TURN6_STATE_WAIT_FOUND))
  {
    float left_target;
    float right_target;

    /* 左转时左轮内切并反转，右轮外圈前进；右转时反过来。 */
    if (TURN_TEST_DIRECTION > 0)
    {
      left_target = TURN_TEST_INNER_COUNTS_S;
      right_target = TURN_TEST_OUTER_COUNTS_S;
    }
    else
    {
      left_target = TURN_TEST_OUTER_COUNTS_S;
      right_target = TURN_TEST_INNER_COUNTS_S;
    }

    Robot_UpdateSpeedControl(&g_robot,
                             left_target,
                             right_target,
                             ROBOT_CONTROL_PERIOD_MS);
    g_turn_elapsed_ms += ROBOT_CONTROL_PERIOD_MS;

    if (g_turn_state == TURN6_STATE_WAIT_LOST)
    {
      /* 先确认已经离开原线，避免刚上电就误判找到新线。 */
      if (g_robot.tracking.line_found == 0U)
      {
        g_turn_confirm_count++;
      }
      else
      {
        g_turn_confirm_count = 0U;
      }
/*111222*/
      if (g_turn_confirm_count >= 2U)
      {
        g_turn_confirm_count = 0U;
        g_turn_state = TURN6_STATE_WAIT_FOUND;
      }
    }
    else
    {
      /* 只有中间两路同时检测到线才结束，外侧灯检测到线不算。 */
      if ((g_robot.tracking.active_mask & TURN_TEST_CENTER_MASK) ==
          TURN_TEST_CENTER_MASK)
      {
        g_turn_confirm_count++;
      }
      else
      {
        g_turn_confirm_count = 0U;
      }

      if (g_turn_confirm_count >= 2U)
      {
        Turn6_Finish(0U);
      }
    }

    /* 如果一直找不到线，记录最终脉冲并停车，避免长时间旋转。 */
    if ((g_turn_state != TURN6_STATE_DONE) &&
        (g_turn_elapsed_ms >= TURN_TEST_MAX_MS))
    {
      Turn6_Finish(1U);
    }
  }

#elif (ROBOT_TEST_MODE == 7U)
  /* 模式 7：只读陀螺仪，不驱动电机。
     得到的 rate 是 Z 轴角速度，单位是度每秒。
     小车绕竖直轴转动时，rate 会变正或变负。
     angle 是把这个角速度按时间累加出来的角度。 */
  {
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;

    if (MPU6050_ReadGyroRaw(&gyro_x, &gyro_y, &gyro_z) != 0U)
    {
      g_gyro_read_ok = 1U;
      g_gyro_raw_z = gyro_z;
      g_gyro_rate_z_dps =
          ((float)gyro_z - g_gyro_bias_z_raw) / MPU6050_GYRO_LSB_PER_DPS;
      g_gyro_angle_z_deg += g_gyro_rate_z_dps * GYRO_TEST_DT_S;
    }
    else
    {
      g_gyro_read_ok = 0U;
    }
  }

#else
#error "ROBOT_TEST_MODE must be 1, 2, 3, 4, 5, 6, or 7"
#endif

  if ((g_loop_count % 10U) == 0U)
  {
    /* 每 10 个周期打印一次，也就是约 100 ms 一次。 */
#if (ROBOT_TEST_MODE == 1U)
    Debug_Printf("ENC Lspd=%ld Rspd=%ld Ltotal=%ld Rtotal=%ld\r\n",
                 (long)g_robot.left_encoder.speed_counts_s,
                 (long)g_robot.right_encoder.speed_counts_s,
                 (long)g_robot.left_encoder.total_count,
                 (long)g_robot.right_encoder.total_count);
#elif (ROBOT_TEST_MODE == 2U)
    Debug_Printf("MOTOR Lcmd=%d Rcmd=%d\r\n",
                 (int)Motor_GetCommand(&g_robot.left_motor),
                 (int)Motor_GetCommand(&g_robot.right_motor));
#elif (ROBOT_TEST_MODE == 3U)
    Debug_Printf("PID Ltarget=%d Lspeed=%ld Lpwm=%d Rt=%d Rs=%ld Rpwm=%d\r\n",
                 (int)g_robot.left_target_counts_s,
                 (long)g_robot.left_encoder.speed_counts_s,
                 (int)g_robot.left_command,
                 (int)g_robot.right_target_counts_s,
                 (long)g_robot.right_encoder.speed_counts_s,
                 (int)g_robot.right_command);
#elif (ROBOT_TEST_MODE == 4U)
    Debug_Printf("TRACK mask=0x%X err=%d found=%u\r\n",
                 (unsigned int)g_robot.tracking.active_mask,
                 (int)g_robot.tracking.error,
                 (unsigned int)g_robot.tracking.line_found);
#elif (ROBOT_TEST_MODE == 5U)
    Debug_Printf("LINE found=%u err=%d Lt=%d Ls=%ld Lpwm=%d Rt=%d Rs=%ld Rpwm=%d\r\n",
                 (unsigned int)g_robot.line_found,
                 (int)g_robot.line_error,
                 (int)g_robot.left_target_counts_s,
                 (long)g_robot.left_encoder.speed_counts_s,
                 (int)g_robot.left_command,
                 (int)g_robot.right_target_counts_s,
                 (long)g_robot.right_encoder.speed_counts_s,
                 (int)g_robot.right_command);
#elif (ROBOT_TEST_MODE == 6U)
    if (g_turn_state == TURN6_STATE_DONE)
    {
      Debug_Printf("T6 to=%u Lstart=%ld Ldiff=%ld Rstart=%ld Rdiff=%ld\r\n",
                   (unsigned int)g_turn_timeout,
                   (long)g_turn_start_left,
                   (long)g_turn_diff_left,
                   (long)g_turn_start_right,
                   (long)g_turn_diff_right);
    }
#elif (ROBOT_TEST_MODE == 7U)
    if (g_gyro_read_ok != 0U)
    {
      Debug_Printf("GYRO z=%d rate_x100=%ld angle_x10=%ld\r\n",
                   (int)g_gyro_raw_z,
                   (long)(g_gyro_rate_z_dps * 100.0f),
                   (long)(g_gyro_angle_z_deg * 10.0f));
    }
    else
    {
      BSP_UART_SendString("GYRO read failed.\r\n");
    }
#endif
  }

  /* 固定 10 ms 节拍；调试阶段可以先用阻塞延时。 */
  BSP_DelayMs(ROBOT_CONTROL_PERIOD_MS);
}
