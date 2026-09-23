# STM32F103 标准库版：编码器、电机、循迹与 PID

这套代码使用 **STM32F10x 标准外设库（StdPeriph）**，不是 HAL 库。

用到的库函数包括：

- `RCC_APB1PeriphClockCmd()`、`RCC_APB2PeriphClockCmd()`
- `GPIO_Init()`、`GPIO_SetBits()`、`GPIO_ResetBits()`
- `TIM_TimeBaseInit()`、`TIM_EncoderInterfaceConfig()`
- `TIM_OC1Init()`、`TIM_OC2Init()`
- `TIM_SetCompare1()`、`TIM_SetCompare2()`
- `USART_Init()`、`USART_SendData()`

引脚分配：

- 左编码器：PA0 / PA1，TIM2 编码器模式
- 右编码器：PB6 / PB7，TIM4 编码器模式
- 左电机：ENA=PA6，IN1=PA2，IN2=PA3
- 右电机：ENB=PA7，IN3=PA4，IN4=x'zPA5
- 四路循迹：PB12、PB13、PB14、PB15
- 调试串口：USART1，PA9 / PA10，115200

## 文件说明

| 文件 | 作用 |
| --- | --- |
| `App/Inc/app_config.h` | 方向、PID 初值、目标速度等参数 |
| `App/Src/encoder.c` | 用标准库配置 TIM2/TIM4 编码器并计算速度 |
| `App/Src/motor.c` | 用 TIM3 标准库 PWM 和 GPIO 控制 L298N |
| `App/Src/tracking.c` | 读取 PB12 到 PB15 并计算线的位置误差 |
| `App/Src/pid.c` | 通用 PID |
| `App/Src/robot.c` | 左右轮速度闭环和循迹外环 |
| `App/Src/bsp_time.c` | 10 ms 延时，只用 SysTick 做测试节拍 |
| `App/Src/bsp_uart.c` | USART1 标准库初始化和发送 |
| `Example/user_code.c` | 5 种分步测试模式 |
| `Example/main.c` | 工程的 main 函数，只调用 Setup 和 Loop |

所有 `.c` / `.h` 都加了中文注释：每个结构体成员、每个参数宏、每段配置和计算过程都有说明，遇到不认识的地方可以直接看注释。
文件统一保存为 **UTF-8 带 BOM**，Keil 打开不会出现中文乱码。

## 一、先确认标准库工程
## 推荐阅读顺序

不要从 `main.c` 开始按顺序通读所有文件，会很快卡在寄存器定义里。建议按这个顺序，读一个模块就在板子上验一个模块：

1. **建立地图**：看上面的引脚表，再读 `Example/main.c` 和 `Example/user_code.c` 里的 `Robot_UserSetup()`、`Robot_UserLoop()`。目标是知道“程序只在这两个函数里被调用，主循环 10 ms 跑一次”。
2. **造眼睛**：`bsp_time.c`、`bsp_uart.c`。这两个最简单，而且串口是后面调 PID 唯一的观察窗口。
3. **三个硬件模块**：`encoder.h/.c` -> `motor.h/.c` -> `tracking.h/.c`，顺序和实测顺序一致。每个模块先看 `.h` 的数据结构，再看 `.c` 的 `Init` 和 `Update`。
4. **PID**：`pid.c`。纯数学，不碰硬件。建议拿纸手算一遍 `PID_Update()`。
5. **拼装**：`robot.c`。先 `Robot_Init`（整车接线图），再 `Robot_UpdateSpeedControl`（只有速度环），最后 `Robot_UpdateLineFollowing`（速度环套循迹环）。
6. **回头看**：这时再回 `user_code.c` 看 5 个测试模式，就都看得懂了。

第一遍可以跳过的四块：`encoder.c` 里 16 位计数器的回绕补偿、`pid.c` 里的抗积分饱和、`tracking.c` 里的位镜像运算、`Motor_Brake` 和 `Motor_Stop` 的区别。知道有这回事，以后需要了再回来看。


你的 Keil 工程应该包含这些标准库源文件：

```text
stm32f10x_rcc.c
stm32f10x_gpio.c
stm32f10x_tim.c
stm32f10x_usart.c
misc.c
```

Keil 的 `Options for Target -> C/C++ -> Define` 里要有：

```text
USE_STDPERIPH_DRIVER,STM32F10X_MD
```

`stm32f10x_conf.h` 中至少启用：

```c
#define _GPIO
#define _RCC
#define _TIM
#define _USART
```

如果你的芯片文件选择的是 `STM32F10X_HD`，就保留工程原来的宏，不要强行改成 `MD`。这里写 `MD` 是因为 F103C8T6 属于中容量系列。

标准库老模板通常使用 ARM Compiler 5。如果你用 ARM Compiler 6，旧版 `core_cm3.h` 可能不兼容；这种情况下先在 Keil 中切换为 ARM Compiler 5，或者使用较新的标准库/CMSIS 文件。

## 二、加入代码到 Keil

### 第 1 步：加入源文件

1. 新建一个组，例如 `RobotApp`。
2. 加入 `encoder.c`、`motor.c`、`tracking.c`、`pid.c`、`robot.c`、`bsp_time.c`、`bsp_uart.c`。
3. 再加入 `Example/user_code.c` 和 `Example/main.c`（main.c 放在原来的 User 组里也行）。
4. Include Paths 加入：

```text
App/Inc
Example
```

### 第 2 步：处理工程里原来的 main.c

完整的程序必须有一个 `main()`，而且**整个工程只能有一个**。我给的 `Example/main.c` 内容如下：

```c
#include "stm32f10x.h"
#include "user_code.h"

int main(void)
{
  Robot_UserSetup();   /* 初始化：串口、编码器 TIM2/TIM4、PWM TIM3、循迹 GPIO、PID */

  while (1)
  {
    Robot_UserLoop();  /* 每个 10 ms 周期跑一次测试或控制 */
  }
}
```

时钟配置由启动文件自动调用 `SystemInit()` 完成，不需要在 `main()` 里再写一遍。

标准库模板工程（例如江科大模板）本来就有自己的 `User/main.c`，里面是 OLED、按键、串口的演示代码。两种做法都行：

- 直接把那个 `main.c` 的内容全部删掉，换成上面这一份，文件名保持不变。
- 或者把模板的 `main.c` 从工程里 Remove（不是删文件），再加入我这份 `Example/main.c`。

**千万不要两个 `main.c` 同时参与编译**，否则链接时会报 `Symbol main multiply defined`。

### 第 3 步：模板工程里要避开的文件

标准库模板通常自带 LED、按键、OLED、串口这几个演示模块，它们会和本项目的引脚抢资源：

| 模板文件 | 占用引脚 | 与本项目冲突 |
| --- | --- | --- |
| `Hardware/LED.c` | PA1、PA2 | **冲突**。PA1 是左编码器 B 相，PA2 是左电机 IN1 |
| `Hardware/Serial.c` | USART1、PA9/PA10、9600 波特率 | **冲突**。本项目用同一组引脚，115200 波特率 |
| `Hardware/OLED.c` | PB8、PB9（软件 I2C） | 不冲突，可用 |
| `Hardware/Key.c` | PB1、PB11 | 不冲突，可用 |
| `System/Delay.c` | SysTick | 不冲突，和 `BSP_DelayMs()` 用法相同 |

最省事的做法：**不要调用 `LED_Init()` 和 `Serial_Init()`**，其他模块想留着就留着。只要没调用 `LED_Init()`，PA1/PA2 就不会被改成普通输出。

如果确定用不到，可以把 `LED.c`、`Serial.c`、`OLED.c`、`Key.c` 从工程组里 Remove，让工程干净一点。文件本身不会被删除，以后想用再加回来。

`Robot_UserSetup()` 会自己配置编码器、TIM3 PWM、电机方向引脚、循迹输入和 USART1，不需要 CubeMX 生成的初始化代码，也不需要模板里的 `Serial_Init()`。

## 三、上电前检查

- L298N 的 ENA / ENB 跳线帽必须拔掉。
- 电机电源、L298N 和 STM32 必须共地。
- 第一次测试把车轮架起来。
- 先用限流电源或保险丝更安全。
- 先不要接循迹模块做电机测试，避免接线问题互相干扰。

## 四、分步测试

在 `Example/user_code.c` 中每次只改一个测试模式，然后重新编译下载。

### 1. 编码器测试

```c
#define ROBOT_TEST_MODE 1U
```

把轮子架起来，用手转动左轮和右轮，串口会输出：

```text
ENC Lspd=... Rspd=... Ltotal=... Rtotal=...
```

检查：

- 向前转时两个速度都应是正数。
- 一个编码器数字不动：检查编码器电源、A/B 相、PA0/PA1 或 PB6/PB7。
- 左轮方向相反：把 `ENCODER_LEFT_SIGN` 改为 `-1`。
- 右轮方向相反：把 `ENCODER_RIGHT_SIGN` 改为 `-1`。
- 数字偶尔跳：可以先把 `ENCODER_IC_FILTER` 从 `0x0F` 调低或调高，并让编码器线远离电机线。

### 2. 电机开环测试

```c
#define ROBOT_TEST_MODE 2U
```

轮子保持架空。程序每约 2 秒切换一次正转和反转。

检查：

- 两个轮子都要转。
- 正转时两个轮子都向小车前方转。
- 左轮方向相反：`MOTOR_LEFT_FORWARD_INVERT` 改成 `1U`。
- 右轮方向相反：`MOTOR_RIGHT_FORWARD_INVERT` 改成 `1U`。
- 200 的 PWM 不转，可以先把 `MOTOR_TEST_COMMAND` 改成 300 或 400。
- 一个轮子不转：检查 IN1/IN2、IN3/IN4、ENA/ENB 跳线和 L298N 供电。

### 3. 左右轮速度 PID

```c
#define ROBOT_TEST_MODE 3U
#define WHEEL_TEST_TARGET_COUNTS_S 300.0f
```

输出示例：

```text
PID Ltarget=300 Lspeed=... Lpwm=... Rt=300 Rs=... Rpwm=...
```

这一步开始调 `app_config.h` 中的速度环 PID。先保持：

```c
#define WHEEL_PID_KI 0.00f
#define WHEEL_PID_KD 0.00f
```

然后逐步修改 `WHEEL_PID_KP`。

### 4. 循迹模块测试

```c
#define ROBOT_TEST_MODE 4U
```

把四个探头分别放在黑线和白底上，观察：

```text
TRACK mask=0x... err=... found=...
```

- bit0 到 bit3 分别代表传感器 1 到 4。
- 默认把低电平当成检测到黑线。
- 如果黑白值反了，把 `TRACKING_ACTIVE_LEVEL` 从 `0U` 改成 `1U`。
- 如果左右顺序反了，把 `TRACKING_REVERSE_ORDER` 从 `0U` 改成 `1U`。
- 线偏左时误差为负，线偏右时误差为正。

### 5. 完整循迹

```c
#define ROBOT_TEST_MODE 5U
```

只有前面四步都正确后再测试。默认参数：

```c
#define LINE_BASE_SPEED_COUNTS_S 300.0f
#define LINE_PID_KP              30.0f
#define LINE_PID_KI              0.0f
#define LINE_PID_KD              0.0f
```

如果车修正方向反了，只改：

```c
#define LINE_STEERING_SIGN -1.0f
```

不要把 `Kp` 和方向宏同时乱改。

## 五、PID 调参

### 速度环

速度环的目标和测量单位都是 `counts/s`，输出是 PWM：

```text
-1000 到 +1000
```

建议顺序：

1. `Ki=0`，`Kd=0`。
2. 把 `Kp` 从 `0.10f` 慢慢增加：`0.20`、`0.30`、`0.40`、`0.60`。
3. 实际速度远低于目标，增加 `Kp`。
4. 速度上下振荡、电机声音忽高忽低，减小 `Kp`。
5. `Kp` 稳定后，再以 `0.01f` 为单位增加 `Ki`，消除固定速度误差。
6. 速度环通常先不用 `Kd`。

### 循迹环

速度环调好后再调循迹环：

1. `Ki=0`、`Kd=0`。
2. `Kp` 从 `20.0f` 开始。
3. 弯道跟不上，增加 `Kp`。
4. 直线来回摆，减小 `Kp`，或者加 `Kd=2.0f` 到 `8.0f`。
5. 只有固定小偏差消不掉时，才尝试很小的 `Ki`。
6. 每次只改一个参数，并记录现象。

## 六、常见问题

### 编码器一直为 0

- 编码器没有供电。
- A/B 相接错。
- TIM2/TIM4 时钟或 GPIO 没有初始化。
- 编码器没有和 STM32 共地。

### 电机不转

- ENA/ENB 跳线没有拔掉。
- L298N 电机电源没有接。
- 电机电源和 STM32 没有共地。
- PA6/PA7 没有正确配置成 TIM3 PWM 复用输出。
- PWM 占空比太小，先做开环测试并提高测试命令。

### 车修正方向相反

- 先确认左右轮的正转方向。
- 再确认循迹传感器顺序。
- 最后才改 `LINE_STEERING_SIGN`。

### 车丢线后不停

默认超过 `LINE_LOST_TIMEOUT_MS = 200U` 会停车。如果不停，检查 `Robot_UpdateLineFollowing()` 是否在循环中持续调用，以及循迹模块是否始终输出“检测到线”。
        
### 串口没有输出

- 波特率不是 115200。
- TX / RX 接反：模块的 RX 接 PA9，模块的 TX 接 PA10。
- 模块和 STM32 没有共地。
- 工程里模板自带的 `Serial.c` 也初始化了 USART1，和本工程的 `bsp_uart.c` 混在一起。

### 编译报 Symbol main multiply defined

工程里有两个 `main.c` 同时参与编译，按第二节第 2 步处理。
