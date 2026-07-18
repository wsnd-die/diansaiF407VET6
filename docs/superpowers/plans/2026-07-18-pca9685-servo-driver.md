# PCA9685 五路舵机驱动 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在 I2C2（PB10/PB11）上驱动 PCA9685，以 50 Hz 控制一台 270 度和四台 180 度舵机。

**Architecture:** 新增独立的 PCA9685 驱动模块，负责 I2C 寄存器访问、频率配置与角度到 PWM 计数的换算。`main.c` 仅在 CubeMX 已初始化 I2C2 后调用模块初始化；不会自动驱动任何舵机移动。

**Tech Stack:** STM32F4 HAL I2C、PCA9685 寄存器协议、Keil MDK。

## Global Constraints

- 使用 `hi2c2`，PB10=SCL、PB11=SDA，现有 I2C 速率为 100 kHz。
- PCA9685 7 位地址固定为 `0x40`，HAL 传输地址使用左移一位后的 `0x80`。
- PWM 固定为 50 Hz；脉宽范围 500 至 2500 us，中位 1500 us。
- 通道 0 为 270 度舵机，角度范围 -135 至 +135 度；通道 1 至 4 为 180 度舵机，范围 -90 至 +90 度。
- 所有 `main.c` 改动限制在 `USER CODE` 区域；不改动 CubeMX 生成的 I2C 初始化。

---

### Task 1: PCA9685 驱动与换算单元测试

**Files:**
- Create: `Core/Inc/pca9685.h`
- Create: `Core/Src/pca9685.c`
- Create: `tests/pca9685_math_test.c`

**Interfaces:**
- Consumes: `extern I2C_HandleTypeDef hi2c2`（来自 `Core/Inc/i2c.h`）。
- Produces: `int32_t PCA9685_Init(void)`、`int32_t PCA9685_Set270Angle(float angle_deg)`、`int32_t PCA9685_Set180Angle(uint8_t channel, float angle_deg)`、`static inline uint16_t PCA9685_PulseUsToTicks(uint16_t pulse_us)`。

- [ ] **Step 1: 写入脉宽换算的失败测试**

在 `tests/pca9685_math_test.c` 写入：

```c
#include <assert.h>
#include "pca9685.h"

int main(void)
{
    assert(PCA9685_PulseUsToTicks(0U) == 102U);
    assert(PCA9685_PulseUsToTicks(500U) == 102U);
    assert(PCA9685_PulseUsToTicks(1500U) == 307U);
    assert(PCA9685_PulseUsToTicks(2500U) == 512U);
    assert(PCA9685_PulseUsToTicks(3000U) == 512U);
    return 0;
}
```

- [ ] **Step 2: 运行测试并确认失败**

运行：

```powershell
gcc -ICore/Inc tests/pca9685_math_test.c -o tests/pca9685_math_test.exe
```

预期：失败，提示找不到 `pca9685.h` 或 `PCA9685_PulseUsToTicks`，因为功能尚未实现。

- [ ] **Step 3: 实现最小换算与 PCA9685 驱动**

在 `Core/Inc/pca9685.h` 声明：

```c
#ifndef __PCA9685_H
#define __PCA9685_H

#include <stdint.h>

#define PCA9685_MIN_PULSE_US     500U
#define PCA9685_MAX_PULSE_US     2500U
#define PCA9685_PERIOD_US        20000U

static inline uint16_t PCA9685_PulseUsToTicks(uint16_t pulse_us)
{
    if (pulse_us < PCA9685_MIN_PULSE_US) pulse_us = PCA9685_MIN_PULSE_US;
    if (pulse_us > PCA9685_MAX_PULSE_US) pulse_us = PCA9685_MAX_PULSE_US;
    return (uint16_t)(((uint32_t)pulse_us * 4096U + (PCA9685_PERIOD_US / 2U)) /
                      PCA9685_PERIOD_US);
}

int32_t PCA9685_Init(void);
int32_t PCA9685_Set270Angle(float angle_deg);
int32_t PCA9685_Set180Angle(uint8_t channel, float angle_deg);

#endif
```

在 `Core/Src/pca9685.c` 写入：

```c
#include "pca9685.h"
#include "i2c.h"

#define PCA9685_ADDRESS          (0x40U << 1)
#define PCA9685_MODE1            0x00U
#define PCA9685_PRESCALE         0xFEU
#define PCA9685_LED0_ON_L        0x06U
#define PCA9685_PRESCALE_50HZ    121U

static int32_t PCA9685_Write(uint8_t reg, uint8_t *data, uint16_t len)
{
    if (HAL_I2C_Mem_Write(&hi2c2, PCA9685_ADDRESS, reg,
                          I2C_MEMADD_SIZE_8BIT, data, len,
                          HAL_MAX_DELAY) != HAL_OK)
    {
        return -1;
    }
    return 0;
}

static int32_t PCA9685_SetTicks(uint8_t channel, uint16_t off_ticks)
{
    uint8_t data[4];

    if (channel > 15U)
    {
        return -1;
    }

    data[0] = 0U;
    data[1] = 0U;
    data[2] = (uint8_t)(off_ticks & 0xFFU);
    data[3] = (uint8_t)(off_ticks >> 8);
    return PCA9685_Write((uint8_t)(PCA9685_LED0_ON_L + 4U * channel), data, sizeof(data));
}

static uint16_t PCA9685_AngleToPulseUs(float angle_deg, float max_angle_deg)
{
    float pulse_us;

    if (angle_deg < -max_angle_deg) angle_deg = -max_angle_deg;
    if (angle_deg > max_angle_deg) angle_deg = max_angle_deg;
    pulse_us = 1500.0f + angle_deg * 1000.0f / max_angle_deg;
    return (uint16_t)(pulse_us + 0.5f);
}

int32_t PCA9685_Init(void)
{
    uint8_t value;

    value = 0x10U;
    if (PCA9685_Write(PCA9685_MODE1, &value, 1U) != 0) return -1;
    value = PCA9685_PRESCALE_50HZ;
    if (PCA9685_Write(PCA9685_PRESCALE, &value, 1U) != 0) return -1;
    value = 0x00U;
    if (PCA9685_Write(PCA9685_MODE1, &value, 1U) != 0) return -1;
    HAL_Delay(1U);
    value = 0xA1U;
    return PCA9685_Write(PCA9685_MODE1, &value, 1U);
}

int32_t PCA9685_Set270Angle(float angle_deg)
{
    return PCA9685_SetTicks(0U, PCA9685_PulseUsToTicks(
        PCA9685_AngleToPulseUs(angle_deg, 135.0f)));
}

int32_t PCA9685_Set180Angle(uint8_t channel, float angle_deg)
{
    if (channel < 1U || channel > 4U)
    {
        return -1;
    }
    return PCA9685_SetTicks(channel, PCA9685_PulseUsToTicks(
        PCA9685_AngleToPulseUs(angle_deg, 90.0f)));
}
```

`pca9685.c` 包含 `i2c.h` 并将 HAL 传输状态转换为 `0`（成功）或 `-1`（失败）。`PCA9685_Init()` 按顺序写入 `MODE1=0x10`、`PRESCALE=121`、`MODE1=0x00`、延时 1 ms、`MODE1=0xA1`，得到 50 Hz 自动递增输出模式。角度接口先限幅，再线性换算到 500 至 2500 us；`PCA9685_Set180Angle()` 仅接受通道 1 至 4，其他通道返回 `-1`。写 PWM 时向 `LED0_ON_L + 4 * channel` 连续写 `0, 0, off_ticks低字节, off_ticks高字节`。

- [ ] **Step 4: 运行换算测试并确认通过**

运行：

```powershell
gcc -ICore/Inc tests/pca9685_math_test.c -o tests/pca9685_math_test.exe
./tests/pca9685_math_test.exe
```

预期：退出码为 0；500、1500、2500 us 分别换算为 102、307、512 计数。

- [ ] **Step 5: 提交该独立驱动任务**

```powershell
git add Core/Inc/pca9685.h Core/Src/pca9685.c tests/pca9685_math_test.c
git commit -m "feat: add PCA9685 servo driver"
```

### Task 2: 工程集成与构建验证

**Files:**
- Modify: `Core/Src/main.c` 的 `/* USER CODE BEGIN Includes */` 与 `/* USER CODE BEGIN 2 */`
- Modify: `MDK-ARM/vet6_mdk.uvprojx` 的 Application/User 源文件组

**Interfaces:**
- Consumes: `PCA9685_Init(void)`。
- Produces: 上电后 PCA9685 已配置为 50 Hz；应用层可在任务代码中调用两个角度控制接口。

- [ ] **Step 1: 写入集成失败检查**

在 `Core/Src/main.c` 的用户包含区暂时加入：

```c
#include "pca9685.h"
```

在初始化用户区、`MX_I2C2_Init();` 之后暂时加入：

```c
if (PCA9685_Init() != 0)
{
  Error_Handler();
}
```

保持 `pca9685.c` 未加入 `MDK-ARM/vet6_mdk.uvprojx`，执行 Keil Rebuild。

预期：链接失败并显示 `Undefined symbol PCA9685_Init (referred from main.o)`，证明集成检查能够捕获遗漏工程源文件。

- [ ] **Step 2: 将驱动文件加入 Keil 项目并重新构建**

在 `MDK-ARM/vet6_mdk.uvprojx` 的 Application/User `<Files>` 内加入：

```xml
<File>
  <FileName>pca9685.c</FileName>
  <FileType>1</FileType>
  <FilePath>../Core/Src/pca9685.c</FilePath>
</File>
```

- [ ] **Step 3: 构建并确认工程通过**

在 Keil 中执行 Rebuild all target files。

预期：构建日志出现 `compiling pca9685.c...`，且没有 `Undefined symbol PCA9685_Init` 或编译警告。

- [ ] **Step 4: 进行低风险硬件验证**

烧录后，先只调用以下小角度命令：

```c
PCA9685_Set270Angle(0.0f);
PCA9685_Set180Angle(1U, 0.0f);
PCA9685_Set180Angle(2U, 10.0f);
PCA9685_Set180Angle(3U, -10.0f);
PCA9685_Set180Angle(4U, 0.0f);
```

预期：通道 0 至 4 分别独立响应，未出现复位、发热、抖动或机械撞限位。若方向与机构相反，仅在调用处传递相反角度，保持驱动换算不变。

- [ ] **Step 5: 提交集成任务**

```powershell
git add Core/Src/main.c MDK-ARM/vet6_mdk.uvprojx
git commit -m "feat: initialize PCA9685 on I2C2"
```
