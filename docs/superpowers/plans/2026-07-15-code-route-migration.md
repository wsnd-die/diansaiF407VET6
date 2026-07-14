# A/C 自动路线迁移 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 将 `code` 工程的 A/C 自动路线、毫米导航和双轮里程计安全移植到 STM32F407VET6。

**Architecture:** USART6 只接收 `A/a`、`T/t` 并交给应用状态机。应用任务按路线逐点请求导航；导航任务每 10 ms 用 UART4 HWT101 和 USART2 双轮编码器更新坐标、向 USART2 的四个电机输出差速速度，C 完成后停车回空闲。

**Tech Stack:** STM32F4 HAL、CMSIS-RTOS2、FreeRTOS、Keil MDK ARMCC 5.06、PowerShell、MinGW GCC。

## Global Constraints

- USART1 仅 TOF；USART2 仅电机和地址 3/4 里程计回包；USART3 仅语音；UART4 仅 HWT101；UART5 无业务；USART6 仅 A/T。
- 坐标单位 mm；HWT101 yaw 单位度；路线目标 yaw 单位 rad；底盘地址固定为左 3/2、右 4/1。
- 仅修改 CubeMX 文件的 `USER CODE` 区域；不覆盖或暂存用户已有未提交改动。
- `StartTask02()` 不发送任何上电位置运动命令。
- 不迁移蓝牙行通信、B 路线、USART1 调试输出或无关升降/舵机代码。

---

## 文件结构

| 文件 | 责任 |
| --- | --- |
| `Core/Inc/app.h`、`Core/Src/app.c` | A/C 路线、A/T 命令和 C 完成后的停车。 |
| `Core/Inc/navigation.h`、`Core/Src/navigation.c` | 毫米导航状态机和四轮差速输出。 |
| `Core/Inc/odometer.h`、`Core/Src/odometer.c` | 地址 3/4 回包、成对位移和坐标更新。 |
| `Core/Src/bujin.c` | 42.5 mm 底盘轮半径。 |
| `Core/Src/main.c`、`freertos.c`、`stm32f4xx_it.c` | 初始化、任务调度和 USART6 分发。 |
| `MDK-ARM/vet6_mdk.uvprojx` | 加入 `app.c`，移除已删除的 `bluetooth.c`。 |
| `tests/host/*` | 主机侧路线契约测试。 |

### Task 1: 建立路线状态机的失败测试

**Files:**
- Create: `tests/host/include/main.h`
- Create: `tests/host/include/FreeRTOS.h`
- Create: `tests/host/include/task.h`
- Create: `tests/host/test_app_route.c`
- Create: `tests/host/run_app_route_test.ps1`
- Test: `tests/host/test_app_route.exe`

**Interfaces:**
- Consumes: Task 2 的 `App_Init()`、`App_CommandUartRxByte()`、`App_RunCurrentMode()`。
- Produces: A 5 点、A 转 C、C 12 点后停车空闲、T 停车的可执行契约。

- [ ] **Step 1: 写最小 HAL/FreeRTOS 桩和测试**

`main.h`：

```c
#ifndef TEST_MAIN_H
#define TEST_MAIN_H
#include <stdbool.h>
#include <stdint.h>
#endif
```

`FreeRTOS.h`：

```c
#include <stdint.h>
typedef uint32_t TickType_t;
#define pdMS_TO_TICKS(ms) ((TickType_t)(ms))
```

`task.h`：

```c
#include "FreeRTOS.h"
void vTaskDelay(TickType_t ticks);
```

`test_app_route.c` 必须定义 `Navigation_Request`、`Navigation_IsIdle`、`Navigation_Stop`、`vTaskDelay` 和 `Emm_V5_Vel_Control` 桩；每次 `Navigation_Request` 保存 `x/y/yaw` 并置忙，`vTaskDelay` 置空闲。断言：首次 `A` 后请求数为 5 且模式为 C；第二次运行后请求数为 17、模式为空闲、停车次数为 1；`A` 后立即 `T` 再运行时停车次数为 1。

`run_app_route_test.ps1`：

```powershell
$ErrorActionPreference = 'Stop'
gcc -std=c11 -Wall -Wextra -Werror -I tests/host/include -I Core/Inc tests/host/test_app_route.c Core/Src/app.c -lm -o tests/host/test_app_route.exe
& tests/host/test_app_route.exe
```

- [ ] **Step 2: 运行测试并确认它正确失败**

Run: `powershell -ExecutionPolicy Bypass -File tests/host/run_app_route_test.ps1`

Expected: FAIL，提示 `Core/Src/app.c` 或 `app.h` 不存在。

- [ ] **Step 3: 提交失败测试**

```powershell
git add -- tests/host
git commit -m "test: define A/C route completion contract"
```

### Task 2: 迁移应用、导航和双轮里程计

**Files:**
- Create: `Core/Inc/app.h`、`Core/Src/app.c`
- Modify: `Core/Inc/navigation.h`、`Core/Src/navigation.c`
- Modify: `Core/Inc/odometer.h`、`Core/Src/odometer.c`
- Modify: `Core/Src/bujin.c:10`
- Test: `tests/host/test_app_route.exe`

**Interfaces:**
- Consumes: `HWT101_GetYaw()`、全部 `Emm_V5_*` UART2 驱动接口和 `usart2TXHandle`。
- Produces: `App_*`、`Navigation_TaskTick()`、`Navigation_Request()`、`Navigation_IsIdle()`、`Navigation_Stop()`、`g_robot_pos` 和 `Odometer_UartRxByte()`。

- [ ] **Step 1: 从新版源工程迁移业务基线**

用 `D:\codexproject\智慧农业赛道\厂里大运\code\Core` 中当前的 `app.c/.h`、`navigation.c/.h`、`odometer.c/.h` 作为唯一业务来源，通过 `apply_patch` 写入目标文件。不要复制源 `bujin.c`，目标版的 UART2 互斥锁名称 `usart2TXHandle` 已正确。

- [ ] **Step 2: 按目标协议收紧 app 模块**

`app.h` 的公共接口固定为：

```c
typedef enum { APP_MODE_IDLE, APP_MODE_ROUTE_A, APP_MODE_ROUTE_B, APP_MODE_ROUTE_C } AppMode_t;
typedef struct { float x_mm; float y_mm; float yaw_rad; } AppWaypoint_t;
extern volatile AppMode_t g_app_mode;
void App_Init(void);
void App_RunCurrentMode(void);
void App_SetMode(AppMode_t mode);
bool App_IsRunning(void);
void App_CommandUartRxByte(uint8_t data);
```

保留源工程的 A 5 点和 C 12 点数组；`App_CommandUartRxByte()` 只处理 A/T：

```c
if (data == 'a' || data == 'A') {
    s_app_running = true; s_stop_requested = false; App_SetMode(APP_MODE_ROUTE_A);
} else if (data == 't' || data == 'T') {
    s_app_running = false; s_stop_requested = true; App_SetMode(APP_MODE_IDLE);
}
```

`App_RunCurrentMode()` 先消费停止请求并在任务上下文调用 `Navigation_Stop()`。C 路线调用 `App_RunRoute(k_route_c, ..., APP_MODE_IDLE)`；`App_RunRoute()` 完成且 `next_mode == APP_MODE_IDLE` 时执行：

```c
s_app_running = false;
Navigation_Stop();
App_SetMode(APP_MODE_IDLE);
```

- [ ] **Step 3: 迁移导航/里程计并清除 USART1 调试依赖**

迁移源导航的四状态处理、毫米位置、`Navigation_TaskTick()` 和 `Chassis_SetSpeed()`。删除源导航的 `#include "usart.h"`、`#include <stdio.h>` 和 `Navigation_DebugPrint()`，使导航文件没有 `huart1` 或 `HAL_UART_Transmit`。

迁移源里程计的地址 3/4、50 ms 交替 `S_CPOS` 轮询和以下成对位移公式：

```c
motor_half_delta_mm = (odom_left.delta_mm - odom_right.delta_mm) * 0.5f;
Navigation_UpdateByDelta(motor_half_delta_mm, g_robot_pos.yaw);
```

将 `Core/Src/bujin.c` 改为：

```c
#define BUJIN_WHEEL_RADIUS_MM     42.5f
```

- [ ] **Step 4: 验证转绿并提交核心**

Run: `powershell -ExecutionPolicy Bypass -File tests/host/run_app_route_test.ps1`

Expected: exit code 0，全部 A/C/T 断言通过。

```powershell
git add -- Core/Inc/app.h Core/Src/app.c Core/Inc/navigation.h Core/Src/navigation.c Core/Inc/odometer.h Core/Src/odometer.c Core/Src/bujin.c tests/host
git commit -m "feat: migrate A/C route navigation core"
```

### Task 3: 接入 F407 初始化、任务和 USART6

**Files:**
- Modify: `Core/Src/main.c:28-42,117-137`
- Modify: `Core/Src/freertos.c:27-40,218-324`
- Modify: `Core/Src/stm32f4xx_it.c:24-29,320-336`
- Modify: `MDK-ARM/vet6_mdk.uvprojx:393-452`

**Interfaces:**
- Consumes: Task 2 的 app、导航和里程计接口。
- Produces: USART6 唯一命令入口、10 ms 导航任务和 50 ms 应用任务。

- [ ] **Step 1: 修正初始化职责**

在 `main.c` 的用户包含区删除 `bluetooth.h`，加入 `app.h`。删除 `Bluetooth_Init()`，启用 TOF 接收：

```c
HWT101_HAL_Init();
TOF200F_Init();
/* 保留现有 HWT101 最多等待 1000 ms 的就绪循环。 */
Navigation_Reset(NAV_START_CENTER_X_MM, NAV_START_CENTER_Y_MM, HWT101_GetYaw());
Odometer_Init();
```

- [ ] **Step 2: 复用任务槽且取消上电运动**

在 `freertos.c` 包含 `app.h`。删除 `StartTask02()` 循环前的四条 `Emm_V5_Pos_Control`、同步触发和 1000 ms 延时，仅保留 `for (;;) { osDelay(100); }`。

将 `StartTask03()` 改为：

```c
for (;;) {
    g_robot_pos.yaw = Navigation_NormalizeDeg(HWT101_GetYaw() - nav_yaw_zero_deg);
    Navigation_TaskTick();
    Odometer_Update();
    osDelay(10);
}
```

将 `StartTask06()` 改为：

```c
App_Init();
for (;;) {
    App_RunCurrentMode();
    osDelay(50);
}
```

- [ ] **Step 3: 将 USART6 改为 A/T 专用**

在 `stm32f4xx_it.c` 删除 `bluetooth.h`，加入 `app.h`。保留 ORE 清除和读 DR，只替换 RXNE 分发：

```c
App_CommandUartRxByte(data);
```

不改 USART1 的 `TOF200F_UartRxByte(data)`、USART2 的 `Odometer_UartRxByte(data)` 和 UART4 的 `HWT101_ParsePacket(rx_buf)`。

- [ ] **Step 4: 修正 Keil 成员**

在与 `navigation.c` 同一 `<Files>` 组加入：

```xml
<File>
  <FileName>app.c</FileName>
  <FileType>1</FileType>
  <FilePath>../Core/Src/app.c</FilePath>
</File>
```

删除 `bluetooth.c` 的完整 `<File>...</File>` 块。

- [ ] **Step 5: 再运行测试、检查串口并提交**

```powershell
powershell -ExecutionPolicy Bypass -File tests/host/run_app_route_test.ps1
rg -n 'App_CommandUartRxByte|Bluetooth_UartRxByte|TOF200F_UartRxByte|Odometer_UartRxByte|HWT101_ParsePacket' Core/Src/main.c Core/Src/stm32f4xx_it.c
git add -- Core/Src/main.c Core/Src/freertos.c Core/Src/stm32f4xx_it.c MDK-ARM/vet6_mdk.uvprojx tests/host
git commit -m "feat: wire A/C route to F407 UART tasks"
```

Expected: 测试成功；app 命令只从 USART6 进入；无 `Bluetooth_UartRxByte`；提交不含用户原有改动。

### Task 4: 编译与交付验证

**Files:**
- Verify: `MDK-ARM/vet6_mdk.uvprojx`、`MDK-ARM/build.log`、`tests/host/test_app_route.exe`

**Interfaces:**
- Consumes: Task 1-3 的测试、工程成员和迁移模块。
- Produces: 主机测试、串口分配和 Keil 零错误构建证据。

- [ ] **Step 1: 运行主机测试**

Run: `powershell -ExecutionPolicy Bypass -File tests/host/run_app_route_test.ps1`

Expected: exit code 0。

- [ ] **Step 2: 用 Keil 重建**

```powershell
$uv4 = @('C:\Keil_v5\UV4\UV4.exe','C:\Keil\UV4\UV4.exe','D:\Keil_v5\UV4\UV4.exe','D:\Keil\UV4\UV4.exe') | Where-Object { Test-Path -LiteralPath $_ } | Select-Object -First 1
if ($null -eq $uv4) { throw '未找到 UV4.exe，请在 Keil IDE 中对 MDK-ARM\vet6_mdk.uvprojx 执行 Rebuild。' }
& $uv4 -b MDK-ARM\vet6_mdk.uvprojx -j0
if ($LASTEXITCODE -ne 0) { throw "Keil build failed: $LASTEXITCODE" }
```

Expected: `build.log` 末尾为 `"vet6_mdk\\vet6_mdk.axf" - 0 Error(s), 0 Warning(s).`，并出现 `compiling app.c...`、`navigation.c`、`odometer.c`、`freertos.c`、`stm32f4xx_it.c`。

- [ ] **Step 3: 检查补丁范围**

```powershell
git diff --check -- Core/Inc/app.h Core/Src/app.c Core/Inc/navigation.h Core/Src/navigation.c Core/Inc/odometer.h Core/Src/odometer.c Core/Src/bujin.c Core/Src/main.c Core/Src/freertos.c Core/Src/stm32f4xx_it.c MDK-ARM/vet6_mdk.uvprojx tests/host
git status --short
```

Expected: `git diff --check` 无输出；没有本计划文件的未提交改动。
