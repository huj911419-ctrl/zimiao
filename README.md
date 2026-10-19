# zimiao · STM32H723 GM6020 双轴云台与自瞄接入

## 项目详解

本仓库是 **DM-MC02 / STM32H723 的电控固件**，以 GM6020 双轴云台为主，接入 BMI088 姿态、DT7 遥控、USB CBoard 协议和上位机绝对目标角。目标识别与视觉解算位于外部视觉工程，不在本仓库内。

详细接入说明见 [自瞄跟随](application/auto_aim/README.md)，已有实机记录与未完成事项见 [视觉联调状态](docs/VISION_INTEGRATION_STATUS.md)。`fashejigou` 中的 J4310 单轴速度台架属于另一项目，不能替代本工程。

### 已实现与已验证分别是什么

- **源码已有**：双轴控制、姿态/状态遥测、USB 目标帧解析、四元数坐标转换、目标时效、遥控模式、限位与故障约束。
- **仓库记录的实机验证**：2026-10-18 的观察版完成 STM32 → Ubuntu 姿态/状态收发与手动四方向坐标核对。
- **尚未完成的实机验证**：真实视觉目标 → MCU → 电机的闭环跟随，相机内外参及机械零位仍需确认。
- **发射机构未接入**：收到 `shoot=1` 只记录，不应触发发射。15 m/s 联调弹速是配置假设值，不是测速结果。

上述实机记录对应当次观察版，不能用于推定设备此刻烧录的固件或参数。

### 核心数据流与文件

```text
BMI088 → INS 一致快照 → 四元数变换 → USB 姿态/状态上报
USB 目标帧 → 字节流解析/邮箱 → 时效与控制条件 → 云台目标
DT7 模式 ────────────────────→ 授权与状态切换
云台角度外环 → 速度 PI → GM6020 输出、限位与故障处理
```

| 路径 | 职责 |
| --- | --- |
| [Core/Src/main.c](Core/Src/main.c) | 外设、机器人初始化、标定后启动看门狗与调度器 |
| [application/auto_aim/](application/auto_aim/) | 目标接收、会话、坐标变换和遥测 |
| [application/gimbal/gimbal_control.c](application/gimbal/gimbal_control.c) | 双轴控制与保护核心 |
| [application/gimbal/gimbal_runtime.c](application/gimbal/gimbal_runtime.c) | 控制任务与传输/状态的运行装配 |
| [application/cmd/dt7_cmd.c](application/cmd/dt7_cmd.c) | 遥控模式与输入映射 |
| [module/imu/ins_task.c](module/imu/ins_task.c) | 姿态采样与快照 |
| [module/cboard/cboard_protocol.c](module/cboard/cboard_protocol.c) | CBoard 字节协议 |
| [bsp/usb/vision_usb_port.c](bsp/usb/vision_usb_port.c) | USB 连接、接收队列与发送适配 |
| [tests/run_tests.py](tests/run_tests.py) | 主机编译与回归入口 |

### 通信约定

| 方向 | 帧 | 字节数 | 主要字段 |
| --- | --- | ---: | --- |
| 电控 → 上位机 | AA | 10 | `AA + w,x,y,z(int16×4) + DD` |
| 电控 → 上位机 | BB | 9 | `BB + speed,mode,shoot_mode,ft_angle(int16×4)`，无帧尾 |
| 上位机 → 电控 | CC | 8 | `CC + control(u8) + shoot(u8) + yaw(i16) + pitch(i16) + EE` |

多字节字段小端；姿态分量与目标角乘以 10000，目标角单位 rad；速度乘以 100。目标为绝对角度，不是摇杆式角速度积分。该协议没有 CRC，合法性检查不能发现全部负载位翻转。

最后有效视觉目标超过 150 ms、USB 断开或溢出时撤销目标；`control=0` 不代表云台回零。无效帧不能刷新目标有效期，旧模式/旧会话目标不能在下一次自瞄中复用。具体保持、停机和故障行为以当前控制状态及专项说明为准。

### 先选择构建模式

需要 GNU Arm Embedded 工具链、Make 和 Python 3。普通 `make` 的 `GIMBAL_MOTOR_OUTPUT_ENABLED` 默认值是 **1**；`AUTO_AIM_COORDINATES_CONFIRMED=0` 只限制视觉接管，不等于禁止手动电机输出。

初次做姿态/通信观察时，显式构建禁用电机输出的版本，并使用独立输出目录：

```bash
git clone https://github.com/huj911419-ctrl/zimiao.git
cd zimiao
make -j8 ROBOT=sentry BUILD_DIR=build/vision-observe \
  AUTO_AIM_USB_MODE=1 AUTO_AIM_COORDINATES_CONFIRMED=0 \
  GIMBAL_MOTOR_OUTPUT_ENABLED=0
```

产物前缀为 `build/vision-observe/Meta-Embedded-NG-sentry`，包含 `.elf/.hex/.bin`。工具链不在 PATH 时加 `GCC_PATH=/实际工具链/bin`。命令只编译，不连接或烧录开发板。

| 构建参数 | 用途 |
| --- | --- |
| `ROBOT=sentry/infantry` | 选择机器人入口；哨兵额外包含被动 CAN 扫描 |
| `AUTO_AIM_USB_MODE=1` | CBoard 二进制 USB；0 为原调试模式 |
| `AUTO_AIM_COORDINATES_CONFIRMED=0` | 安装坐标和标定尚未确认时禁止视觉接管 |
| `GIMBAL_MOTOR_OUTPUT_ENABLED=0` | 观察版禁止电机输出；普通默认构建并非此配置 |

不要仅为获得 `mode=1` 而开启坐标确认。完整标定和实际控制验收完成后，才有依据调整对应配置。

### 遥控与配置入口

CBoard 模式下，右拨杆下档为 OFF、中档请求 AUTO、上档为手动；左拨杆下档选择双轴。AUTO 还受到坐标确认、阶段、反馈、USB、遥控和联调参数等条件限制，拨杆位置本身不保证进入自动跟随。

- [auto_aim_config.h](application/auto_aim/auto_aim_config.h)：协议模式、世界/机体/Solver 变换、超时与联调弹速。
- [gimbal_manual_config.h](application/gimbal/gimbal_manual_config.h)：手动云台参数、限制和输出开关。
- [联调状态](docs/VISION_INTEGRATION_STATUS.md)：坐标方向、安装标定和真实闭环的证据边界。

### 主机验证与阅读顺序

```bash
python3 tests/run_tests.py
```

主机回归使用本机 GCC 和替身接口，覆盖协议、坐标、USB、模式、过期目标及控制保护；它不能代替真实相机、电机和动态跟随验收。

建议先看联调状态与自瞄 README，再看配置、`auto_aim.*`、`gimbal_control.*`、运行装配和相应测试。MIT 许可证及上游框架致谢保留在 [LICENSE](LICENSE) 和下文。

---

以下为原有机器人框架中英文介绍。首次构建模式与自瞄阶段请先按上面的项目说明选择。

> 面向达妙 **DM-MC02（STM32H723VGT6）** 控制板的 RoboMaster 机器人电控固件。
> RoboMaster robot control firmware for the DaMiao **DM-MC02 (STM32H723VGT6)** board.

[![MCU](https://img.shields.io/badge/MCU-STM32H723VGT6-blue)](#)
[![Board](https://img.shields.io/badge/Board-DM--MC02-orange)](#)
[![RTOS](https://img.shields.io/badge/RTOS-FreeRTOS-green)](#)
[![License](https://img.shields.io/badge/License-MIT-lightgrey)](#license--致谢--license--acknowledgements)

**[简体中文](#简体中文) | [English](#english)**

---

## 简体中文

### 项目简介

**Meta-Embedded-NG** 是一套用于 RoboMaster 比赛机器人的嵌入式电控固件。它在湖南大学跃鹿战队开源框架 [`basic_framework`](https://github.com/HNUYueLuRM/basic_framework) 的基础上做了大量修改，将原本面向 STM32F407（大疆 C 板）的代码移植并适配到了基于 **STM32H723** 的 **达妙 DM-MC02** 控制板。

项目沿用了原框架优秀的设计思想：

- **三层分层架构** —— `bsp`（板级支持）→ `module`（可复用算法与设备驱动）→ `application`（机器人行为）。
- **消息中心（Message Center）发布-订阅** —— 各应用之间不直接相互包含，而是通过话题（topic）解耦通信。
- **离线守护（Daemon）** —— 统一监控各模块心跳，掉线时触发回调以进入安全状态。

### 硬件平台

| 项目 | 说明 |
| --- | --- |
| 主控 MCU | STM32H723VGT6（Cortex-M7，最高 550 MHz，1 MB Flash） |
| 控制板 | 达妙 DM-MC02 |
| 板载 IMU | BMI088（SPI 接口，含温度补偿与在线标定） |
| 链接脚本 | `STM32H723XG_FLASH.ld` |
| RTOS | FreeRTOS |

### 与 basic_framework 的差异

本项目是对 `basic_framework` 的**深度移植与裁剪**，主要区别如下：

| 方面 | basic_framework | Meta-Embedded-NG |
| --- | --- | --- |
| 主控 / 控制板 | STM32F407（大疆 C 板） | STM32H723（DM-MC02） |
| 模块目录名 | `modules/` | `module/` |
| 电机驱动 | DJI / DM / HT / LK / 舵机 / 步进 | DJI / DM / 舵机 / 步进 / **XM（新增）**，移除 HT、LK |
| 传感器/外设模块 | 含 IST8310 磁力计、OLED、蓝牙、TFminiPlus 激光雷达 | 已移除上述模块 |
| 裁判系统 | 原版 | 修复了裁判系统协议与任务 |

核心三层架构、消息中心设计与代码规范保持不变。

### 目录结构

```
Meta-Embedded-NG/
├── application/      # 机器人行为：底盘、云台、发射、指令分发、各机器人（哨兵/步兵）
├── module/           # 可复用算法与设备驱动（电机、IMU、消息中心等）
├── bsp/              # 板级支持包，封装 STM32H7 片上外设
├── Core/             # CubeMX 生成的外设/FreeRTOS/中断代码（Inc + Src）
├── Drivers/          # STM32H7xx HAL 与 CMSIS（厂商代码）
├── Middlewares/      # FreeRTOS、USB 设备库、CMSIS-DSP、SEGGER RTT
├── USB_DEVICE/       # USB 设备中间件（App + Target）
├── Makefile          # arm-none-eabi-gcc 构建脚本
├── Meta-Embedded-NG.ioc   # CubeMX 工程文件
└── STM32H723XG_FLASH.ld   # 链接脚本
```

### 软件架构

- **application/**：`chassis`（底盘运动学/功率限制）、`gimbal`（云台姿态控制）、`shoot`（摩擦轮 + 拨弹）、`cmd`（指令分发，消费遥控/键鼠输入并发布控制量），以及各机器人目录 `sentry`（哨兵）、`infantry`（步兵），每个机器人目录含统一入口 `robot.c`/`robot_task.c` 与专属参数 `<robot>_def.h`。机器人类型在构建时经 `make ROBOT=<robot>` 选择，通用消息契约与选择器见 `application/robot_def.h`。
- **module/**：电机驱动（`motor/`：DJI、DM、XM、舵机、步进）、`algorithm`（PID、卡尔曼滤波、LQR、EKF 姿态解算、CRC）、`message_center`、`daemon`、`BMI088`、`imu`、`encoder`、`can_comm`（多板 CAN 通信）、`super_cap`（超级电容）、`referee`（裁判系统/UI）、`remote`（DT7 遥控）、`master_machine`、`standard_cmd`、`unicomm`、`alarm`。
- **bsp/**：`can`、`usart`、`spi`、`iic`、`pwm`、`adc`、`gpio`、`log`、`usb`、`flash`、`dwt`。
- **FreeRTOS 任务**：INS 姿态解算（约 1 kHz）、电机控制、系统监控、机器人主逻辑等并行运行。

各子模块旁通常附带 `*.md` 说明文档（如 `module/message_center/message_center.md`、`module/motor/DJImotor/dji_motor.md`、`module/BMI088/bmi088.md`），开发前建议先阅读。

### 环境与构建

依赖 **GNU Arm Embedded 工具链**（`arm-none-eabi-gcc`）。

> 当前 Makefile 会自动创建构建输出目录。也可预先执行 `mkdir -p build`，重复执行不会报错。

```bash
# 可选：预先创建构建目录
mkdir -p build

# 按机器人种类构建（ROBOT 默认为 sentry）
# -j 启用并行编译；可用 -j<N> 限制并发数，如 -j8
make -j ROBOT=sentry      # 生成 build/sentry/Meta-Embedded-NG-sentry.elf / .hex / .bin / .map
make -j ROBOT=infantry    # 生成 build/infantry/Meta-Embedded-NG-infantry.elf / .hex / .bin / .map
make -j                   # 等价于 make -j ROBOT=sentry

# 清理当前机器人的 build/<robot>/ 目录
make clean

# 指定工具链路径而不修改 PATH
make -j ROBOT=infantry GCC_PATH=/path/to/gcc-arm-none-eabi/bin
```

> 每种机器人各自编译到独立的 `build/<robot>/` 子目录，切换机器人无需先 `make clean`。

> 仓库未提供烧录目标（flash target）。请使用团队的调试器/烧录器（如 OpenOCD + CMSIS-DAP、Ozone 等）将生成的 `.hex`/`.bin` 烧录到板子。

### 配置说明

- **机器人类型在构建时选择**：`make ROBOT=sentry|infantry` 会传入 `-DROBOT_SENTRY` / `-DROBOT_INFANTRY`，并编译对应的 `application/<robot>/` 目录。
- 各模块间通用的消息契约（底盘/云台/发射的枚举与结构体）与机器人选择器统一放在 `application/robot_def.h`；各机器人专属的物理参数（轮距、云台限位、拨盘参数等）放在 `application/<robot>/<robot>_def.h`。
- 新增一种机器人：把 `application/sentry/` 复制为 `application/<new>/`，在 `robot_def.h` 增加一条 `#elif defined(ROBOT_<NEW>)`，并在 Makefile 的机器人变体块中增加一条 `else ifeq ($(ROBOT),<new>)`。
- BMI088 支持上电在线标定，标定相关参数见 `module/BMI088/bmi088.md`。

### 代码规范与贡献

详见 [`AGENTS.md`](AGENTS.md)。要点：

- 遵循 Conventional Commits：`feat:`、`fix:`、`chore:`、`doc:`、`feat!:`（破坏性变更）。
- C 代码 4 空格缩进；宏 `UPPER_SNAKE_CASE`，配置结构体以 `_s` 结尾，typedef 以 `_t` 结尾，初始化/控制 API 形如 `ChassisInit()`、`DJIMotorControl()`。
- CAN/串口等协议结构体使用 `#pragma pack(1)`。
- 每次改动至少保证 `make` 通过；行为变更请记录硬件验证情况。
- 不要手动修改 CubeMX 生成块，除非是有意且记录在案的改动。

### License & 致谢

本项目以 **MIT License** 发布，Copyright (c) 2025 Meta-Team，详见 [`LICENSE`](LICENSE)。

衷心感谢 **湖南大学 RoboMaster 跃鹿战队** 开源的 [`basic_framework`](https://github.com/HNUYueLuRM/basic_framework)（MIT，Copyright (c) 2022 NeoZng / Even），本项目在其基础上开发。

---

## English

### Overview

**Meta-Embedded-NG** is embedded control firmware for RoboMaster competition robots. It is heavily modified from the Hunan University YueLu Team's open-source [`basic_framework`](https://github.com/HNUYueLuRM/basic_framework), porting and adapting code originally targeting the STM32F407 (DJI C-board) to the **STM32H723**-based **DaMiao DM-MC02** control board.

The project keeps the original framework's core design ideas:

- **Three-layer architecture** — `bsp` (board support) → `module` (reusable algorithms & device drivers) → `application` (robot behavior).
- **Message Center pub-sub** — applications never include each other directly; they communicate through decoupled topics.
- **Daemon offline watchdog** — monitors module heartbeats and triggers a callback to enter a safe state on timeout.

### Hardware

| Item | Detail |
| --- | --- |
| MCU | STM32H723VGT6 (Cortex-M7, up to 550 MHz, 1 MB Flash) |
| Board | DaMiao DM-MC02 |
| Onboard IMU | BMI088 (SPI, with temperature compensation & online calibration) |
| Linker script | `STM32H723XG_FLASH.ld` |
| RTOS | FreeRTOS |

### Differences from basic_framework

This project is a **deep port and trim** of `basic_framework`. Key differences:

| Aspect | basic_framework | Meta-Embedded-NG |
| --- | --- | --- |
| MCU / board | STM32F407 (DJI C-board) | STM32H723 (DM-MC02) |
| Modules folder | `modules/` | `module/` |
| Motor drivers | DJI / DM / HT / LK / servo / stepper | DJI / DM / servo / stepper / **XM (new)**; HT & LK removed |
| Sensor/peripheral modules | Includes IST8310 magnetometer, OLED, Bluetooth, TFminiPlus LiDAR | These modules removed |
| Referee system | Original | Referee protocol & task fixed |

The core three-layer architecture, Message Center design, and coding conventions are unchanged.

### Project Structure

```
Meta-Embedded-NG/
├── application/      # Robot behavior: chassis, gimbal, shoot, cmd dispatch, per-robot dirs (sentry/infantry)
├── module/           # Reusable algorithms & device drivers (motors, IMU, message center...)
├── bsp/              # Board support package wrapping STM32H7 on-chip peripherals
├── Core/             # CubeMX-generated peripheral/FreeRTOS/interrupt code (Inc + Src)
├── Drivers/          # STM32H7xx HAL & CMSIS (vendor code)
├── Middlewares/      # FreeRTOS, USB device library, CMSIS-DSP, SEGGER RTT
├── USB_DEVICE/       # USB device middleware (App + Target)
├── Makefile          # arm-none-eabi-gcc build script
├── Meta-Embedded-NG.ioc   # CubeMX project file
└── STM32H723XG_FLASH.ld   # Linker script
```

### Architecture

- **application/**: `chassis` (kinematics / power limiting), `gimbal` (attitude control), `shoot` (friction wheels + loader), `cmd` (command dispatch — consumes remote/keyboard input and publishes control setpoints), plus per-robot dirs `sentry` and `infantry` — each holds a common entry point `robot.c`/`robot_task.c` and its own params header `<robot>_def.h`. The robot type is selected at build time via `make ROBOT=<robot>`; the common message contract and variant selector live in `application/robot_def.h`.
- **module/**: motor drivers (`motor/`: DJI, DM, XM, servo, stepper), `algorithm` (PID, Kalman filter, LQR, EKF attitude solver, CRC), `message_center`, `daemon`, `BMI088`, `imu`, `encoder`, `can_comm` (multi-board CAN), `super_cap`, `referee` (referee system / UI), `remote` (DT7 receiver), `master_machine`, `standard_cmd`, `unicomm`, `alarm`.
- **bsp/**: `can`, `usart`, `spi`, `iic`, `pwm`, `adc`, `gpio`, `log`, `usb`, `flash`, `dwt`.
- **FreeRTOS tasks**: INS attitude estimation (~1 kHz), motor control, system monitoring, and the main robot logic run concurrently.

Most submodules ship with a companion `*.md` doc (e.g. `module/message_center/message_center.md`, `module/motor/DJImotor/dji_motor.md`, `module/BMI088/bmi088.md`) — read them before diving in.

### Build

Requires the **GNU Arm Embedded toolchain** (`arm-none-eabi-gcc`).

> The current Makefile creates output directories automatically. An optional `mkdir -p build` is safe to repeat.

```bash
# Optional: create the build directory in advance
mkdir -p build

# Build per robot variant (ROBOT defaults to sentry)
# -j enables parallel compilation; pass -j<N> to cap the job count, e.g. -j8
make -j ROBOT=sentry      # produces build/sentry/Meta-Embedded-NG-sentry.elf / .hex / .bin / .map
make -j ROBOT=infantry    # produces build/infantry/Meta-Embedded-NG-infantry.elf / .hex / .bin / .map
make -j                   # same as make -j ROBOT=sentry

# Clean the current variant's build/<robot>/ directory
make clean

# Use a specific toolchain without changing PATH
make -j ROBOT=infantry GCC_PATH=/path/to/gcc-arm-none-eabi/bin
```

> Each robot compiles into its own `build/<robot>/` subdirectory, so switching robots needs no `make clean`.

> There is no flash target in the Makefile. Use your team's debugger/programmer (e.g. OpenOCD + CMSIS-DAP, Ozone) to flash the generated `.hex`/`.bin` to the board.

### Configuration

- **Robot type is selected at build time**: `make ROBOT=sentry|infantry` passes `-DROBOT_SENTRY` / `-DROBOT_INFANTRY` and compiles the matching `application/<robot>/` directory.
- The message contract shared across modules (chassis/gimbal/shoot enums and structs) and the robot selector live in `application/robot_def.h`; each robot's own physical params (wheel geometry, gimbal limits, loader params, etc.) live in `application/<robot>/<robot>_def.h`.
- Adding a robot: copy `application/sentry/` to `application/<new>/`, add an `#elif defined(ROBOT_<NEW>)` arm in `robot_def.h`, and add an `else ifeq ($(ROBOT),<new>)` arm in the Makefile's robot-variant block.
- BMI088 supports power-on online calibration; see `module/BMI088/bmi088.md` for parameters.

### Conventions & Contributing

See [`AGENTS.md`](AGENTS.md). Highlights:

- Follow Conventional Commits: `feat:`, `fix:`, `chore:`, `doc:`, `feat!:` (breaking change).
- 4-space indentation; macros `UPPER_SNAKE_CASE`, config structs end in `_s`, typedefs end in `_t`, init/control APIs like `ChassisInit()` / `DJIMotorControl()`.
- Use `#pragma pack(1)` for CAN/serial protocol structs.
- A clean `make` is the minimum verification for every change; document hardware validation for behavior changes.
- Do not hand-edit generated CubeMX blocks unless the change is intentional and documented.

### License & Acknowledgements

Released under the **MIT License**, Copyright (c) 2025 Meta-Team. See [`LICENSE`](LICENSE).

Sincere thanks to the **Hunan University RoboMaster YueLu Team** for the open-source [`basic_framework`](https://github.com/HNUYueLuRM/basic_framework) (MIT, Copyright (c) 2022 NeoZng / Even), on which this project is built.
