# DGE_vision_26 × STM32 GM6020 自瞄联调状态

更新：2026-10-18。本仓库是 **STM32 电控代码**；视觉代码在 https://github.com/yzixuan306-wq/DGE_vision_26 ，不是本仓库的子目录。请勿用另一个 `fashejigou` J4310 Yaw-only 台架项目替代当前 GM6020 双轴工程。

## 已有的电控链路

- `module/imu/ins_task.*`：BMI088 姿态快照；`application/auto_aim/auto_aim_coordinates.*`：四元数坐标转换，向 DGE Hero 的 `w,x,y,z` 惯例适配。
- `module/cboard/cboard_protocol.*`、`module/vision_comm/*`：CBoard 二进制协议帧编解码与接收保护。
- `bsp/usb/vision_usb_port.*`：STM32 USB CDC 收发、缓冲、断线处理。
- `application/auto_aim/auto_aim.c`：接收目标角、超时判断、控制权限仲裁和姿态/状态遥测。
- `application/cmd/dt7_cmd.c`：遥控器模式；`application/gimbal/2yaw_gimbal.c`、`gimbal_control.c`：原 GM6020 双轴姿态外环/速度内环与限位保护。
- 发射机构**未接入**；任何接收帧中的 `shoot=1` 不应引起射击。

## 线协议（USB CDC，和 DGE io/cboard.cpp 对齐）

| 方向 | 字节长度 | 帧格式 | 编码 |
|---|---:|---|---|
| MCU → 视觉 | 10 | `AA + w,x,y,z(int16×4) + DD` | 每分量乘以 10000，小端，四元数 `w,x,y,z` |
| MCU → 视觉 | 9 | `BB + speed,mode,shoot_mode,ft_angle(int16×4)` | 小端；速度×100，角度×10000 |
| 视觉 → MCU | 8 | `CC + control(u8) + shoot(u8) + yaw(int16) + pitch(int16) + EE` | 小端；角度 rad×10000，绝对目标角，不是角速度 |

通信协议没有 CRC；固件在接收端检查长度、帧头/尾、角度合法性，并防止使用过期目标。实时视觉目标超时阈值为 **150 ms**。`control=0` 取消目标并保持，不应使云台归零。注意 int16×10000 的角度表达范围是约 ±3.2767 rad。

## 2026-10-18 已完成的实机验证

1. Ubuntu 通过 CMSIS-DAP_JX + OpenOCD 0.12.0 识别 STM32H723；安全观察版已通过 `program/verify/reset` 完成烧录。
2. 使用观察版 `ROBOT=sentry`、`AUTO_AIM_USB_MODE=1`、`AUTO_AIM_COORDINATES_CONFIRMED=0`、`GIMBAL_MOTOR_OUTPUT_ENABLED=0`：实时采集到有效四元数和状态二进制帧。USB 断开重连后端口编号可能改变，不能假定固定是 ttyACM0/ttyACM1。
3. 手动低速转动镜头（站在镜头后方向前看）得到：左转 ΔYaw **+26.2°**、ΔPitch +1.0°；右转 ΔYaw **−28.1°**、ΔPitch −0.6°；抬头 ΔYaw +1.7°、ΔPitch **−15.6°**；低头 ΔYaw −2.5°、ΔPitch **+22.3°**。四方向均符合 Hero 约定。
4. 观察状态 `mode=0`（idle），联调报文速度 15.0 m/s 为**假设值**，不是弹速实测值。方向性验证不代表角度绝对零位标定已完成。
5. Linux 主机端自动化测试 `python3 tests/run_tests.py` 覆盖 CBoard、USB、切换与超时、手动云台、观察版禁止电机输出等。

## 尚未完成，切勿直接打开自动跟随

- **尚未实机测试视觉 → MCU 的指令闭环**：代码和主机模拟测试已有，不代表真实 DGE 目标角已经正确控制过电机。
- **相机内参与外参、机械零位还未确认**。需核对视觉 `configs/hero.yaml` 中 `R_camera2gimbal`、`t_camera2gimbal`、`R_gimbal2imubody` 以及目标板实际安装情况。
- 当前电控固件烧录的是禁用电机输出的观察版。不要把 `AUTO_AIM_COORDINATES_CONFIRMED=0` 改为 1，或把 `hero.calibration_confirmed=false` 改为 true，仅为了获得 `mode=1`。
- 建议下一步先进行**不驱动电机的双向收发验收**，确认真实 DGE 下发的帧符合协议、方向及目标参考系；之后再按实机安全流程逐级验证小角度云台闭环。保持 `hero.auto_fire=false`。

更多协议、模式与安全条件见 [`application/auto_aim/README.md`](../application/auto_aim/README.md)。
