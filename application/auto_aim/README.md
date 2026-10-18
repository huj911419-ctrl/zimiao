# GM6020 云台自瞄跟随接入说明

**实机验证更新（2026-10-18）：** 见 [`docs/VISION_INTEGRATION_STATUS.md`](../../docs/VISION_INTEGRATION_STATUS.md)。STM32→Ubuntu 四元数与状态帧已实测，手动四方向与 Hero 约定一致；目前仍为电机输出禁用的观察版，视觉→电机闭环尚未验证。

2026-10-13。修改项目：`/home/hjj/Development/Embedded/damiao_pure - 副本`。
本次只实现云台跟随，不启用发射、裁判、摩擦轮或拨盘；收到 `shoot=1` 只计数，不执行。
视觉仓库保持原样，未修改 `hero.cpp`、`io::CBoard` 或其接口。

## 已接入的链路

```text
USB CC 控制帧 → 有界字节流解析 → 最新目标邮箱（记录 USB 到达时间）
                                        ↓
DT7 模式授权 → 健康/标定/阶段检查 → AUTO 绝对角度目标
                                        ↓
BMI088 → INS 一致快照 → 角度 P → 速度 PI → GM6020 电压命令
                                        ↓
                        编码器限位、温度、输出租约与故障保护
```

AUTO 目标单位为 rad，不按角速度处理，不做摇杆式积分。Yaw 用当前连续反馈附近的最短角度差，
跨 ±π 和多圈不绕远路；Pitch 继续按编码器软硬限位处理。AUTO 保留速度、PI 输出及积分限幅。
角度阶跃使用现有角度环与速度限幅，未加入新的自动轨迹规划器；实机需要确认动态跟随效果。

## 遥控器操作

| 右拨杆 | CBoard 二进制模式 | 原调试模式 |
|---|---|---|
| 下档 | OFF，零输出；回中并等 READY 才能再使能 | 原 OFF |
| 中档 | 请求自瞄；缺目标或接管条件不满足时保持 | 原 HOLD |
| 上档 | 原手动摇杆控制 | 原手动控制 |

左拨杆仍为下档双轴、中档 Pitch、上档 Yaw。自瞄只允许双轴，即左下档；
切轴先右下档回中。AUTO 接管还要求 stage=2、遥控在线、反馈健康、USB 已枚举、
坐标确认有效及联调弹速有效。未回中使能、轴掩码改变或故障时，不上报 auto_aim。

`control=0` 立即撤销视觉目标并保持当前姿态，随帧的零角度不触发回零。
最后完整有效命令超过 150 ms、USB 断开或接收溢出，同样撤销目标；
无效字节和错误帧不会刷新有效目标时间。恢复后必须收到新的有效目标。
退出自瞄、OFF、手动或故障期间收到的目标不会在下一次自瞄会话复用。
当前规则是视觉目标丢失时保持，新的有效目标可在仍获授权的自瞄会话重新接管；
遥控/IMU/电机等现有故障仍锁存，必须 OFF 重新准备。

新增状态编号：`state=5` 为 AUTO；原 0 OFF、1 READY、2 HOLD、3 MANUAL、4 FAULT 不变。
新增请求编号：`request=3` 为 AUTO。USART10 日志继续使用原格式。

## USB 模式和帧格式

`AUTO_AIM_USB_MODE=1` 为默认 CBoard 二进制模式，唯一 USB 收发回调仍由 gimbal_runtime 管理。
此模式关闭 USB 文本、JustFloat、ASCII 调参和 CAN 扫描文字；日志继续走 USART10/RTT。
二进制帧不掺入 ACK、表头或换行。

`AUTO_AIM_USB_MODE=0` 恢复原调试模式，包括 IMU 文本、JustFloat 和 ASCII 调参；此时不接受视觉接管。
USB 模式通过构建参数明确选择，当前不支持从二进制流发送 ASCII 命令来切换模式。

| 方向 | 长度 | 格式 |
|---|---:|---|
| 电控→上位机姿态 | 10 | `AA + int16(w,x,y,z) + DD`，分量×10000 |
| 电控→上位机状态 | 9 | `BB + int16(speed,mode,ShootMode,ft_angle)` |
| 上位机→电控目标 | 8 | `CC + uint8(control,shoot) + int16(yaw,pitch) + EE` |

全部 int16 小端。速度 m/s×100，目标角 rad×10000。状态帧没有帧尾。
模式 0 idle、1 auto_aim；ShootMode 和 ft_angle 当前为预留 0，不表示射击授权或实测机构角度。
协议无 CRC：这里只能检查帧头、长度、帧尾、布尔字段和角度范围，不能检测所有负载位翻转。

姿态目标 100 Hz，状态目标 50 Hz；USB 忙则跳过、不积压旧遥测。
只发送有效、已标定、年龄不超过 10 ms 的新 INS 样本；重复序号不重复发送姿态。
失效姿态不伪造为新数据，状态在姿态失效时上报 idle。

## 不发射时为什么仍有弹速

Hero 当前即使 `auto_fire=false`，仍先做弹道解算才生成跟随目标。
`configs/hero.yaml` 的有效弹速范围是 10～20 m/s，0 会使控制器返回 `invalid_speed`。
因此电控使用显式配置 `AUTO_AIM_TRACKING_SPEED_MPS=15.0f` 作为不发射的联调参数。
它不是测速结果，不能用于证明真实弹道、命中率或射击性能。后续有真实测速再替换来源。
不论上位机是否意外发送 shoot，本次代码都不调用发射机构。

## 坐标变换与实际标定

配置集中在 `application/auto_aim/auto_aim_config.h`，默认
`AUTO_AIM_COORDINATES_CONFIRMED=0`。在确认前可上报用于核对的候选姿态，但始终 idle，
右中档按保持处理，不允许视觉接管。不要仅为获得 mode=1 直接打开确认标志。

电控欧拉字段中的 Pitch 是 X 旋转，Roll 是 Y 旋转；Hero 使用 Z-Y-X 的 yaw/pitch/roll，
其 pitch 抬头为负。变换采用完整四元数，不直接把两个名为 pitch 的字段连接起来：

```text
q_wire = W * q_INS * B
q_Hero = inverse(C) * q_wire * C
q_INS_target = inverse(inverse(C) * W) * q_Hero_target * inverse(B * C)
```

`W` 为 AUTO_AIM_WORLD_QUATERNION，`B` 为 AUTO_AIM_BODY_QUATERNION；
`C` 为 AUTO_AIM_SOLVER_QUATERNION，必须对应视觉 YAML 的 `R_gimbal2imubody`。
源码 Solver 的公式就是 `C^-1 * R_wire * C`，所以 C 也进入了目标的逆变换。
所有配置四元数均为 w,x,y,z、单位四元数。

基础轴变换采用 W=Z轴+90°、B=Z轴−90°，使 Hero forward=-INS Y、left=INS X、up=INS Z，
将 INS 的 Pitch-X 旋转映射为 Hero Pitch-Y，Yaw 保持同向；C 默认单位四元数，
对应参考仓库当前 `hero.yaml` 的单位矩阵。此候选只通过数学测试，尚未核对实物安装。
若安装或外参不同，应修正 W/B/C；不能只调 PID 或单独反转下发目标来掩盖坐标错误。

静止及手动阶段逐项确认：

1. 保持电机输出停止，记录静止四元数、Yaw/Pitch 编码器与上位机 Z-Y-X 角度。
2. 手转偏航正反方向，核对上报 Yaw、连续角度与电控电压符号。
3. 手抬/下压 Pitch，核对 Hero pitch 抬头为负，并确认电控 INS Pitch 与编码器符号。
4. 核对相机内参、相机→云台外参、Solver C、机械零点及 INS 安装变换。
5. 核对完成才开启电控 `AUTO_AIM_COORDINATES_CONFIRMED=1`；
   上位机另需完成自身相机/云台标定并设 `hero.calibration_confirmed=true`。
   上位机保持 `hero.auto_fire=false`。

## 构建与测试

GNU Arm 工具链可指定：

```bash
make -j8 ROBOT=sentry AUTO_AIM_USB_MODE=1 AUTO_AIM_COORDINATES_CONFIRMED=0 \
  GCC_PATH=/home/hjj/.local/share/stm32cube/bundles/gnu-tools-for-stm32/14.3.1+st.2/bin

# 原调试固件，恢复 USB 文本/JustFloat/ASCII
make -j8 ROBOT=sentry AUTO_AIM_USB_MODE=0 \
  GCC_PATH=/home/hjj/.local/share/stm32cube/bundles/gnu-tools-for-stm32/14.3.1+st.2/bin

python3 tests/run_tests.py
```

Makefile 现在创建父目录，且记录 CFLAGS；在同一输出目录切换 USB/坐标参数会重新编译。
`ROBOT=infantry` 使用同一云台链路，仅 sentry 多被动 CAN 扫描（自瞄流中不会输出扫描文字）。
本次没有改 Windows VS Code 烧录任务，也没有执行烧录。

主机测试覆盖原 8 组回归以及实际 DT7 自瞄拨杆映射、CBoard 黄金字节、分包/粘包/重同步、
坐标往返与非单位 Solver 外参、绝对角度不积分、Yaw 跨圈、Pitch 限位、
control=0 不回零、150 ms 超时、旧会话目标不复用、USB 断连/溢出、
故障/过温/控制周期异常优先、标定关闭禁止接管、有效弹速编码及 shoot 忽略。

实机四元数发送与四方向已经验证；视觉接管和电机跟随仍未验证。完成剩余标定后，先单轴小目标跟随，再测跨±π和Pitch端点，
再测双轴、control=0、拔USB、关遥控；记录USART10/RTT日志。
本次不能据主机测试或编译通过宣称云台实机稳定或视觉标定完成。

## 关键文件与源码对照

- `module/cboard/cboard_protocol.[ch]`：帧编码、字节流解析，无 HAL/RTOS 依赖。
- `application/auto_aim/auto_aim_coordinates.[ch]`：四元数及目标角变换，无 HAL/RTOS 依赖。
- `application/auto_aim/auto_aim.[ch]`：最新目标、接管授权、超时、姿态/状态上报。
- `application/gimbal/gimbal_runtime.c`：统一 USB 回调，明确选择二进制/调试流。
- `application/gimbal/gimbal_control.[ch]`：AUTO 状态、绝对目标与原保护。
- `application/gimbal/2yaw_gimbal.c`：接管仲裁进入实际电机适配器。
- `application/cmd/dt7_cmd.c`：右中档自瞄映射。
- `tests/run_tests.py`：Linux/Windows GCC 主机测试入口；旧 PowerShell 回归仍保留。

视觉对照版本：`yzixuan306-wq/DGE_vision_26` commit
`06eca11badf282574c70396d6849c7684b31dcd7`。
参考：[CBoard](https://github.com/yzixuan306-wq/DGE_vision_26/blob/06eca11badf282574c70396d6849c7684b31dcd7/io/cboard.cpp)、
[Hero控制器](https://github.com/yzixuan306-wq/DGE_vision_26/blob/06eca11badf282574c70396d6849c7684b31dcd7/tasks/hero/controller.cpp)、
[Solver](https://github.com/yzixuan306-wq/DGE_vision_26/blob/06eca11badf282574c70396d6849c7684b31dcd7/tasks/auto_aim/solver.cpp)。

## 本次最终验证结果

- 12组主机测试全部通过，包括默认禁止接管和测试用已确认坐标两种配置。
- sentry/infantry默认二进制配置、sentry调试配置、infantry测试用坐标确认配置全部编译通过。
- 新增自瞄、坐标和协议模块无编译警告；其他保留模块各构建有20条原有警告。
- 同一构建目录切换USB模式已验证会重新编译，避免旧宏配置被复用。
- 2026-10-18 已烧录并实测安全观察版；未修改视觉仓库，四元数四向方向通过，尚未进行视觉闭环跟随。

## 2026-10-13 固定零位与通信补偿更新

沿用现有遥控代码与既有实机方向：右转INS Yaw减小，抬头INS Pitch减小。
不改变摇杆方向、PID、电机输出符号或原始INS反馈。用户近似水平固定姿态的152个样本：
INS Pitch均值−0.130816°（标准差0.00514°），Roll均值4.316776°（标准差0.01805°），
Pitch编码器749～750。IMU有效、标定有效、年龄1ms，两轴输出0。
该段遥控离线，旧故障25仍锁存；它用于静态姿态参考，不代表可使能或闭环已通过。

通信BODY变换追加上述固定倾斜的逆旋转。零位上报Pitch/Roll约0，视觉目标经过
相同变换的逆映射回原控制坐标。Yaw继续使用原INS初始化参考；没有把此段6.822°
当作永久安装零偏，六轴IMU仍不提供无漂移绝对航向。
此补偿仅为联调候选，保留AUTO_AIM_COORDINATES_CONFIRMED=0；原始INS初始化/安装参数不变。
参考近似水平，不表示已完成相机外参精密标定。

新增完全被动的二进制检查构建：

```bash
make -j8 ROBOT=infantry AUTO_AIM_USB_MODE=1 AUTO_AIM_COORDINATES_CONFIRMED=0 \
  GIMBAL_MOTOR_OUTPUT_ENABLED=0 \
  GCC_PATH=/home/hjj/.local/share/stm32cube/bundles/gnu-tools-for-stm32/14.3.1+st.2/bin
```

该固件发送CBoard四元数/状态，任何遥控或视觉请求都不能产生电机输出。
正常构建的GIMBAL_MOTOR_OUTPUT_ENABLED默认仍为1，保持原手动行为。
追加被动输出测试后共13组主机测试通过。
采集记录位于本聊天outputs/usb-logs/fixed-zero-usb.log及fixed-zero-summary.json。


## 2026-10-18 接入已验证通信仓库

已把 `huj911419-ctrl/carm_linkcoure` 提交 `a44bdf5` 的可复用通信核心和USB适配层接入正式运行链路。
来源、平台适配及上游测试帧与Hero帧的区别见 `module/vision_comm/UPSTREAM.md`。
二进制USB使用VisionUSBPort，调试USB仍使用原回调；接收时间不会在任务消费时重置。
原姿态/状态的帧格式、真实INS快照、坐标补偿、控制仲裁和不发射逻辑保留。
没有导入上游测试任务、固定模拟遥测或CD回执。坐标确认标志仍为0。
