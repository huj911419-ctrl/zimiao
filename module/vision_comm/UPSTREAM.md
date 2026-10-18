# carm_linkcoure 通信核心接入记录

来源：[huj911419-ctrl/carm_linkcoure](https://github.com/huj911419-ctrl/carm_linkcoure)，
固定提交 `a44bdf5cc534f112702543960b18ab68f0df5e01`，2026-10-18。
保留上游 MIT 许可证于本目录 `LICENSE.upstream`。

## 实际接入

- `vision_comm.c/.h`：上游纯 C 流式接收、错误帧重同步、半帧20 ms超时、到达时间和计数逻辑。
- `bsp/usb/vision_usb_port.c/.h`：上游2048字节接收队列、USB接收时间、断连/溢出通知、64字节发送持久缓冲区和收发计数。
- 正式链路：`GimbalRuntimeComm → VisionUSBPortRead → AutoAimReceiveByte → CBoardParseByteAt → VisionCommFeedByte`。
- 二进制发送：`INS_GetVisionSnapshot → 原坐标转换/有效性检查 → CBoardPackAttitude/Status → VisionUSBPortTransmit`。
- USB只注册一个拥有者：二进制模式由VisionUSBPort管理；原调试模式仍由GimbalRuntime管理。
- 当前工程在RobotInit前启动USB，而上游建议枚举前初始化：因此给VisionUSBPortInit增加回调与RX指针注册的临界区，避免已经枚举时中断访问尚未发布的指针。
- 同步上游Cube USER CODE内两项必要适配：接收回调先复制数据再重挂OUT缓冲区；发送前检查USB配置状态及类指针。

## 保持当前 Hero 线格式

上游的通信测试与当前 `DGE_vision_26/io/cboard.*` 存在实际字节差异，不能整包照搬：

| 帧 | carm_linkcoure 台架格式 | 当前 Hero 要求 |
|---|---|---|
| AA，10字节 | x,y,z,w + DD | w,x,y,z + DD |
| BB，9字节 | int16 speed、int16 mode、uint8 shoot_mode、int16 ft_angle、DD | int16 speed、mode、ShootMode、ft_angle，无帧尾 |
| CC，8字节 | CC + control/shoot + int16 yaw/pitch + EE | 相同；正式应用另检查角度范围 |

所以保留 `module/cboard` 的Hero遥测打包，并让其接收包装调用上游解析器。
上游 `VisionCommPackTelemetry` 保留供来源回归测试使用，不进入正式遥测发送链路。
没有修改视觉 `hero.cpp`、`io::CBoard` 接口，没有用字段重命名伪装坐标变换。

## 保持的电控行为

原遥控方向、PID、安装坐标补偿、150 ms视觉目标有效期、100 Hz姿态/50 Hz状态、联调用15 m/s及故障/OFF优先级不变。
20 ms只限制一帧的分包间隔，与150 ms完整有效视觉目标超时不同。
通信任务延后处理时仍使用USB中断的到达时间。
`control=0`不回零，`shoot`仍只计数不执行。坐标确认仍为0，电机输出由已有构建参数单独选择。
没有导入上游测试应用、固定模拟四元数、25 m/s模拟弹速、CD回执、UART10测试任务、看门狗刷新或旧13/34字节协议。
因此上游 `tools/usb_comm_test.py` 不能原样拿来验收正式固件：它期望固定假数据和CD回执。
已有本聊天的IMU/相机只读工具可继续读取正式AA/BB数据。

## 来源校验

| 文件 | 上游 SHA256 | 本地处理 |
|---|---|---|
| `module/vision_comm/vision_comm.c` | `82c931ea3e15e0938ba9726bdd9b020172702690f9182c0ac465aec824e74dfc` | 原样导入 |
| `module/vision_comm/vision_comm.h` | `578587dc75b83b1514fff36056f69ab5970994892f895c5d932e69d6107dcac4` | 原样导入 |
| `bsp/usb/vision_usb_port.c` | `b2ab8c9d08ff4b85caa817eaf30a3509b61dc62132bebcbef5b2324281669f07` | 仅 Init 增加回调注册临界区 |
| `bsp/usb/vision_usb_port.h` | `09fbde622d699a7a306e849f0506ba759cde612c796147dc83799eb13a2f4bb5` | 原样导入 |
| `tests/vision_comm_test.c` | `7ca0fbb013637bb8571bccd65cc378b9b72e294eb49127375d4ed638d3d3f1c9` | 原样导入 |
