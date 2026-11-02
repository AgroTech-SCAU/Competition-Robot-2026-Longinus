# UART7 光流与现有 IMU 里程计融合说明

本次在现有工程内完成接入，主控为 STM32H723VGT6。用户确认光流模块位于底盘中心底部，使用 UART7。安装绕 Z 轴的实际方向尚未实测，编译默认传感器 +X 向前、+Y 向左，安装角为 0 rad，方向符号为 +1。

## 交付与烧录

- 固件：`mcu/build/OpticalFlow/robot.hex`、`robot.bin`、`robot.elf`。
- HEX 自带地址；BIN 下载起始地址是 `0x08000000`，沿用 `STM32H723VGTX_FLASH_USER.ld`。
- 使用工程原有的 CMSIS-DAP/OpenOCD 或 STM32 下载工具，下载 HEX 即可。代码已完成编译、链接和主机集成测试，未在实物上烧录或进行里程精度标定。
- 完整交付包中的 `mcu` 含 HAL/CMSIS、启动代码及 CubeMX 生成文件，`external` 含工程实际依赖的 Standard SDK，不依赖接收方原先的构建缓存。
- 不接光流时仍执行原来的底盘轮速 + IMU 融合；光流设备未发送数据不阻止启动。

在仓库根目录执行：

```powershell
.\tools\build_opticalflow.ps1
```

脚本优先使用当前用户 EIDE 的 Arm GNU 编译器，也可显式指定：

```powershell
.\tools\build_opticalflow.ps1 -Compiler 'C:\path\to\arm-none-eabi-gcc.exe'
```

编译配置与现有 Debug 目标一致：Cortex-M7、硬件单精度浮点、hard ABI、GNU C11、Debug、newlib-nano、原有 FLASH 链接脚本。该脚本扫描当前源文件并完整构建，避免复用原 `builder.params` 中漏列 UART/SPI/TIM 等 HAL 源文件的旧列表。EIDE 原工程本身仍可使用，无需改变项目层级。

## 接线与协议

| 光流模块信号 | STM32 信号 |
|---|---|
| TX | PE7 / UART7_RX |
| RX | PE8 / UART7_TX；只接收数据时可不连接模块 RX |
| GND | 主控 GND |
| 电源 | 按模块实际规格供电；不要把 UART 信号电平与供电电压混淆 |

UART7 沿用已有的 115200、8N1、正常极性配置，不使用 UART5 遥控串口，也不占用 USART1 日志串口。

自动识别 UPIXELS 14 字节、UPIXELS_EXT 20 字节，以及包中指定的 PX4 光流/测距帧。PX4 路径沿用系统 ID `0x00`、组件 ID `0x9E` 的检查；光流有效标记沿用 `quality == 0xF5`。APM/MSP 不在支持范围。程序不会自动向模块发送协议切换或写入掉电保存设置。

## 保持现有架构的方式

原工程并非通过 IMU 加速度二次积分计算 X/Y。原来的 `chassis_imu_odom` 已使用舵轮正运动学速度和 IMU 数据进行九状态滤波：

```text
X = [x, y, z, roll, pitch, yaw, vx, vy, wz]^T
```

本次保留该状态定义、原有预测模型、噪声配置、BMI088 姿态与零偏处理、500/250/100Hz 调度、应用状态机和 `odom.get_*()` 接口。

原有六个观测是：

```text
[wheel_vx, wheel_vy, wheel_wz, acc_roll, acc_pitch, imu_gyro_z]
```

本次只增加两个独立光流速度观测，总观测维度从 6 变为 8；通用滤波器原本支持最大 9 维观测，不需要修改 `kalman.c/.h` 或 `matrix.c/.h`。

保留旧 `chassis_imu_odom.update()`，其内部调用新 `update_with_flow(..., NULL, dt)`，因此原有调用者不需要迁移。

```text
UART7 中断 -> 带接收时间的字节队列
                         |
原有 odom.process() 250Hz -> 光流解析、有效性检查、速度换算
                         -> 原有 IMU 更新与底盘速度读取
                         -> 同一个九状态滤波器增加光流速度观测
                         -> 原有 odom.get_odom()/get_velocity()/get_angle()
```

中断中只收字节、记录时间、重新启动接收，不运行光流解析、三角函数或滤波。所有状态更新仍由同一里程计任务串行完成。

## 光流变成速度的过程

UPIXELS 原始有符号整数先除以 10000 得到积分光流角量，微秒除以 1000000 得到积分时间，毫米除以 1000 得到米制距离。PX4 直接读取 rad 浮点量。

在近似水平、平面地面条件下：

```text
vx_sensor = flow_x_rad * height_m / flow_dt_s
vy_sensor = flow_y_rad * height_m / flow_dt_s
```

例如 `flow_x_raw=200`、`height=0.5m`、`flow_dt=0.02s`，得到 `vx_sensor=0.5m/s`，该光流积分窗口内位移是 0.01m。

然后同时乘方向符号，并按安装角 alpha 转到车体轴：

```text
vx_body = sign * (cos(alpha)*vx_sensor - sin(alpha)*vy_sensor)
vy_body = sign * (sin(alpha)*vx_sensor + cos(alpha)*vy_sensor)
```

当前底盘服务已经把 `cur_vx/cur_vy` 投影到首次有效 IMU 姿态定义的参考平面。新光流服务在同一次有效 IMU 更新时记录相同的 roll/pitch 参考，并使用同一个 `R0^T * R` 的 XY 投影公式，使两种速度处在一致的参考平面。即使尚未收到光流，也会先记录参考姿态，避免在机器人已经倾斜后才建立不同参考。

这个投影沿用已有底盘约定，不代表实现了完整的倾斜相机成像补偿。超过允许倾角时跳过光流。

## 同一个滤波器如何融合

保留的预测主要是：

```text
x   += (vx*cos(yaw) - vy*sin(yaw))*dt
y   += (vx*sin(yaw) + vy*cos(yaw))*dt
yaw += wz*dt
roll/pitch 继续由现有陀螺积分及可信重力方向观测修正
```

IMU 加速度在这里用于 roll/pitch 的重力方向约束，并未新增平移加速度二次积分。IMU gyro_z 与舵轮 wz 继续约束角速度，航向由现有角速度状态积分。

用户安装在中心，故 `sensor_x_m=sensor_y_m=0`，两个新观测直接观察状态 vx/vy。

保留安装偏移参数时，采用以下量测模型：

```text
flow_vx = vx - wz*sensor_y_m + noise_x
flow_vy = vy + wz*sensor_x_m + noise_y
```

相应光流观测矩阵的非零项为：

```text
H_flow_x[VX]=1, H_flow_x[WZ]=-sensor_y_m
H_flow_y[VY]=1, H_flow_y[WZ]= sensor_x_m
```

这样可以把安装偏移和角速度不确定性一起保留在滤波模型中。中心安装时两个 WZ 项都为零。

滤波继续使用现有计算：

```text
innovation = z - H*X_predict
S = H*P_predict*H^T + R
K = P_predict*H^T*inverse(S)
X = X_predict + K*innovation
P = (I-K*H)*P_predict
```

光流通过同一个 P 和状态向量修正速度及与其相关的位置，没有建立另一份光流位置再与 IMU 位置做百分比平均，也没有重复累计两份位移。

没有新光流样本时，两个新增观测行置 `H=0,z=0,R=1`，数学上不会给状态或协方差提供额外约束。旧的六个观测保持不变。

## 频率、静止与降级

- 光流帧中的积分时间只用于计算该帧平均速度，不能用固定 0.004s 代替。
- 滤波的 dt 仍由现有 IMU 时间戳/回退时钟逻辑计算。
- 每个光流新样本通过 `updated` 标记最多进入滤波一次。50Hz 光流不会在 250Hz 任务中被重复融合五次。
- 最新有效样本在 100ms 内可用于否决错误的静止判断，但不会因此重复成为观测。
- 如果一次任务中积压多帧，解析所有帧并保留最后一帧有效状态，最多融合最新样本一次，不把旧帧当作当前独立观测反复更新。
- 原有轮速/gyro 静止条件仍保留。若新鲜光流显示速度模长大于 0.02m/s，就不强制清零平移速度，避免车体被推动、轮速近零时丢掉运动。
- 若轮速、gyro、光流都满足静止窗口，将光流观测也按零速处理，避免小光流噪声通过位置与速度的协方差引入静止位移。
- 光流或有效高度过期、无效标记、非法数值、过大速度、过大倾角均不会进入光流更新，自动回到现有轮速 + IMU 路径。
- UART 出错会在任务中重新接收；字节队列溢出会清理不完整流和旧高度缓存，随后从新完整帧恢复。
- IMU 尚在校准时也排空光流队列，避免长期堆积。IMU 不可用时仍沿用原工程的 NOT_READY 行为，没有新增加“只靠光流继续运行”的模式。

解析器新增了“最后一次实际有效测距时间”，使用旧距离不会刷新此时间。原压缩包会无限保留旧距离；本工程在 100ms 后停止用它进行速度换算。

## 当前参数与方向确认

安装与串口参数集中在 `mcu/src/service/assemble/assemble_odom.c`，光流有效范围集中在 `opticalflow_default_config()`，融合噪声集中在 `odom_default_config()`。

| 参数 | 当前值 | 说明 |
|---|---:|---|
| 安装角 | 0 rad | 默认传感器 X 前、Y 左，实物方向需验证 |
| flow_sign | +1 | 两轴一起翻转；不能单独修复一轴镜像 |
| 安装偏移 | X=0、Y=0m | 用户确认中心安装 |
| fixed_height_m | 0 | 不猜测固定高度，必须有新鲜 TOF；若改用固定值，需要实际测量 |
| 高度接受范围 | 0.02～4m | 软件检查范围，不等于硬件保证量程 |
| 光流积分时间 | 0～0.1s，排除 0 | 使用帧中时间 |
| 光流速度模长上限 | 5m/s | 排除异常速度 |
| 最大 roll/pitch | 0.35rad，约20° | 超过任一轴限制跳过光流 |
| 光流/测距超时 | 各100ms | 依据 UART 接收时间，不用任务解析时间伪造新鲜度 |
| flow_velocity_noise | 0.0025 (m/s)^2 | 标准差0.05m/s |
| flow_velocity_gate | 2m/s | 每轴残差门限；<=0关闭；首次有效光流不设此门限 |
| flow_static_threshold | 0.02m/s | 静止判断中光流速度门限 |
| 原轮速噪声 | 0.04 | 保留原值 |
| 原 IMU gyro 噪声 | 0.03 | 保留原值 |
| yaw_rate_scale | 0.988 | 保留原工程装配值 |

这些是可编译的初始配置，不是对实物测得的最优参数。实际前推时 vx 应为正，左推时 vy 应为正；双轴同时反向可用 `flow_sign=-1`，绕 Z 安装角不一致则改 `sensor_yaw_rad`。不能只因为位置在底盘中心就推断模块图像轴方向也正确。

只要轮速观测仍有效，它仍会影响融合输出。默认权重不是固定百分比，观测频率和过程噪声也会影响结果；持续打滑、低于门限的偏差不会被保证完全消除。

## 修改范围

修改原有 7 个运行时代码文件：

| 文件 | 必要修改 |
|---|---|
| `src/domain/kalman/chassis_imu_odom.c/.h` | 追加光流输入、两个观测、残差门限及静止判定兼容 |
| `src/service/odom.c/.h` | 在原流程读取光流，调用扩展融合入口，增加只读诊断字段 |
| `src/service/assemble/assemble_odom.c` | 装配 UART7 与中心安装参数 |
| `src/platform/stm32_hal_uart.c/.h` | 在现有回调分发中追加 UART7 |

新增 `src/domain/opticalflow/upixels_flow.c/.h`，复用压缩包解析器，只补实际测距时间及浮点转整数/速度计算的有限值保护；新增 `src/service/opticalflow.c/.h` 管理字节队列、有效性与坐标换算。

未替换为压缩包的六状态融合器，未新增舵轮运动学实现。未修改应用控制、状态机、调度、电机、遥控、BMI088 SDK、通用矩阵/滤波器或 CubeMX 生成源码。本次开始时 `mcu/robot.ioc` 已有用户修改，保留该文件内容不变，SHA-256 为：

```text
250BCEB23DEB9A9545CA76E24789759C6A51E9DEA53EC993D9B637F25E82CE7B
```

## 验证结果与使用限制

主机实际编译并执行了真实 C 解析器、光流服务、九状态融合器、通用滤波器与矩阵实现，测试源为 `tests/opticalflow_fusion_test.c`：

- 29 个集成场景，包括 UPIXELS/EXT/PX4、CRC 恢复、无效质量/时间/高度、测距与样本过期、积压旧帧、时钟环绕、方向/安装角/姿态投影、UART 接收失败及队列溢出恢复。
- 250Hz滤波与50Hz光流只更新一次验证、静止小噪声、光流否决错误零速、残差拒绝、安装偏移原地旋转。
- 连续50,000周期，检查状态/协方差有限、协方差对角线非负及对称性；这不是对所有工况正定性的数学证明。
- 与任务开始时提交 `48367480a1e8eadaf078f10b215055f7dd02391d` 的原六观测实现分别编译，比较无光流情况下2,500周期后的全部9个状态和81个协方差元素，误差均在1e-6内。

一个固定输入演示：轮速报告1m/s、光流报告0.5m/s，运行4秒后融合位置约2.499064m，最后一周期速度约0.670213m/s。这证明两路观测都参与；它不是实物精度结果，也不表示融合器能完全消除轮速偏差。

接收方可用主机 C 编译器编译测试文件与实际模块；兼容性对照还需独立编译上述提交的原 `chassis_imu_odom.c/.h`，重命名导出符号并提供 legacy adapter。交付包的 `verification` 目录保留了本次使用的原始快照、适配器与结果。

实物上仍需核对安装方向、TOF测距、模块实际协议与刷新率。程序没有新增视觉绝对位置、磁力计航向或位置闭环修正；光流与轮速的速度偏差仍会积分为位置误差。地面纹理、光照、反光以及姿态变化会影响光流；本次保留现有二维里程模型，不承诺复杂坡面或强烈旋转下的成像补偿精度。

## 调试读取

原有位置/速度/姿态获取方式不变。可通过调试器读取：

```c
const Odom* state = odom.get_state();
/* state->opticalflow_ready: 样本与高度有效且倾角通过检查 */
/* state->opticalflow_updates: 实际接受的光流帧数 */
/* state->opticalflow_rejects: 被融合门限拒绝的帧数 */

const OpticalflowState* flow = opticalflow_get_state();
/* received_bytes, frames_ok, crc_errors, invalid_samples, uart_errors,
   queue_overflows, height_m, vx_sensor_m_s, vy_sensor_m_s */
```

`opticalflow_ready` 表示输入层可用，不保证本帧通过残差门限；需结合更新/拒绝计数判断。诊断读取不增加周期日志，也不修改现有上位机协议。
