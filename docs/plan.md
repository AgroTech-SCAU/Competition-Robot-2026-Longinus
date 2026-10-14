# Longinus MCU 底盘与 Standard SDK 实工程验证计划

> 目标文件 `docs/plan.md`
>
> 执行原则 以 Longinus 真实工程验证 Standard 的当前 `main`，每次测试固定实际 Commit，公共问题回到 Standard 修复，不以预先发布 Release 作为开工条件

# 1 本次唯一目标

以 Atlas 的 STM32H723 四舵轮底盘为基线创建 Longinus MCU 工程，通过 Git Submodule 集成 Standard 的 `main`，完成 DM 转向、DJI M2006/C610 驱动、FS-iA10B 遥控及失联安全停止的真机闭环，并将集成暴露的通用 SDK 问题反馈修复到 Standard

# 2 本次明确不做

- 不开发机械臂、吸盘、疏果机构、Pi、PC、ASRPro、ROS2、视觉或自动比赛任务
- 不新增上位机通信协议、独立 Chip SDK 或其他未用硬件抽象
- 不重写 Atlas 已验证的底盘运动学、遥控手感和 S 曲线策略
- 不把 Longinus 的 CAN ID、安装方向、机械参数和比赛逻辑移入 Standard
- 不在 Longinus 复制维护公共 `device / domain / infra` 的第二套源码
- 不为当前目标新增 `tests/`、CHANGELOG 或多余说明文档
- 不要求 Standard 提前发布特定版本才能开始 Longinus
- 不将未验证的 Standard `main` 更新自动应用到每次构建

# 3 当前已知基线

- Atlas 正式仓库为 `AgroTech-SCAU/Competition-Robot-2026-Atlas`
- Atlas MCU 源工程位于 `Atlas/chassis_control_code/`，包含 STM32H723 CubeMX、EIDE、linker、app、service、platform 和旧公共 SDK
- Atlas 四舵轮逻辑轮序为 `FL FR RR RL`，转向当前采用 DM、驱动采用 DJI M2006/C610，FDCAN1 连接转向侧、FDCAN2 连接驱动侧
- Atlas 已存在遥控 `vx / vy / wz`、速度档、steer-then-drive、等效角、驱动反向、S 曲线、yaw hold 和驻车流程
- Standard `main` 已收敛为 `sdks/device`、`sdks/domain`、`sdks/infra`
- Standard 的 DM、DJI M2006/C610、FS-iA10B 和 steer_wheel_kine 在模块状态文档中标记为 Stable，但不代表 Longinus 整机已验收
- Standard 目前公开 Release 列表尚未显示目标中的 `v0.2.0`
- 计划名称对应的 Longinus 远端仓库目前尚未查询到，需创建后才能实施
- Standard 的公共 bus_motor 使用逻辑电机 ID、Profile、Command 和 Feedback，厂家绑定在 `service/assemble`
- 新 SDK 的 `stop` 和 `brake` 语义不保证等价，部分模式可能返回 `MOTOR_STATUS_UNSUPPORTED`

上述是仓库检查结论，不代表已经在 Longinus 进行过编译或真机验证

# 4 完成标准

- [ ] Longinus 基于 AgroTech Repository Template 创建，MCU 工程统一放在 `mcu/`
- [ ] Longinus 不包含首版无关的 Atlas 机械臂、吸盘、PC、Pi、ASRPro 业务
- [ ] Standard 以真正的 Git Submodule 引入，跟踪 `main` 用于人工更新，每个通过验证的 Longinus 检查点都指向确定的 SDK Commit
- [ ] Longinus 编译时只使用 Standard 中所需的正式 SDK 模块，无重复符号或私有公共 SDK 副本
- [ ] 四个 DM 转向与四个 DJI 驱动在正确 CAN 总线上完成初始化、命令下发及反馈读取
- [ ] Atlas 必要手动底盘行为保留且可运行
- [ ] 遥控失联、异常反馈、初始化失败和停止请求有真实可达的安全处理路径
- [ ] 低速真机完成 `vx / vy / wz`、速度档、steer-then-drive、brake / stop 和遥控断联验收
- [ ] 所有阻塞性公共 SDK 修复都进入 Standard，Longinus 最终指向无未提交修改的 SDK Commit

# 5 最低成本验证路径

1. 静态目录、依赖来源、接口、轮序和 CAN 映射检查
2. 使用原有 EIDE / MCU 工具链构建和链接
3. 使用架空底盘或台架验证单模块命令与反馈
4. 低速整车验证运动方向、控制模式及安全停止

不因 SDK 标记 Stable 而省略真实设备 Gate，也不为首版增加长稳、极限速度或无关算法实验

# 6 总执行 Prompt

以下内容必须附加到每个步骤的 Codex Prompt 前面

```text
你正在修改 AgroTech-SCAU/Competition-Robot-2026-Longinus
上游基线为 AgroTech-SCAU/Competition-Robot-2026-Atlas 的 Atlas/chassis_control_code
公共 SDK 为 AgroTech-SCAU/Embedded-Electronic-Control-Standard

当前唯一目标是实现 Longinus 的 STM32H723 四舵轮遥控底盘，同时以真实工程验证 Standard SDK
Standard 在开发阶段跟踪 main，但每个完成检查点必须固定并记录实际 SDK Commit
不得因为 main 更新就隐式切换 Longinus 依赖
不得为了等待 Release 阻塞 Longinus 开发

只做当前步骤必要工作，不开发机械臂 吸盘 Pi PC ASRPro ROS2 视觉 自动任务或比赛疏果机构

开始修改前读取本步骤直接相关的 Atlas Longinus Standard 源码 构建配置和必要文档
保留现有项目代码风格 命名 注释 日志 错误处理和目录组织
优先复用已有接口，代码阅读与定位不是独立实施步骤
禁止顺手重构邻近模块和添加不必要的通用抽象

Longinus 只拥有 app service service/assemble platform CubeMX 和实际装机参数
通用 device domain infra 优先引用 Standard 的 sdks 正式源码
禁止复制 SDK 到 Longinus 维护第二份实现
新旧 SDK 迁移时允许旧模块短暂保留作为尚未迁移的依赖
但同一模块只允许有一个参与最终链接的实现，替换验证完成后删除旧副本

公共 SDK 有问题时先定位并保留最小复现证据
若修复目标是通用协议 控制语义 驱动或设备解析，在 Standard 仓库修改和验证
若修复目标是 CAN ID GPIO 电机方向 机械尺寸 零位 遥控手势及整车安全策略，在 Longinus 修改
允许本地临时未提交代码用于迭代排错，但不能作为已完成或可交付的 SDK 基线
只有经负责人确认的 Standard Commit 才能作为 Longinus 的最终依赖指针

项目级停止策略必须明确区分 stop brake disable 和真正的硬件急停
遇到 UNSUPPORTED TIMEOUT NO_FEEDBACK 或 PORT_ERROR 等状态不得当作成功
不得以屏蔽超时 故障 失联保护来通过真机测试

不主动更新 README CHANGELOG 架构文档 其他计划或与当前步骤无关的文档
仅允许必要源码 配置 工程文件和本计划进度发生变化

不得自行新建或切换工作分支
不得自行执行 git commit 或 git push
SDK 合入和 Longinus gitlink 指针提交由负责人另行确认执行

自然语言说明 新增 Markdown 和汇报中禁止使用中文句号
自然语言段落末尾不添加标点
上述限制不适用于 C C++ Python shell YAML JSON 和配置文件语法，也不得破坏原有代码注释规范

验证从足以证明当前目标的最低成本证据开始
没有执行过的编译与测试不得写为 PASS
没有真实设备结果不得写真机 Gate PASS

完成后只汇报修改文件与内容 实际验证命令和结果 未验证项目及剩余阻塞问题
重要驱动与安全路径修改完成后，如 requesting-code-review 可用则调用进行审查
任何步骤标记完成前，如 verification-before-completion 可用则调用核对新鲜证据
发现不阻塞的后续需求只记入 Later Optional
阻塞问题只增量调整受影响步骤，不重写已完成计划
```

# 7 实施步骤

## 步骤 1 `[ ]` 创建 Longinus 并抽离 Atlas MCU 可编译基线

**前置条件**

Longinus 远端仓库由负责人基于 AgroTech Repository Template 创建

**目标**

先得到可追溯、可比较且尚未改动原有底盘行为的 Longinus `mcu/`

**修改范围**

- 仓库模板文件
- `mcu/` 及其 EIDE CubeMX linker 和构建必需资源
- `README.md` 只补充首版必要定位

**必做**

- 从 Atlas `Atlas/chassis_control_code/` 抽取 MCU 工程到 `mcu/`
- 保留启动 调度 CAN 通信 遥控和底盘所依赖的现有文件
- 仅修复迁移引起的路径或构建引用
- 不带入 Atlas 独立 Pi PC ASRPro 目录 模型和比赛资产
- 记录源 Atlas Commit 以便回归对比

**禁止**

- 不重写 chassis 和 remote
- 不在本步骤切换 Standard SDK
- 不提前大量删除旧业务

**最小验证**

- [ ] 仓库模板基础文件齐全
- [ ] MCU 目录和工具链引用完整
- [ ] 抽离后的 MCU 与 Atlas 源工程可达到同级别构建状态

**Codex Prompt**

```text
读取 Repository Template 和 Atlas/chassis_control_code 的入口 README 构建配置与目录
只完成 MCU 基线抽离和必要路径调整
记录 Atlas 源 Commit
确保首次构建可复核
不要清理大量业务，也不要接入 Standard
```

## 步骤 2 `[ ]` 收敛 Atlas 为纯遥控底盘 MCU

**前置条件**

步骤 1 已完成且构建基线已记录

**目标**

移除首版无关业务，同时保持遥控到底盘及失联停止路径可达

**修改范围**

- `mcu/src/app/`
- `mcu/src/service/` 和 `service/assemble/`
- `mcu/src/platform/` 与确实无用的 CubeMX 外设配置
- 直接受影响的构建文件

**必做**

- 删除 arm suction pc_comms pi_comms asr_comms 与对应 app 状态及 assemble 路径
- 去掉未使用的外设绑定，但保留现有底盘、遥控、IMU 与时基真实依赖
- 保留 FS-iA10B 的 `vx / vy / wz` 映射、SWB 速度档、SWC 控制、等效角、S 曲线、yaw hold 和必要 brake / stop
- 将状态机简化为 Idle Manual Fault 或维持等价的安全状态集合
- 上电、异常及断联不能无条件使能驱动或复用旧运动命令

**禁止**

- 不修改车轮 IK FK 或厂家驱动实现
- 不接入 Standard 新电机 API
- 不为未来功能保留不可达占位业务

**最小验证**

- [ ] MCU 构建通过
- [ ] `remote -> app -> chassis` 控制链静态成立
- [ ] 失联和故障路径静态到达安全停止
- [ ] 无仍可达的非底盘业务

**Codex Prompt**

```text
读取 Longinus 当前 app remote chassis entry 和 CubeMX
只删除 Longinus 首版不用的 Atlas 业务和依赖
尽量原样保留手动控制与失联保护的实际逻辑
完成后编译并检查遥控输入到驱动控制及失联停止的完整调用链
不要迁移 Standard 驱动
```

## 步骤 3 `[ ]` 引入跟踪 main 的 Standard Submodule 并逐模块去重

**前置条件**

步骤 2 的 MCU 构建和静态安全链通过

**目标**

让 Longinus 构建直接使用 Standard 正式公共源码，同时确保依赖 Commit 可复现

**修改范围**

- `.gitmodules`
- `external/Embedded-Electronic-Control-Standard/` 子模块指针
- `mcu/` 构建 source 和 include 配置
- 已完成迁移的本地重复 `device / domain / infra`
- 与公共模块初始化相关的 `service/assemble`、platform、remote

**必做**

- 添加 Standard 子模块并在 `.gitmodules` 中设置 `branch = main`
- 首次记录 `git submodule status` 显示的确切 Commit
- 每次人工升级时显式获取 `origin/main`，先看差异，再更新 Longinus 子模块指针
- 只引入当前需要的 `fs_ia10b`、`steer_wheel_kine`、`imu`、`rgb_led`、`delay`、`log` 等正式源码
- 逐模块迁移 PortOps 和实例初始化，迁完一个模块就移除其旧文件的构建入口及副本
- 旧 `agv_motor` 和相关电机依赖可以暂留到步骤 4，但不得同时链接相同公共符号的两套 bus_motor
- 对照 Standard 当前坐标约定 `+x 前 +y 左 +wz 逆时针` 和 `FL FR RR RL`，确认 Longinus 的变换边界

**禁止**

- 不将整个 Standard 目录全部编译
- 不要求先发布 Tag 或 Release
- 不把 `main` 解释为每次构建自动拉取最新代码
- 不在未替换旧驱动前贸然删除它们导致中间状态无法构建
- 不允许脏子模块成为已完成检查点

**最小验证**

- [ ] `.gitmodules` 指明 Standard 和跟踪分支 main
- [ ] `git submodule status` 可复核 SDK Commit
- [ ] 已迁模块的正式源码路径全部来自 Standard
- [ ] 编译无重复符号 缺失源文件或 include 污染
- [ ] 遥控在线判断和运动学坐标语义经静态检查符合预期

**Codex Prompt**

```text
先读取 Standard sdks/README.md module-status 架构和 Longinus 现有公共模块
在 Longinus 中添加指向 Standard main 的真实子模块
记录当前 SDK Commit
逐模块切换构建路径和项目级 PortOps 绑定
每迁完一个模块移除其旧副本并检查编译
旧电机相关模块留给步骤 4 统一替换
不得产生两份同名公共符号
不修改 Standard 源码实现
```

## 步骤 4 `[ ]` 将底盘驱动迁移到 Standard bus_motor 并打通安全路径

**前置条件**

步骤 3 完成，当前 SDK Commit 可确定

**目标**

用 Standard 的 DM 和 DJI 实现替代 Atlas 旧 `agv_motor` 接口，维持现有底盘功能与必要安全行为

**修改范围**

- `mcu/src/service/chassis.*`
- `mcu/src/service/assemble/assemble_chassis.c`
- 必要的最小项目级电机适配边界
- `mcu/src/platform/stm32_hal_can.*`
- 构建配置和未迁完的旧 bus_motor 源码

**必做**

- 先评估直接使用 Standard `bus_motor` 是否足够，只有旧 chassis 接口兼容确实需要时才建立最薄的项目级适配层
- 在 assemble 中绑定四个逻辑转向电机到 DM、四个逻辑驱动电机到 DJI M2006/C610
- 保留 CAN1 转向和 CAN2 驱动的真实分配，并验证 ID、主机 ID、反馈路由与轮序
- DM 先配置驱动所需 PortOps RX 回调，保证模式切换 ACK 可处理，再进行 Profile 激活和使能
- DJI 按新 `DjiMotorConfig` 确定反馈超时、控制周期、限幅和必要 PID，周期性调用所要求的 update 与组帧发送路径
- `chassis` 不依赖特定 DM DJI CAN 句柄和协议帧，厂家特有配置只留在 assemble
- 保留 Atlas 的绝对舵角选取、正反转、S 曲线、steer-then-drive 和驻车策略
- 明确处理 SDK 的 `MOTOR_STATUS_UNSUPPORTED`、`NO_FEEDBACK`、`TIMEOUT` 和 `PORT_ERROR`
- 评估新 DM brake 和 DJI stop 在真实工作 Profile 下的语义，不能假设直接替换旧接口即保持原有安全效果
- 发现属于 SDK 的阻塞问题时在 Standard 本体修复，经 Longinus 复测后使用通过验证的 SDK Commit
- 替换完成后删除旧 `agv_motor` 及重复电机 SDK 副本

**禁止**

- 不重构轮胎运动学算法和遥控映射
- 不为单辆车修改 Standard 公共 API
- 不绕过 ACK 反馈保护或 timeout 来获取表面成功

**最小验证**

- [ ] MCU 构建和链接通过
- [ ] 电机逻辑 ID 和物理 ID 一一对应
- [ ] CAN RX 到统一反馈的路径正确
- [ ] 底盘旧公共电机实现不再参与最终构建
- [ ] 架空或台架条件能获取四转向四驱动反馈，相关启停路径实际执行成功

**Codex Prompt**

```text
读取 Longinus chassis assemble_chassis platform CAN 和 Standard bus_motor DM DJI public headers
最小化替换电机服务边界
明确初始化 PortOps 实例数组 RX ACK Profile Group 和周期 update 路径
仅在必要时定义项目级薄适配层
保留已有的轮序 坐标变换 等效角 S 曲线 和遥控语义
特别检查 brake 与 stop 的支持范围 不以返回 UNSUPPORTED 冒充停车成功
先构建 再做最低风险的反馈和启停台架验证
公共 SDK bug 只在 Standard 本体修复并重新集成验证
```

## 步骤 5 `[ ]` FS-iA10B 低速整机验收与 SDK 基线冻结

**前置条件**

步骤 1 至 4 完成，机械急停或可靠断电手段可用，车辆已采取防跑车措施

**目标**

用 Longinus 真机同时验收底盘控制能力与当前 Standard SDK 的实际可用性

**修改范围**

- Longinus 当前项目的必要电机配置、安全 service 和 assemble
- 若确认为通用 SDK 缺陷，仅在 Standard 修复并重新更新子模块引用
- 最终只更新本计划的验证状态和必要备注

**必做**

- 先架空确认四个 DM 转向和四个 DJI 驱动的轮位 编号 方向 正常反馈和停止行为
- 再在低速和可控场地确认 `vx` 前后、`vy` 横移、`wz` 自转和组合运动
- 验证 SWB 速度档、SWC steer-then-drive、brake 和 stop
- 关闭遥控或模拟断联，确认在定义超时内进入安全停止且不会自动恢复旧运动目标
- 检查电机反馈超时、状态错误、初始化失败下的安全行为
- 验证 SDK 的实际 Commit 与构建来源，子模块和 Longinus 不得依赖未提交的 SDK 修改
- 通用驱动问题反复按照 `Longinus 复现 -> Standard 修复 -> Longinus 更新 Commit -> 同场景复测` 进行，直到无阻塞问题
- 完成后由负责人决定是否将经验证的 Standard Commit 发布成 Tag / Release，此动作不是验收的前置条件

**禁止**

- 不在验收阶段新增自动比赛功能
- 不关闭超时和故障保护来满足验收
- 不扩展到长稳、极限速度或整机性能优化

**最小验证**

- [ ] 四个转向和驱动的反馈与方向正确
- [ ] 三个运动自由度及组合运动正确
- [ ] 速度档、steer-then-drive 与原底盘必要行为正常
- [ ] brake stop 实际达到定义的安全运动效果
- [ ] 失联安全停止和重新使能条件明确通过
- [ ] 固定 SDK Commit 后的完整 MCU 构建和低速控制复测通过
- [ ] 没有阻塞交付的未解决公共 SDK 故障

**Codex Prompt**

```text
本步骤只做 Longinus 四舵轮遥控真机 Gate
确认急停条件后按单模块到整机 低速到组合运动顺序测试
记录实际 SDK Commit 设备条件 控制命令和真实结果
检查遥控断联 故障停止和恢复行为
Longinus 配置问题只改 Longinus
Standard 公共驱动问题只在 Standard 修复 经相同场景复测后更新子模块指针
不新增比赛功能 不做无关性能实验
没有真机证据不可标记 PASS
```

# 8 Standard main 与 Longinus Submodule 的协作规则

## 第一次引入

```bash
git submodule add -b main https://github.com/AgroTech-SCAU/Embedded-Electronic-Control-Standard.git external/Embedded-Electronic-Control-Standard
git submodule status
git -C external/Embedded-Electronic-Control-Standard rev-parse HEAD
```

`branch = main` 用于告诉 Git 人工执行远端更新时选择哪条分支

父仓库实际记录的是子模块的确定 Commit，即使上游 main 前进，旧 Longinus 检查点仍可检出此前提交过的 SDK 内容

## 从 Standard main 更新 Longinus 依赖

```bash
git submodule update --init --recursive
git -C external/Embedded-Electronic-Control-Standard status --short
git submodule update --remote --checkout external/Embedded-Electronic-Control-Standard
git diff --submodule=log
git submodule status
```

只有子模块工作区干净时才执行这种覆盖检出的更新流程

确认差异、构建和测试通过后，再由负责人在 Longinus 中记录新的子模块指针

## 开发期间发现 SDK 问题

1. Longinus 记录报错、设备型号、调用路径、最小复现条件和使用的 SDK Commit
2. 在 Standard 仓库修复公共代码，可以使用独立工作区开发，或在子模块自身 Git 工作区临时迭代
3. 临时未提交修改可以用于排错，但不能当作可复现的完成状态
4. 由负责人将通过检查的 Standard 修改提交并纳入上游 main
5. Longinus 显式更新到修复后的 Standard Commit，使用原场景重新验证
6. 负责人再分别提交 Standard 和 Longinus 的引用变化，禁止让 Codex 自动提交或推送

所有公共改动的所有权始终属于 Standard，Longinus 只记录项目配置和所依赖的 Commit

# 9 最终验收

- [ ] Longinus 工程能稳定构建并正常遥控四舵轮底盘
- [ ] Standard 公共模块由单一源码来源提供
- [ ] SDK Commit 可复现且没有未提交的依赖修改
- [ ] 真机运动与安全 Gate 均有实际证据
- [ ] 新增需求没有扩展到当前明确不做的功能

# 10 后续可选

- Standard 发布经过 Longinus 真机验证的 Tag 和 Release
- 长时间运行与高速度性能优化
- 疏果执行器、Pi / ROS2 和自动任务
- 更多板卡与电机组合的专项验证
- 独立测试目录或 CI 建设

# 11 当前立即执行

使用 AgroTech Repository Template 创建 `Competition-Robot-2026-Longinus` 仓库，然后执行步骤 1，先取得可编译的 Atlas MCU 原样抽离基线

# 12 进度统计

| 步骤 | 任务 | 状态 |
|---:|---|:---:|
| 1 | 创建 Longinus 并抽离可编译 MCU | [ ] |
| 2 | 删除非底盘业务 | [ ] |
| 3 | 引入 Standard main 子模块与公共模块迁移 | [ ] |
| 4 | 迁移 DM DJI 到新版 bus_motor | [ ] |
| 5 | 遥控真机验收和 SDK Commit 冻结 | [ ] |
| 最终验收 | Longinus MCU Baseline | [ ] |
