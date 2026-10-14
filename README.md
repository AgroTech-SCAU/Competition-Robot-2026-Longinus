
> [!IMPORTANT]
> ## 仓库初始化
>
> 本仓库由 **AgroTech Repository Template** 创建
>
> 项目负责人 / 仓库管理员首次创建仓库后，请完成以下初始化：
>
> - [ ] 点击绿色按钮 `Code` → `Clone using the web URL.` → `git clone <仓库 URL>`，将仓库克隆到本地
> - [ ] 填写本 README 中的项目基本信息、环境、构建与运行方式
> - [ ] 填写 [`docs/plan.md`](docs/plan.md)，明确当前目标与下一步
> - [ ] 确认默认分支为 `main`
> - [ ] 确认仓库已启用 Issues，并检查 `New issue` 页面可以看到仓库自带的 Issue Forms
> - [ ] **如果为 public 仓库，请导入规则集以保护 main 分支**：进入 `Settings → Rules → Rulesets → New ruleset → Import a ruleset`，导入 [`.github/rulesets/main-protection.json`](.github/rulesets/main-protection.json)，点击 `Create` 并确认规则已作用于 `main`
> - [ ] 推荐：运行本仓库的一键 Git 设置脚本，开启本地防误操作提醒（可跳过；Ruleset 仍会保护 `main`）
>
> Linux / Ubuntu：
>
> ```bash
> bash setup-scripts/setup-git.sh
> ```
>
> Windows PowerShell：
>
> ```powershell
> .\setup-scripts\setup-git.ps1
> ```
>
> GitHub Free Organization 的 Repository Rulesets 对私有仓库的可用性取决于当前套餐；如果仓库设置中没有对应入口，以 GitHub 实际提供的功能为准
>
> **初始化全部完成后，请删除本段“仓库初始化”提示**

# <项目名称>

> 一句话说明项目 / 仓库解决什么问题或维护什么资产

## 当前状态

- **状态：** 开发中
- **最新稳定版本：** 暂无
- **当前开发计划：** [`docs/plan.md`](docs/plan.md)

> 首次形成可复现的稳定版本后，再创建 Git Tag + GitHub Release，并将本 README 更新为该稳定版本的完整使用说明

> [!IMPORTANT]
> ## 参与本项目开发
>
> 推荐流程：
>
> **Issue → Branch → Commit → Push → Pull Request → 项目负责人 Merge**
>
> - 开始开发前，原则上先创建或认领 Issue
> - 从 Issue 的 `Development` 区域创建任务分支并在 git 本地切换分支；或者在 git 本地里从最新 `main` 手动创建分支并推送后，在 Issue 的 `Development` 区域绑定分支
> - 推荐分支名：`feat/xxx`、`fix/xxx`、`refactor/xxx`、`docs/xxx` 等；这是协作约定，不做硬性拦截
> - 请勿直接在 `main` 开发或 Push
> - 如果已经误在 `main` 上产生了有用 Commit，**不要先 `reset --hard`**，先按协作指南把提交保存到新分支
>
> 完整流程与常见问题：[`CONTRIBUTING.md`](.github/CONTRIBUTING.md)

## 1. 项目简介

说明：

- 项目用途
- 主要能力
- 适用场景
- 当前边界 / 不支持的内容

## 2. 环境要求

### 软件

- OS：
- ROS / Runtime：
- Compiler / Python：
- 关键依赖：

### 硬件（如适用）

- 主控：
- 传感器：
- 执行器：
- 接口：

## 3. 安装

```bash
# 示例
```

## 4. 构建（如适用）

```bash
# 示例
```

> 文档、模型、数据等非软件仓库可将本节改为“获取 / 导入 / 使用方式”

## 5. 快速开始

```bash
# 最小可运行示例
```

预期现象 / 结果：

- ...

## 6. 配置说明

| 配置项 | 默认值 | 说明 |
| --- | --- | --- |
| `<param>` | `<value>` | ... |

## 7. 目录结构

```text
.
├── README.md
├── docs/
│   └── plan.md
├── .github/
│   ├── CONTRIBUTING.md
│   ├── pull_request_template.md
│   ├── ISSUE_TEMPLATE/
│   └── rulesets/
├── .githooks/          # 可选的本地防误操作提醒
└── setup-scripts/      # 一键启用本地 Git 设置
```

## 8. 文档

- `docs/plan.md`：项目规划（必须）
- `.github/CONTRIBUTING.md`：成员协作流程与误操作急救
- `docs/architecture.md`：系统架构（如有）
- `docs/interface.md`：接口说明（如有）
- `docs/deployment.md`：部署说明（如有）
- `docs/calibration.md`：标定说明（如有）
- `docs/troubleshooting.md`：故障排查（如有）

## 9. 常见问题

### 问题 1

现象：

原因：

处理：

## 10. 版本与发布

正式稳定版本使用 **Git Tag + GitHub Release** 发布

版本历史见：<Releases 链接>

## 11. 维护者

- Maintainer / 项目负责人：`@GitHub-ID`
