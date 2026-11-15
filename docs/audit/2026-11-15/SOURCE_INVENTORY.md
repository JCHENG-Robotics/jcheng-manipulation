# SOURCE_INVENTORY — 多源工作区与历史来源盘点

> 审计日期：2026-11-15（Asia/Shanghai）
> 审计范围：R9000P 本地、M70q（<M70Q_TAILSCALE_IP>，SSH）、SD 卡原始系统盘、USB 计划盘、本机/远程 Agent 记录
> 审计方式：只读。未修改任何项目文件、未执行 `git add/commit/push`、未打开串口、未向机器人发送任何命令。
> 本文件之外的 4 份交付物：`CHANGE_RECONCILIATION.md`、`PROJECT_STATUS_AUDIT.md`、`MERGE_PLAN.md`、`DECISION_REQUIRED.md`
>
> **执行更新（2026-11-15 晚些时候）**：Board 批准后已执行合并；S17 于 11:34 完成并 hand-off；S04/S05/S10 已移入隔离区。本文件仍保留审计时点快照，执行后的状态见 [MERGE_EXECUTION_REPORT.md](MERGE_EXECUTION_REPORT.md)。

## 0. 一句话结论

共发现 **8 个项目相关工作区/副本**（6 个有实质内容，其中 S17 在审计期间仍在写入；2 个空/残缺）、**2 处 M2.2 证据副本**、**2 个历史备份**、**1 个机器人原始系统盘**、**1 个计划 U 盘**、**4 类 Agent/会话记录**。没有任何一个来源可以被默认为"最新且完整"；唯一主工作区建议为 R9000P 的 `/home/jerichocheng/projects/ros2-industrial-arm-learning`（详见 `MERGE_PLAN.md`）。

## 1. 主机与访问矩阵

| 主机/介质 | 地址/路径 | 本次访问 | 说明 |
|---|---|---|---|
| R9000P（Lenovo Legion R9000P2021H） | 本机，`<R9000P_TAILSCALE_IP>`（Tailscale） | 可访问 | 厂商资料、SD 卡、USB、仿真工作区、本地 Git 克隆都在此 |
| M70q（ThinkCentre M70q） | `<M70Q_TAILSCALE_IP>`，SSH key `<SSH_KEY>` | 可访问（只读命令） | 远程 Git 克隆、M2.2 worktree、Debug Studio、Paperclip |
| M70q GitHub 出口 | `git ls-remote origin` | 可访问 | 可读到 `main`、`agent/JER-18-remediation`、`phase2/week01-execution-reliability` 及两个 tag |
| R9000P GitHub 出口 | `git ls-remote origin` | **超时/不可用** | 本机无法直接验证 GitHub；需经 M70q 中继 |
| 机器人（Raspberry Pi / ROCK 5T） | `<ROBOT_IP>:22`、`<ROBOT_AP_IP>:22`、`<ROBOT_TAILSCALE_IP>:22`（历史命令中出现） | **不可达** | 2026-11-15 探测 22 端口均失败；Tailscale 无机器人节点 |
| SD 卡原始系统盘 | `/media/jerichocheng/rootfs`（52.7G，95% 已用） | 部分可访问 | `/var/lib/docker`、`/root` 为 root-only，当前用户不可读 |
| USB 计划盘 | `/media/jerichocheng/CC98-6092`（29.3G，vfat） | 可访问 | 含 `MASTER_PLAN_V3_PRIVATE.md`、`ROADMAP_PUBLIC.md` |

## 2. 工作区/副本清单

### S01 — R9000P 本地主克隆

| 项 | 值 |
|---|---|
| 路径 | `/home/jerichocheng/projects/ros2-industrial-arm-learning` |
| 类型 | Git 仓库（完整克隆） |
| HEAD | `main` @ `435f8ec1e891bfc22084536e16e4d61bdaa7e5a0`（2026-11-08，Merge JER-17） |
| Tag | `panda-baseline-v1`（annotated `b116577` → `435f8ec`）、`v0.1.0`（`464026f` → `eb08b9b`） |
| 远端 | `https://github.com/JerichoCheng/ros2-industrial-arm-learning.git`（本机不可达） |
| 跟踪文件 | 66 个 |
| 工作区状态 | 2 个已修改跟踪文件 + 1 个未跟踪 YAML + `__pycache__` |
| 大小 | 74M（`.git` 21M） |
| 主要内容 | 8 周 Panda 学习项目：`cpp_warmup/`、`docs/weekly-log/`、`ros2_ws/src/{arm_basics,arm_description,arm_interfaces,arm_moveit_demo}` |
| 独有内容 | 本地 M2.2 运行时验证变体（`gazebo.launch.py` `ee85…`、`simple_arm.xacro` `b3be…`、`arm_controllers_gain_0p2.yaml` `5ceea…`） |
| 角色 | **推荐作为唯一主工作区**（理由见 `MERGE_PLAN.md` §1） |

未提交明细：

```text
 M ros2_ws/src/arm_description/launch/gazebo.launch.py
 M ros2_ws/src/arm_description/urdf/simple_arm.xacro
?? ros2_ws/src/arm_description/config/arm_controllers_gain_0p2.yaml
?? ros2_ws/src/arm_description/launch/__pycache__/
```

### S02 — M70q 远程主仓库

| 项 | 值 |
|---|---|
| 路径 | `/home/jerichocheng/projects/ros2-industrial-arm-learning` |
| 类型 | Git 仓库（与 S01 同一提交的独立克隆） |
| HEAD | `main` @ `435f8ec`（与 S01 相同） |
| Tag | 与 S01 相同；已推送到 GitHub |
| 分支 | `main`、`phase0/armpi-model`（=435f8ec，无独有提交）、`m2-2-v2`（=435f8ec，无独有提交）、`jer-18-execute-result-check`（2bc038b，main 祖先）、`origin/agent/JER-18-remediation`（1bbb136，未合并）、`origin/phase2/week01-execution-reliability`（e596c5a，未合并） |
| 跟踪文件 | 66 个，跟踪内容与 S01 完全一致（除 S01 的 2 个本地修改） |
| 未跟踪 | `armpi-debug-studio/`（约 177M）、`docs/phase0/`（2 份）、`__pycache__/` |
| 大小 | 248M |
| 独有内容 | Debug Studio G1 Mock 切片、`docs/phase0` 两份审计 |
| 角色 | **合并来源**：把 Studio 与 phase0 文档同步到主工作区；之后降级为备份/GitHub 中继 |

### S03 — M70q M2.2 worktree

| 项 | 值 |
|---|---|
| 路径 | `/home/jerichocheng/projects/ros2-industrial-arm-learning-m2_2-v2` |
| 类型 | Git worktree（`m2-2-v2` 分支，`435f8ec`） |
| 状态 | 4 个已修改跟踪文件 + 7 个未跟踪目录/文件组 |
| 独有内容 | Gate 2 rework1 实现（`controller_config.py`、测试、0.1/0.2 gain profile、watchdog/launch 改造）+ `evidence/m2_2_v2/gate2/{20261111T0547Z,20261112T014542Z-rework1}/implementation-report.md` |
| 关键事实 | 分支**没有任何提交**；`m2-2-v2` ref 只在 M70q 本地，GitHub 上没有；所有变更均为未提交状态 |
| 角色 | **M2.2 rework1 的唯一来源**；合并前必须原样保留 |

### S04 — R9000P `Projects_M70q` 空骨架（非工作区）

| 项 | 值 |
|---|---|
| 路径 | `/home/jerichocheng/Projects_M70q/ros2-industrial-arm-learning` |
| 文件数 | **0**（仅目录骨架） |
| `.git` | 188K，缺少 `HEAD`、`config`、`index`；`git` 命令报"不是 Git 仓库" |
| 目录 | `armpi-debug-studio/`、`cpp_warmup/`、`docs/`、`ros2_ws/`、`build/`、`install/`、`log/` 等均为空；`.agents/.aws/.codex/.paperclip` 为 11-04 残留目录 |
| 结论 | **不是可用副本**，不能作为主工作区；不得据此判断"代码丢失"或"文件为空" |

### S05 — R9000P 本地 M2.2 残缺副本

| 项 | 值 |
|---|---|
| 路径 | `/home/jerichocheng/Projects_M70q/ros2-industrial-arm-learning-m2_2-v2` |
| 文件数 | 25（仅 `cpp_warmup/`、`docs/demo.gif`、根目录 README/ROADMAP/LICENSE/.gitignore/CI） |
| `ros2_ws` | 目录存在但为空 |
| `.git` | 文本指针：`gitdir: /home/jerichocheng/projects/.../.git/worktrees/...`，本机不存在该目录 → Git 不可用 |
| 与 S03 关系 | 25 个文件全部与 S03 对应文件哈希相同；**无任何独有内容**，是早期中断的复制 |
| 结论 | 可清理候选（需批准） |

### S06 — R9000P ArmPi 仿真工作区

| 项 | 值 |
|---|---|
| 路径 | `/home/jerichocheng/armpi_sim_ws` |
| 类型 | **非 Git** ROS 2 工作区 |
| 包 | `src/armpi_ultra_description`（ament_python）、`src/armpi_ultra_moveit_config`（ament_cmake） |
| 大小 | 35M |
| 来源 | 从 `ArmPiUltra/3 功能包文件/simulations/` 复制；`armpi_ultra.xacro` 与厂商 A 版逐字节相同；STL 全部相同 |
| 构建 | 2026-11-14 13:52 最后一次 `colcon build` 返回 0；install 中两个包齐全 |
| 运行 | 2026-11-14 13:53–13:58 MoveIt demo 实跑；3 条 arm 轨迹执行 SUCCEEDED（mock hardware） |
| 修改 | `joint_limits.yaml` 仅对 joint1–5 启用了 `max_acceleration: 1.0`；gripper 关节仍未启用 → 日志中 `l_in_joint` 规划失败 |
| 独有内容 | 唯一的 ArmPi 5-DoF MoveIt 仿真配置与运行记录 |
| 角色 | 合并进主仓库；不宜长期留在仓库外 |

### S07 — R9000P 厂商资料与 Debug Studio 文档

| 项 | 值 |
|---|---|
| 路径 | `/home/jerichocheng/ArmPiUltra` |
| 类型 | 非 Git 参考资产 |
| 大小 | **16G**（含 `ROS2/Armpi_ultra-main.zip` 921MB、STM32、PDF、视频教程） |
| 主要内容 | 厂商 ROS 2 源码（`ROS2/Armpi_ultra-main/armpi_ultra`）、`3 功能包文件/simulations/`、`7.URDF建模及仿真/`、`STM32/`、`1.ArmPi Ultra使用入门.pdf` |
| 独有文档 | `ARMPI_DEBUG_STUDIO_PRD_V1.md`（v1.0，2026-11-13）、`docs/g0/`（README + 01–07） |
| 角色 | 厂商只读参考；PRD/G0 需要纳入主仓库 `docs/`；**16G 资产不得进入 Git** |

### S17 — R9000P `Projects_M70q/armpi_ultra_ws_clean`（审计期间新建，进行中）

| 项 | 值 |
|---|---|
| 路径 | `/home/jerichocheng/Projects_M70q/armpi_ultra_ws_clean` |
| 类型 | **非 Git** ROS 2 工作区；由并行会话 `sess_83d860c9` 于 2026-11-15 11:20–11:24 创建/写入 |
| 大小/文件数 | 28M / 248 个文件 / 85 个目录 |
| 包 | `peripherals`、`app`、`bringup`、`armpi_ultra_description`、`ros_robot_controller_msgs`、`servo_controller_msgs`、`ros_robot_controller`、`kinematics_msgs`、`kinematics`、`controller`、`servo_controller`、`sdk`（12 个 package.xml） |
| 独有内容 | `docs/removed_files.tsv`（6.8K，删除/排除清单）、`docs/original/`（6 个原始文件备份）、`tests/test_mecanum_watchdog.py`（5.8K）、对 `kinematics/transform.py`、`controller/mecanum.py`、`ros_robot_controller_node.py`、`bringup/core_check.py` 的修改 |
| 状态 | **仍在写入**（最后一次观察 11:24:05）；未完成、未提交、未验证 |
| 与主仓库关系 | 独立目录；不污染 `/home/jerichocheng/projects/ros2-industrial-arm-learning` |
| 角色 | 厂商精简/重构的进行中工作区；**本轮未做正确性审计**，不能当作已完成成果；待会话结束后作为独立来源审计 |

> 重要：S17 的出现改变了"`Projects_M70q` 完全是空目录"的说法。该目录下的 `ros2-industrial-arm-learning` 仍是 0 文件骨架（S04），但 `armpi_ultra_ws_clean` 已有实质内容。

### S08 / S09 — M2.2 证据副本（两处，内容相同）

| 项 | S08 | S09 |
|---|---|---|
| 路径 | `/home/jerichocheng/ros2_test_data/m2_2` | `/home/jerichocheng/robotics_evidence/m2_2` |
| 主机 | R9000P | M70q |
| 文件数 | 24 | 24 |
| 哈希 | 24 个文件逐一相同 | 24 个文件逐一相同 |
| 差异 | 目录名 `pre_sync_backup/` | 目录名 `config_snapshot/` |
| 内容 | 4 个 mcap（trajectory_test_01 / 1p5s / settling / gain_0p2 settling）、CSV/PNG 分析、`evidence/SHA256SUMS.txt`、源码快照 | 同左 |
| 角色 | M2.2 runtime 证据；S09 是 S08 的 rsync 副本（`~/.bash_history` 有对应 rsync 命令） |

### S10 — R9000P WSL2 迁移构建残留

| 项 | 值 |
|---|---|
| 路径 | `/home/jerichocheng/robotics-migration-backup/old-wsl2-build` |
| 大小 | 175M |
| 内容 | `build/`、`install/`、`log/`（2026-09-14 与 2026-10-10 的构建），**没有源码** |
| 结论 | 仅历史构建产物，无独有知识；可归档 |

### S11 — R9000P 仓库完整备份 tar

| 项 | 值 |
|---|---|
| 路径 | `/home/jerichocheng/Downloads/ros2-industrial-arm-learning-backup.tar.gz` |
| 大小/时间 | 49.5MB，2026-11-10 18:03 |
| 内容 | 完整仓库 + `.git`（899 条 `.git/` 记录）+ build/install/log |
| Git 状态 | `HEAD -> refs/heads/main`，`refs/heads/main = eb08b9b`（v0.1.0），**不是当前 435f8ec** |
| 关键文件哈希 | `gazebo.launch.py` `29f6…`、`simple_arm.xacro` `8de7…`（= 当前 HEAD 版本）、无 `gain_0p2` |
| 结论 | v0.1.0 时点的可回滚快照；不包含 JER-17/18、M2.2、Studio |

### S12 — SD 卡原始机器人系统盘

| 项 | 值 |
|---|---|
| 挂载点 | `/media/jerichocheng/rootfs`（`/dev/sda2`，52.7G ext4，95% 已用） |
| 系统 | Raspberry Pi OS，内核 6.6.20+rpt-rpi-2712 / v8，5G swapfile |
| 可见内容 | `/home/pi/armpi_ultra`（厂商 GUI/tool/wifi_tool）、`hiwonder-toolbox`、`.zcode`（3.18.2 arm64 server）、`.copilot` |
| **未发现** | `ros2_ws`、项目源码、用户自己的 ArmPi 工作区 |
| 不可读 | `/var/lib/docker`（`drwx--x--- root:root`）、`/root`（`drwx------ root:root`） |
| 结论 | 唯一真实机器人系统；但**Docker 容器内的 `/home/ubuntu/ros2_ws` 无法审计**；本次未写入任何内容 |

### S13 — USB 计划盘

| 项 | 值 |
|---|---|
| 挂载点 | `/media/jerichocheng/CC98-6092`（`/dev/sdb1`，29.3G vfat） |
| 计划文件 | `MASTER_PLAN_V3_PRIVATE.md`（18,608B，10-26）、`ROADMAP_PUBLIC.md`（4,656B，10-26） |
| 其他 | `rjx/`（K27、NeckVis-Depth、OpenaArmmini 等另一项目）、`wallpaper/` |
| 结论 | **计划文件的唯一可读副本**；必须保留，不得当作公开仓库内容 |

## 3. Git 仓库状态总表

| 仓库 | 分支 | HEAD | 跟踪文件 | 脏文件 | Tag | 远端可推送 |
|---|---|---|---|---:|---|---|
| S01 R9000P | main | 435f8ec | 66 | 2 M + 1 ?? + pycache | panda-baseline-v1, v0.1.0 | 本机不可达 |
| S02 M70q | main | 435f8ec | 66 | 0 M + 3 ?? | 同上 | 可（GitHub 可达） |
| S03 M70q worktree | m2-2-v2 | 435f8ec | 66 | 4 M + 多条 ?? | 无 | 分支未推送 |

### 3.1 未合并分支（重要）

| 分支 | 提交 | 与 main 关系 | 内容 | 状态 |
|---|---|---|---|---|
| `origin/phase2/week01-execution-reliability` | `e596c5a` | 基于 `eb08b9b`，与 main 分叉；4 个独有提交 | Phase II Week 1：pick/place 参数化、launch 覆盖、typed action readiness gate、失败退出、CHANGELOG、`docs/evolution/phase2/week01.md`、`docs/learning-roadmap.md`；10 文件 483+/161- | **未合并**；分支文档自述"runtime regression remains to be executed" |
| `origin/agent/JER-18-remediation` / 本地 `review/v0.4-pick-place-reliability` | `1bbb136` | 基于 `eb08b9b`；2 个独有提交 | 与 phase2 的 `a2c51bf`/`b73f804` 同源的 gripper readiness + 参数化；6 文件 189+/38- | **未合并**；是 phase2 代码的子集 |
| `phase0/armpi-model` | `435f8ec` | 与 main 相同 | 无独有提交 | 名义分支 |
| `m2-2-v2` | `435f8ec` | 与 main 相同 | 无提交；变更都在 S03 工作区 | 仅 M70q 本地 |

> 结论：仓库当前 `main` 只包含 JER-17 的 13 行"执行失败即停"修复；Phase II Week 1 的完整可靠性工作（含文档）停留在未合并分支，**没有进入 `panda-baseline-v1` 冻结基线**。

## 4. Agent / 会话记录

| 来源 | 位置 | 可读内容 | 价值/限制 |
|---|---|---|---|
| ZCode 会话 `sess_0c557294…` | 本机 ZCode | **本审计会话** | — |
| ZCode 会话 `sess_83d860c9…` | 本机 ZCode | "ArmPi Ultra ROS 2 工程精简与重构任务"；状态 **running**；工作区 `/home/jerichocheng/Projects_M70q`；已发现 SD、USB、厂商源码，尚未落盘修改 | **并行会话，未审计其未来输出**；合并前必须确认其状态 |
| ZCode 会话 `sess_acb3142a…` | 本机 ZCode | "恢复 ArmPi Ultra 调试项目上下文"；先出只读恢复报告，用户回复"就按照你说的开始做"后实现 Studio G1；最后更新 2026-11-14 14:27 | Studio G1 的直接实现记录 |
| ZCode 会话 `sess_3295b3fe…` | 本机 ZCode | "ArmPi Debug Studio 全栈应用开发规划"；只写 `ArmPiUltra/docs/g0`，明确"只做分析与设计" | G0 文档来源 |
| 项目 memory | `~/.zcode/cli/memories/projects/*/memory/` | Studio 硬件约束、无 Git 红线、阶段流程、环境分裂 | 部分内容已过期（如"无应用代码"写于 11-14，早于 Studio 落盘） |
| 模型 IO rollout | `~/.zcode/cli/rollout/*.jsonl` | 上述会话的模型输入输出 | 可用于追溯；体量大 |
| Shell history | R9000P `~/.bash_history`；M70q `~/.bash_history` | M2.2 Gazebo 运行、rsync 证据、Studio uvicorn/npm、机器人直连尝试 | **强运行证据**；无时间戳 |
| Paperclip | M70q `~/.paperclip/instances/default/db`（embedded Postgres，端口 54329） | 任务/项目数据库；未安装 `psql`，本次未查询 | 可能含 JER-* 任务记录；未审计 |
| Codex / Claude / OpenHands | M70q `~/.codex/sessions`（556K）、`~/.claude/sessions`（空）、本机 `.copilot`/`.openhands` | Codex 5 个 11-03/04 会话，grep 无 arm/ros2 命中 | 无项目实质记录 |

## 5. 访问限制（明确未检查）

1. **SD 卡 Docker 数据**：`/media/jerichocheng/rootfs/var/lib/docker` 为 root-only；本机无 Docker CLI；容器内 `/home/ubuntu/ros2_ws` 未审计。这是"厂商真实运行环境"的最大盲区。
2. **SD 卡 `/root`**：root-only，未读。
3. **机器人 SSH**：`<ROBOT_IP>`、`<ROBOT_AP_IP>`、`<ROBOT_TAILSCALE_IP>` 的 22 端口均不可达；Tailscale 中无机器人节点。因此**没有任何实机状态证据**。
4. **R9000P → GitHub**：`git ls-remote` 超时；无法在本机验证远端最新状态；M70q 可访问。
5. **Paperclip Postgres**：无客户端，未查询任务表。
6. **其他 AI 会话**：未假定可访问；仅依据上表实际可读记录。
7. **并行会话 83d860c9 的未来输出**：审计期间已在 `Projects_M70q/armpi_ultra_ws_clean` 落盘 28M/248 文件，但会话仍在 running；本轮未做正确性审计，合并前必须等它结束并重新盘点。

## 6. 来源关系图

```text
                    GitHub (JerichoCheng/ros2-industrial-arm-learning)
                      │  main 435f8ec, tags, phase2, JER-18
          ┌───────────┴───────────┐
          │                       │
   S01 R9000P clone         S02 M70q clone
   main 435f8ec             main 435f8ec
   + M2.2 本地变体           + Studio G1 (untracked)
   + 本地实测 bags           + docs/phase0 (untracked)
          │                       │
          │                       ├── S03 M2.2 worktree (m2-2-v2, 未提交)
          │                       └── S09 robotics_evidence/m2_2 (副本)
          │
          ├── S06 armpi_sim_ws (非 Git, ArmPi MoveIt 仿真)
          ├── S07 ArmPiUltra (厂商 16G + PRD + G0)
          ├── S08 ros2_test_data/m2_2 (M2.2 证据原件)
          ├── S10 WSL2 build 残留
          ├── S11 v0.1.0 全仓库 tar 备份
          ├── S12 SD 卡 rootfs (真实机器人，Docker 不可读)
   └── S13 USB 计划盘 (MASTER_PLAN / ROADMAP_PUBLIC)

   S17 Projects_M70q/armpi_ultra_ws_clean (并行会话 11-15 新建，进行中)
          └── 厂商精简/重构；未纳入本轮文件级比较

   Agent: S14 ZCode 4 sessions + memories + rollouts + shell history
          S15 Paperclip Postgres (未查询)
          S16 Codex/Claude/OpenHands (无项目内容)
```

## 7. 对本轮结论的直接影响

- **不能**把 S04 `Projects_M70q/ros2-industrial-arm-learning` 当作"本地主副本"——它是 0 文件骨架。
- **不能**把整个 `Projects_M70q` 当作空目录——其下 `armpi_ultra_ws_clean`（S17）是并行会话正在写入的厂商精简工作区，未完成、未审计。
- **不能**把 S05 当作 M2.2 备份——它没有 `ros2_ws` 内容。
- **不能**把 `main` 的 `panda-baseline-v1` 当作"包含全部 Panda 可靠性工作"——Phase II 分支未合并。
- **不能**把 Studio 的 G0 文档当作"Studio 未实现"——G1 Mock 代码已在 S02 落盘，但未提交、未记录。
- **不能**把 M2.2 rework1 当作"已运行时验证"——bags 来自 11-10 的旧代码，rework1 是 11-12 的离线改造。
- 主工作区、合并顺序与风险见 `MERGE_PLAN.md`；需要用户裁决的事项见 `DECISION_REQUIRED.md`。
