# DECISION_REQUIRED — 需要 Board 亲自裁决的事项

> 本文件只列**不能由 Agent 自动决定**的问题。每项给出选项、建议、影响与阻塞关系。
> 未获得明确答复前，不执行合并、覆盖、清理或删除。

## 0. 最小批准集（如果只想先推进）

如果希望尽快开始低风险合并，建议至少先批准：

- **D-01**（M2.2 版本策略）
- **D-02**（Studio 归属）
- **D-04**（ArmPi 模型修正范围）
- **D-06**（硬件/容器访问；可先只批准"导出容器工作区"）

其余（D-03、D-05、D-07～D-12）可以在对应阶段前再答。

---

## D-01 M2.2 采用哪条实现线？

**背景：**
- S01 本地未提交版本（`ee85…`/`b3be…`）是 2026-11-10 四个 mcap 的真实源码，**有运行时证据**，但路径硬编码、无校验/看门狗。
- S03 rework1（`61cb…`/`a4f8…`）工程化更好（可移植、schema 校验、watchdog、离线测试），但**没有 Gazebo/ROS 运行时证据**。
- 两者不是简单的新旧关系，而是"实测过的旧版"vs"更严格但未跑过的新版"。

| 选项 | 内容 | 优点 | 缺点 |
|---|---|---|---|
| **A（推荐）** | 先把 S01 实测版固定在分支 `m2.2/gazebo-validated-20261110`；main 采用 S03 rework1；在 R9000P 重跑 Gazebo 回归后再合并 | 保留证据链；最终代码更可维护 | 需要 1–2 天回归 |
| B | 保留 S01 实测版为 main，不采用 rework1 | 运行时已验证 | 放弃可移植性/校验/看门狗；技术债 |
| C | 手工合并两者（rework1 结构 + S01 行为） | 可能兼得 | 冲突面大、需重新测试，风险最高 |

**建议：A。** 如果时间紧，可先执行 A 的"固定分支"部分，再决定 rework1 是否进 main。

**阻塞：** MERGE_PLAN Phase C。

---

## D-02 ArmPi Debug Studio 放在哪里、是否现在提交？

**背景：** Studio G1 Mock 已在 S02（M70q）落盘，未跟踪、未提交、未进 GitHub；177M（含 `.venv`/`node_modules`/`dist`）。PRD §13 建议独立模块；Q-01 原本建议新建独立仓库。

| 选项 | 内容 | 优点 | 缺点 |
|---|---|---|---|
| **A（推荐）** | 作为主仓库子目录 `armpi-debug-studio/`，只提交源码/配置/文档，依赖与 `data/` 排除 | 与主项目同仓、便于引用 G0/PRD；不需要第二个仓库 | 仓库变大（源码约 7k 行，可接受） |
| B | 新建独立仓库 `armpi-debug-studio` | 边界清晰、可单独发布 | 多一个仓库/同步成本；与主项目证据链分离 |
| C | 暂不提交，保留在 M70q | 零风险 | 177M 单点；随时可能丢失；无法追溯 |

**建议：A。** 提交范围仅 `backend/`、`frontend/src/`、`configs/`、`docs/`、`README.md`、`.gitignore`、`pyproject.toml`、`package.json`；排除 `.venv/`、`node_modules/`、`dist/`、`data/`、`*.egg-info/`、`*.tsbuildinfo`。
**提交动作由 Board 执行**，Agent 只准备文件与建议 commit message。

**阻塞：** MERGE_PLAN Phase B。

---

## D-03 Studio 的 "G1" 如何验收？

**背景：** Studio README 称 "G1 Mock-first vertical slice"；PRD §14 的 G1 验收要求"有人看护的实机验收通过"。当前没有 E7 gateway、没有实机、没有 Playwright 文件。

| 选项 | 内容 | 适用 |
|---|---|---|
| **A（推荐）** | 明确改称 **"G1-Mock 切片"**，不宣称 G1 通过；先补 E7 只读 gateway 与 Playwright 文件，再谈 G1 | 诚实、可推进 |
| B | 要求 E7 + Playwright + 实机验收全部完成才允许称 G1 | 严格，但会长期阻塞 |
| C | 以 Mock 测试直接认定 G1 通过 | 不符合 PRD，不建议 |

**建议：A。** 在 README 中把 "Current milestone" 改为 "G1-Mock（未通过 PRD G1 门禁）"，并链接 `PROJECT_STATUS_AUDIT.md`。

**阻塞：** MERGE_PLAN Phase B/F。

---

## D-04 ArmPi 模型修正的范围？

**背景：** Phase 0 已发现 5 类问题：joint3/r_joint 限位冲突、camera optical frame 缺失、限位/effort 占位、陈旧 URDF（joint1/r_joint fixed）、initial_positions 不符。真实机械限位未知。

| 选项 | 内容 | 优点 | 缺点 |
|---|---|---|---|
| **A（推荐）** | 先做"最小可信模型"：统一 joint3/r_joint 限位；加 `camera_link`/`camera_optical_frame`；所有占位值标 `provisional`；删除/重生成陈旧 URDF；补齐 gripper 加速度限制 | 可立即推进 MoveIt/约束规划 | 限位仍非实测，不能实机运动 |
| B | 等实机后按 CAD/测量重建模型 | 最准确 | 阻塞所有仿真/规划工作 |
| C | 不改，继续用现有模型 | 零成本 | 规划可能被静默裁剪；相机链路无法做 |

**建议：A**，并在 README 中明确"仿真限位 provisional，实机运动前必须测量"。B 作为 Phase 1 的后续。

**阻塞：** MERGE_PLAN Phase D。

---

## D-05 Panda Phase II Week 1 未合并分支如何处置？

**背景：** `origin/phase2/week01-execution-reliability`（`e596c5a`）有 4 个提交、10 文件 483+/161-，包含参数化位姿、readiness gate、失败退出、CHANGELOG/evolution 文档；分支自述"runtime regression remains"。`main` 只合并了 13 行失败即停修复；`panda-baseline-v1` 不含 phase2。另有 `agent/JER-18-remediation` 为其代码子集。

| 选项 | 内容 | 优点 | 缺点 |
|---|---|---|---|
| **A（推荐）** | 归档：保留分支与 GitHub ref；在 `docs/archive/` 记录"未合并的 Phase II 实验"；不改 main | 不破坏已冻结基线；保留成果 | phase2 的改进不进入主线 |
| B | 合并：新分支 cherry-pick/merge，补跑 Jazzy+MoveIt 回归后再进 main | 可能提升 Panda 可靠性 | 与"Panda 冻结"冲突；回归成本 |
| C | 删除分支 | 干净 | 丢失成果，不可接受 |

**建议：A**，Phase 0（ArmPi）完成后再评估 B。若选 B，需先明确它与 `panda-baseline-v1` 的关系（新 tag / 重新冻结）。

**阻塞：** MERGE_PLAN Phase E。

---

## D-06 是否授权访问真实机器人/容器？

**背景：** SD 卡 `/var/lib/docker` 与 `/root` 为 root-only；本机无 Docker CLI；机器人 SSH 不可达。容器内 `/home/ubuntu/ros2_ws` 是厂商真实工作区，当前完全未审计。

| 选项 | 内容 | 风险 | 收益 |
|---|---|---|---|
| A | 授权本机 root 读取 SD 卡 Docker 数据（只读挂载/复制） | 需要 sudo；误操作可能损坏 SD | 可审计厂商运行环境 |
| **B（推荐）** | 由你在机器人/容器内导出 `/home/ubuntu/ros2_ws` 的只读 tar（排除 build/install/log），放到 R9000P | 低；不直接碰 SD | 获得真实厂商工作区，可做精简/对照 |
| C | 暂不访问，先做仿真/Mock | 零风险 | 硬件基线继续阻塞 |

**建议：B。** 若必须 A，请明确"只读挂载、不写 SD、不执行 Docker 命令"。
**G0 文档的 D-10 反馈测试**（扭矩关闭、手动转关节、读位置）需要单独书面批准；本轮不建议执行。

**阻塞：** PROJECT_STATUS_AUDIT 的"硬件基线"、MERGE_PLAN Phase D 的实机部分。

---

## D-07 文档的公开/私有边界

**背景：** `MASTER_PLAN_V3_PRIVATE.md` 含申请/时间/预算信息；`ROADMAP_PUBLIC.md` 是公开技术路线；PRD/G0 是内部设计文档。

| 选项 | 内容 |
|---|---|
| **A（推荐）** | 私有计划留在 USB/私有目录；仓库只放 `ROADMAP.md`（由 ROADMAP_PUBLIC 替换）；PRD/G0 放 `docs/armpi/`，视为内部文档，不发布到 GitHub Pages |
| B | 全部提交进仓库 | 可能泄露私有计划，违反 MASTER_PLAN §24 |
| C | 全部不提交 | 文档继续碎片化 |

**建议：A。**

**阻塞：** MERGE_PLAN Phase F。

---

## D-08 证据（mcap / Studio DB）是否进 Git？

**背景：** 仓库 `.gitignore` 已忽略 `*.mcap`、`*.db3`；Studio 子目录忽略 `data/`、`*.db`。mcap 是原始证据，体积大；DB 含运行记录。

| 选项 | 内容 | 优点 | 缺点 |
|---|---|---|---|
| **A（推荐）** | mcap/DB 留在本地/异地备份；Git 只存 `SHA256SUMS.txt`、CSV/PNG、MANIFEST、报告 | 仓库轻；证据可复算 | 原始数据不在 Git |
| B | Git LFS 存 mcap | 原始数据可追溯 | 需要 LFS 配置/配额 |
| C | 直接提交 mcap/DB | 最简单 | 仓库膨胀；二进制不可 diff |

**建议：A + S08/S09 双备份。** 若未来需要公开复现，再考虑 B。

**阻塞：** MERGE_PLAN Phase B/C。

---

## D-09 空/残缺副本与历史残留是否清理？

| 对象 | 建议 | 风险 |
|---|---|---|
| S04 `Projects_M70q/ros2-industrial-arm-learning`（0 文件，坏 `.git`） | 隔离到 `~/backups/quarantine/`，30 天后删除 | 低；确认无会话依赖 |
| S05 `Projects_M70q/...-m2_2-v2`（25 文件，无 `ros2_ws`） | 删除 | 低；已确认无独有内容 |
| S10 `robotics-migration-backup`（175M build 残留） | 打包后删除 | 低 |
| S11 v0.1.0 tar | 保留 | 无 |
| S09 异地证据 | 保留 | 无 |

**建议：全部按上表，先隔离、后删除；本轮不执行。**

**阻塞：** MERGE_PLAN Phase G。

---

## D-10 并行会话 `sess_83d860c9` 如何处置？

**背景：** 该会话正在执行"ArmPi Ultra ROS 2 工程精简与重构"，工作区 `/home/jerichocheng/Projects_M70q`，审计时状态 `running`。它在 2026-11-15 11:20–11:24 已新建 `/home/jerichocheng/Projects_M70q/armpi_ultra_ws_clean`（28M、248 文件、12 个包、无 `.git`），包含 `docs/removed_files.tsv`、`docs/original/`、`tests/test_mecanum_watchdog.py`，并修改了 `kinematics/transform.py`、`controller/mecanum.py`、`ros_robot_controller_node.py`、`bringup/core_check.py`。会话仍在写入，本轮未做正确性审计。

| 选项 | 内容 | 影响 |
|---|---|---|
| A | 先暂停该会话，等主工作区收束后再恢复 | 丢失当前 30 分钟的上下文/进度；但避免任何并发写入 |
| **B（推荐）** | 让它继续在 `armpi_ultra_ws_clean` 内完成，**不并入主仓库**；结束后把该目录作为独立来源审计 | 保留厂商精简成果；需要一次额外审计/合并决策 |
| C | 终止并放弃 | 丢失其已完成的分析与 248 文件工作区 |

**建议：B（隔离）。** 前提是它会话的写入范围仅限于 `Projects_M70q/armpi_ultra_ws_clean`，不触碰 `/home/jerichocheng/projects/ros2-industrial-arm-learning` 或 SD 卡。若发现它开始写主仓库/SD，立即改为 A。

**阻塞：** MERGE_PLAN Phase A。

---

## D-11 GitHub 访问路径

**背景：** R9000P 无法 `git ls-remote` GitHub（超时）；M70q 可以。主工作区在 R9000P。

| 选项 | 内容 |
|---|---|
| **A（推荐短期）** | 用 M70q 作为中继：R9000P ↔ M70q `rsync`/`git bundle`，由 M70q 执行 fetch/push（Board 操作） |
| B | 修复 R9000P 网络/代理，使其直连 GitHub | 长期最简 |
| C | 不 push，仅本地备份 | 失去异地容灾 |

**建议：B 为长期目标，A 为当前可用方案。**

**阻塞：** 任何需要 fetch/push 的合并步骤；不阻塞本地文件合并。

---

## D-12 仓库改名？

**背景：** MASTER_PLAN §25 建议 `ros2-industrial-arm-learning` 可能低估最终项目；候选 `armpi-mobile-manipulation`、`ros2-reliable-manipulation` 等。

| 选项 | 内容 |
|---|---|
| **A（推荐）** | 暂不改名；等研究重点稳定、Phase 2 后再评估 | 改名会影响远端、路径、文档链接；当前不是瓶颈 |
| B | 现在就改 | 需要同步 GitHub、所有本地路径/硬编码 |

**建议：A（暂缓）。**

---

## 决策记录模板（建议 Board 回复格式）

```text
D-01: A
D-02: A
D-03: A
D-04: A
D-05: A
D-06: B（我会导出容器工作区 tar）
D-07: A
D-08: A
D-09: A（先隔离，不删除）
D-10: B（隔离继续；不并入主仓库）
D-11: A（短期用 M70q 中继）
D-12: A（暂缓改名）
```

收到上述回复后，Agent 可以按 `MERGE_PLAN.md` 从 Phase 0/快照开始执行；**任何 `git add/commit/push` 仍由 Board 执行**。
