# MERGE_PLAN — 统一收束方案（未执行，待批准）

> **状态：仅方案。本轮未执行任何合并、覆盖、删除、`git add/commit/push`。**
> 目标：把分散在 R9000P / M70q / SD / USB 的成果收束为一个可追溯的主工作区，同时保留所有有价值的历史与证据。
> 关联文件：`SOURCE_INVENTORY.md`、`CHANGE_RECONCILIATION.md`、`PROJECT_STATUS_AUDIT.md`、`DECISION_REQUIRED.md`

## 0. 原则

1. **单一主工作区**：`/home/jerichocheng/projects/ros2-industrial-arm-learning`（R9000P）。
2. **不覆盖任何未提交成果**：先快照、再合并；每一步可回滚。
3. **Board 独占 Git 写权限**：Agent 只准备 diff/文件/验证结果，不执行 `git add/commit/push`。
4. **证据与代码分离**：mcap/DB/大资产不进 Git；Git 只存源码、文档、清单、可复算的 CSV/PNG。
5. **厂商代码只读**：`ArmPiUltra` 16G 保留在仓库外，不复制、不混编。
6. **私有计划不进公开仓库**：`MASTER_PLAN_V3_PRIVATE.md` 留在 USB/私有目录。

## 1. 推荐主工作区

### 1.1 结论

**主工作区：`/home/jerichocheng/projects/ros2-industrial-arm-learning`（R9000P）**

### 1.2 理由

| 维度 | R9000P 主克隆（S01） | M70q 主仓库（S02） |
|---|---|---|
| Git 完整性 | 完整 `.git` 21M，51 commits，tag 齐全 | 完整，但路径在远程桌面 |
| 已有成果 | M2.2 实测变体 + bags 原件 | Studio G1 + phase0 + M2.2 rework1 |
| 厂商资料 | `ArmPiUltra` 16G、PRD、G0 | 无 |
| 硬件/介质 | SD 卡、USB 计划、机器人直连历史 | 无 |
| 仿真环境 | ROS 2 Jazzy、MoveIt、`armpi_sim_ws`、运行日志 | ROS 环境未知/未验证 |
| 网络 | GitHub 不可达 | GitHub 可达 |
| 结论 | **主工作区** | **合并来源 + 备份 + GitHub 中继** |

### 1.3 各来源的最终角色

| 来源 | 角色 |
|---|---|
| S01 R9000P 主克隆 | **唯一主工作区** |
| S02 M70q 主仓库 | Studio/phase0 合并来源；合并后作为异地备份；GitHub 推送中继 |
| S03 M70q worktree | M2.2 rework1 来源；合并后移除 worktree（保留分支） |
| S04/S05 空/残缺副本 | 隔离，验证后删除 |
| S06 `armpi_sim_ws` | ArmPi 包来源；合并进主仓库 `ros2_ws/src` |
| S07 `ArmPiUltra` | 厂商只读参考；PRD/G0 复制进 `docs/`；16G 资产不进 Git |
| S08/S09 M2.2 证据 | S08 为原件，S09 为异地备份；复制清单/分析进 Git，mcap 不进 |
| S10 WSL2 残留 | 归档/删除 |
| S11 v0.1.0 tar | 历史回滚点，保留 |
| S12 SD 卡 | 真实机器人系统；只读，不部署 |
| S13 USB 计划 | 计划唯一副本；公开路线进仓库，私有计划留外 |
| S17 `Projects_M70q/armpi_ultra_ws_clean` | 并行会话的厂商精简工作区（审计期间新建，仍在写入）；**不并入主仓库**，等待其结束并单独审计 |

### 1.4 为什么不选 `Projects_M70q`

- `Projects_M70q/ros2-industrial-arm-learning` 是 **0 文件骨架**，`.git` 缺 `HEAD/config/index`，不可用。
- `Projects_M70q/...-m2_2-v2` 只有 25 个文件，`ros2_ws` 为空，`.git` 指向不存在的 worktree。
- 若把主工作区迁到 `Projects_M70q`，会破坏 M2.2 中硬编码的 `/home/jerichocheng/projects/...` 路径，并增加一次无收益迁移。
- 并行会话 `sess_83d860c9` 正在使用 `Projects_M70q`，存在冲突风险。

## 2. 目标目录结构（合并后）

```text
/home/jerichocheng/projects/ros2-industrial-arm-learning/
├── README.md                        # 重写：项目定位、状态、vendor/personal 边界、运行方式
├── ROADMAP.md                       # 用 ROADMAP_PUBLIC 内容替换；旧版归档
├── CHANGELOG.md                     # 可选：采用 phase2 的 CHANGELOG 结构
├── docs/
│   ├── audit/2026-11-15/            # 本套 5 份审计报告
│   ├── armpi/
│   │   ├── ARMPI_DEBUG_STUDIO_PRD_V1.md
│   │   └── g0/                      # 01–07 + README
│   ├── phase0/                      # JER-39 / JER-59
│   ├── hardware/                    # MASTER_PLAN §6 的 7 份模板（先标 [U]）
│   ├── evolution/phase2/            # 若采纳 phase2 文档
│   ├── archive/ROADMAP_8WEEK.md     # 旧 8 周路线
│   └── weekly-log/                  # 保留
├── evidence/
│   ├── m2_2_v2/
│   │   ├── MANIFEST.md              # 哈希、时间、代码快照说明
│   │   ├── analysis/ *.csv *.png
│   │   ├── config_snapshot/         # 3 个源码快照
│   │   └── bags/                    # mcap；被 .gitignore 忽略，仅本地/异地备份
│   └── studio/
│       ├── g1-verification.md
│       └── studio.db.snapshot       # 可选，默认不进 Git
├── armpi-debug-studio/              # 只复制源码；deps/data 排除
│   ├── backend/  frontend/  configs/  docs/  README.md  .gitignore
├── ros2_ws/
│   └── src/
│       ├── arm_basics/ arm_description/ arm_interfaces/ arm_moveit_demo/
│       ├── armpi_ultra_description/
│       └── armpi_ultra_moveit_config/
└── (不入库) ../../ArmPiUltra/        # 厂商 16G 只读参考
```

> 备选：若担心 Panda 与 ArmPi 依赖互相污染，可保留两个 ROS 工作区 `ros2_ws/`（Panda）与 `armpi_ws/`（ArmPi），但**同一个 Git 仓库、同一主目录**。推荐先用单 `ros2_ws`，构建失败再拆分。

## 3. 合并前快照（必须先做）

> 这些操作会产生文件，但不改变项目内容；在 Board 批准后执行。**不要**先 `git clean`/`reset --hard`。

### 3.1 仓库快照

```bash
cd /home/jerichocheng/projects/ros2-industrial-arm-learning
mkdir -p /home/jerichocheng/backups/2026-11-15
# 工作区 diff（含未跟踪文件的独立备份）
git diff > /home/jerichocheng/backups/2026-11-15/local-m2.2-tracked.patch
cp ros2_ws/src/arm_description/config/arm_controllers_gain_0p2.yaml \
   /home/jerichocheng/backups/2026-11-15/
# 全仓库 tar（排除 build/install/log/.git 可选；建议保留 .git）
tar --exclude='ros2_ws/build' --exclude='ros2_ws/install' --exclude='ros2_ws/log' \
    -czf /home/jerichocheng/backups/2026-11-15/repo-before-merge.tar.gz .
```

### 3.2 M70q 独有成果快照

```bash
# 在 R9000P 执行；只拉源码，不拉依赖/构建产物
rsync -a --info=progress2 \
  --exclude='.venv/' --exclude='node_modules/' --exclude='dist/' \
  --exclude='data/' --exclude='__pycache__/' --exclude='*.egg-info/' \
  --exclude='.pytest_cache/' --exclude='*.tsbuildinfo' \
  <user>@<M70Q_TAILSCALE_IP>:~/projects/ros2-industrial-arm-learning/armpi-debug-studio/ \
  /home/jerichocheng/backups/2026-11-15/armpi-debug-studio-src/

rsync -a \
  <user>@<M70Q_TAILSCALE_IP>:~/projects/ros2-industrial-arm-learning/docs/phase0/ \
  /home/jerichocheng/backups/2026-11-15/phase0/

rsync -a --exclude='build-gate2*' --exclude='install-gate2*' --exclude='log-gate2*' \
  <user>@<M70Q_TAILSCALE_IP>:~/projects/ros2-industrial-arm-learning-m2_2-v2/ \
  /home/jerichocheng/backups/2026-11-15/m2_2-v2-full/
```

### 3.3 保留 M2.2 实测变体（Git 分支，由 Board 执行）

```bash
# Board 执行，Agent 不执行
git switch -c m2.2/gazebo-validated-20261110
git add ros2_ws/src/arm_description/launch/gazebo.launch.py \
        ros2_ws/src/arm_description/urdf/simple_arm.xacro \
        ros2_ws/src/arm_description/config/arm_controllers_gain_0p2.yaml
git commit -m "chore(m2.2): preserve gazebo-validated controller config (2026-11-10 bags)"
git switch main
```

> 目的：把"产生 mcap 的源码"固定在 Git 中，之后 main 可以安全切换到 rework1。

## 4. 合并顺序（逐阶段）

### Phase A — 冻结并行活动

1. 确认并行会话 `sess_83d860c9`（ArmPi 精简重构）已停止，或明确其输出只写入 `Projects_M70q/armpi_ultra_ws_clean`、不会触碰主工作区；该目录在审计期间仍在写入（11:20–11:24，28M/248 文件）。
2. 确认 M70q 上没有正在运行的 Studio/ROS 进程（本次检查为无）。
3. 在 `DECISION_REQUIRED.md` 获得 D-01～D-06 的答复。

**退出条件**：无并发写入；Board 批准合并范围。

### Phase B — 导入文档与 Studio（低风险，先做）

| 步骤 | 源 | 目标 | 排除 | 验证 |
|---|---|---|---|---|
| B1 | S02 `docs/phase0/` | `docs/phase0/` | — | 2 文件哈希一致 |
| B2 | S07 `ARMPI_DEBUG_STUDIO_PRD_V1.md` | `docs/armpi/` | — | 文件字节一致 |
| B3 | S07 `docs/g0/` | `docs/armpi/g0/` | — | 8 文件哈希一致 |
| B4 | S02 `armpi-debug-studio/` | `armpi-debug-studio/` | `.venv/ node_modules/ dist/ data/ __pycache__/ *.egg-info/ *.tsbuildinfo` | 源码文件数 28 py + 16 ts/tsx；`git status` 不显示依赖 |
| B5 | S08/S09 `evidence/SHA256SUMS.txt` + CSV/PNG | `evidence/m2_2_v2/` | `*.mcap`（本地保留） | 哈希清单可复算 |
| B6 | 本套 5 份审计 | `docs/audit/2026-11-15/` | — | 5 文件存在 |

**注意：**

- `armpi-debug-studio/.gitignore` 已忽略 `.venv/`、`node_modules/`、`dist/`、`data/`、`*.db`；仍需在根 `.gitignore` 中确认不会误收 177M。
- 不要复制 M70q 的 `.venv`/`node_modules` 到 R9000P；Python/Node 版本可能不同，需重装。
- `docs/armpi/g0` 的状态是"待 Board 评审"；导入时保留原文，不要改写为"已完成"。

**退出条件**：`git status` 只显示预期的源文件；`pytest` 在 R9000P 重新安装依赖后可运行（不要求本轮）。

### Phase C — M2.2 合并（中风险）

1. 保留 S01 实测变体（Phase 3.3 分支）。
2. 把 S03 的 rework1 文件复制到 S01：
   - `CMakeLists.txt`、`package.xml`、`launch/gazebo.launch.py`、`urdf/simple_arm.xacro`；
   - `arm_description/controller_config.py`、`test/test_controller_config.py`；
   - `config/arm_controllers_gain_0p1.yaml`、`config/arm_controllers_gain_0p2.yaml`；
   - `evidence/m2_2_v2/gate2/.../implementation-report.md`。
3. 不要复制 S03 的 `build-gate2*`/`install-gate2*`/`log-gate2*`/`__pycache__`。
4. 重新构建：

```bash
cd ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select arm_description --symlink-install
colcon test --packages-select arm_description
colcon test-result --verbose
```

5. 运行时回归（R9000P，Gazebo）：
   - 默认 config、gain 0.1、gain 0.2 各跑一次轨迹；
   - 录制新 bag 到 `evidence/m2_2_v2/bags/`；
   - 记录实际 `ros2_control`/controller 日志；
   - 与 11-10 的 settling/tracking 指标对比。
6. 若 rework1 运行时失败：回滚到 `m2.2/gazebo-validated-20261110`，保留报告，重新评估。

**退出条件**：构建通过；离线测试通过；至少一次 Gazebo 轨迹执行成功；新旧数据差异有解释。

### Phase D — ArmPi 模型与 MoveIt 合并（中高风险）

1. 从 S06 复制两个包到 `ros2_ws/src/`：
   - `armpi_ultra_description`、`armpi_ultra_moveit_config`。
2. 先做**不改变行为**的整理：
   - 删除/重命名陈旧 `urdf/armpi_ultra.urdf`；所有入口只允许 xacro；
   - 在 xacro 中加入 `provisional` 注释/参数标记；
   - 为 `camera_connet_link` 增加 `camera_link` 与 `camera_optical_frame`（REP-103：z 前、x 右、y 下）；
   - 统一 joint3 限位（至少 `-2.094…0`），`r_joint` 与软件范围一致；
   - 补齐 gripper 关节加速度限制；
   - 修正 `initial_positions.yaml` 或明确标注"仅仿真占位"。
3. 验证：

```bash
xacro src/armpi_ultra_description/urdf/armpi_ultra.xacro > /tmp/armpi.urdf
check_urdf /tmp/armpi.urdf
colcon build --packages-select armpi_ultra_description armpi_ultra_moveit_config
ros2 launch armpi_ultra_moveit_config demo.launch.py
ros2 run tf2_tools view_frames
```

4. IK：
   - 安装 `trac_ik` 或改用 `pick_ik`（需 Board 批准依赖变更）；
   - 先做 position-only 可达性统计（明确标注仅可达性）；
   - 再加 `OrientationConstraint` / approach pitch 约束，验证 5-DoF 可解子空间；
   - **不得**把 `position_only_ik: true` 当作最终抓取方案。

**退出条件**：`check_urdf` 通过；TF 单根；RViz 可加载；MoveIt 关节空间目标成功；至少一个约束位姿目标可规划（不要求执行）。

### Phase E — Panda 未合并分支处置（低风险，但需决策）

三选一（见 `DECISION_REQUIRED.md` D-05）：

- **E1 归档**：保留 `origin/phase2/...`，在 `docs/archive/` 记录"未合并实验"；不改 main。
- **E2 合并**：将 phase2 的代码/文档 cherry-pick 到新分支，重新跑仿真回归，再决定是否进 main。
- **E3 放弃**：明确标记 superseded，保留 ref 不删。

推荐 **E1 + 后续评估**：当前 `panda-baseline-v1` 已冻结；不要在无运行回归的情况下改动 Panda 主线。

### Phase F — 计划与文档统一

1. `ROADMAP.md`：
   - 旧版移到 `docs/archive/ROADMAP_8WEEK.md`；
   - 用 S13 `ROADMAP_PUBLIC.md` 内容替换（保留来源/日期）。
2. `README.md`：
   - 项目定位从"8 周学习"改为"ArmPi Ultra 5-DoF 操纵 + 可靠性实验"；
   - 增加状态表（Implemented / Validated / In Progress / Planned）；
   - 明确 Vendor baseline vs 本人实现；
   - 运行方式区分 Panda demo / ArmPi sim / Studio Mock。
3. `docs/hardware/`：按 MASTER_PLAN §6 建立 7 个模板，全部标 `[U]`，等待实机。
4. `MASTER_PLAN_V3_PRIVATE.md`：**不放进公开仓库**；建议复制到 `~/Documents/private/` 并保留 USB 副本；在仓库中只放 `ROADMAP.md`。
5. `CHANGELOG.md`：若采纳 phase2 的格式，可复用其 Keep a Changelog 结构。

### Phase G — 清理（最后做，逐项批准）

| 对象 | 动作 | 前置 |
|---|---|---|
| S04 0 文件骨架 | 移到 `~/backups/quarantine/` 后删除 | 确认无会话依赖 |
| S05 残缺 M2.2 | 删除 | 已确认 25 文件全为 S03 子集 |
| S10 WSL2 残留 | 打包后删除 | 保留 tar |
| S11 v0.1.0 tar | 移到 `~/backups/` | 保留 |
| S03 worktree | 合并后 `git worktree remove` | Board 已提交/合并分支 |
| S09 异地证据 | 保留 | 作为 S08 的异地备份 |
| `Projects_M70q` 目录 | 仅保留本审计目录/或整体归档 | 并行会话结束后 |

## 5. 文件级映射表

| 源路径 | 目标路径 | 动作 | 冲突处理 |
|---|---|---|---|
| S02 `armpi-debug-studio/**`（源码） | `armpi-debug-studio/**` | 复制 | 无本地对应；排除依赖/data |
| S02 `docs/phase0/*.md` | `docs/phase0/` | 复制 | 无 |
| S07 `ARMPI_DEBUG_STUDIO_PRD_V1.md` | `docs/armpi/` | 复制 | 无 |
| S07 `docs/g0/**` | `docs/armpi/g0/` | 复制 | 无 |
| S03 `CMakeLists.txt` | `ros2_ws/src/arm_description/CMakeLists.txt` | 替换 | 采用 S03 |
| S03 `package.xml` | 同上 | 替换 | 采用 S03 |
| S03 `launch/gazebo.launch.py` | 同上 | 替换 | 采用 S03；保留 S01 分支 |
| S03 `urdf/simple_arm.xacro` | 同上 | 替换 | 采用 S03；保留 S01 分支 |
| S03 `arm_description/controller_config.py` | 新文件 | 复制 | — |
| S03 `test/test_controller_config.py` | 新文件 | 复制 | — |
| S03 `config/arm_controllers_gain_0p1.yaml` | 新文件 | 复制 | — |
| S03 `config/arm_controllers_gain_0p2.yaml` | 同路径 | 复制 | 与 S01 哈希相同 |
| S03 `evidence/m2_2_v2/**` | `evidence/m2_2_v2/` | 复制报告 | mcap 不进 Git |
| S06 `src/armpi_ultra_description` | `ros2_ws/src/` | 复制 | 先整理 URDF/限位/camera |
| S06 `src/armpi_ultra_moveit_config` | `ros2_ws/src/` | 复制 | 修 joint_limits/kinematics |
| S08/S09 CSV/PNG/SHA256SUMS | `evidence/m2_2_v2/` | 复制 | mcap 本地保留 |
| S13 `ROADMAP_PUBLIC.md` | `ROADMAP.md` | 替换 | 旧版归档 |
| S13 `MASTER_PLAN_V3_PRIVATE.md` | 仓库外 | 不复制 | 私有 |

## 6. 验证计划（合并后必须执行）

### 6.1 Git/内容层

```bash
git status --short
git diff --check
# 确认没有 node_modules/.venv/dist/data/mcap 被跟踪
git ls-files | grep -E 'node_modules|\.venv|dist/|\.mcap$|studio\.db' && echo "UNEXPECTED" || echo "clean"
```

### 6.2 构建层

```bash
cd /home/jerichocheng/projects/ros2-industrial-arm-learning/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
colcon test
colcon test-result --verbose
```

### 6.3 模型层

```bash
xacro src/armpi_ultra_description/urdf/armpi_ultra.xacro > /tmp/armpi.urdf
check_urdf /tmp/armpi.urdf
# 断言：joint1–5 revolute；r_joint revolute；无 fixed joint1
python3 - <<'PY'
import xml.etree.ElementTree as ET
r=ET.parse('/tmp/armpi.urdf').getroot()
print({j.get('name'):j.get('type') for j in r.findall('joint')})
PY
```

### 6.4 MoveIt 层

- `ros2 launch armpi_ultra_moveit_config demo.launch.py`；
- 检查 TRAC-IK/pick_ik 已加载；
- 关节空间目标：规划+执行成功；
- 约束位姿目标：规划成功（可仅规划）；
- 记录 `move_group` 日志到 `evidence/armpi_moveit/`。

### 6.5 M2.2 回归

- 默认/0.1/0.2 三种 config 各跑一次；
- 新 bag + 新分析；
- 与 11-10 的 CSV 对比 max/RMS/final；
- 结论写入 `evidence/m2_2_v2/REGRESSION.md`。

### 6.6 Studio

```bash
cd armpi-debug-studio/backend
python3 -m venv .venv && . .venv/bin/activate
pip install -e '.[dev]'
pytest -q
cd ../frontend
npm install
npm run build
```

- 期望：41 用例（或实际收集数）全过；`tsc -b && vite build` 成功；
- 若补 Playwright：新增 `frontend/e2e/`，否则在 README 明确"无浏览器自动化"。

### 6.7 文档层

- README 状态表与 `PROJECT_STATUS_AUDIT.md` 一致；
- 所有"已验证"声明附证据路径；
- `docs/hardware/*` 全部标 `[U]`；
- 私有计划不在仓库。

## 7. 回滚方案

| 层级 | 回滚动作 |
|---|---|
| Git 分支 | 每个阶段在独立分支进行；失败则 `git switch main`（不 reset 已有工作） |
| 工作区文件 | 用 `/home/jerichocheng/backups/2026-11-15/repo-before-merge.tar.gz` 解压到临时目录对照，不直接覆盖 |
| M2.2 | 回到 `m2.2/gazebo-validated-20261110`；rework1 文件保留在备份目录 |
| ArmPi 模型 | 保留 S06 原目录；合并失败则从 S06 重新复制 |
| Studio | 保留 S02 原目录；只读复制，失败无影响 |
| 证据 | S08/S09 双副本；任何合并都不删除原件 |
| 清理 | 所有删除先移到 `~/backups/quarantine/`，保留 30 天 |

**禁止**：`git reset --hard`、`git clean -fdx`、`rsync --delete` 到源目录、对 SD 卡写入。

## 8. 风险与缓解

| 风险 | 概率 | 影响 | 缓解 |
|---|---|---|---|
| rework1 运行时回归失败 | 中 | M2.2 主分支不可用 | 先保留实测分支；Gazebo 回归后再合并 |
| 合并丢失未提交 Studio/M2.2 | 低（有备份后） | 高 | Phase 3 快照；只读复制 |
| 陈旧 URDF 被误用 | 中 | 规划错误/实机危险 | 删除/重命名 + xacro-only 入口 + 测试断言 |
| joint3/r_joint 限位错误 | 高 | 实机静默裁剪/碰撞 | 统一限位来源；provisional 标记；实机测量前禁止运动 |
| `position_only_ik` 被当成抓取方案 | 中 | 抓取姿态不可用 | README/代码注释明确仅可达性；加 OrientationConstraint |
| Studio 177M 依赖误入 Git | 中 | 仓库膨胀 | 根/子 `.gitignore` + `git status` 检查 |
| 16G 厂商资产误入 Git | 低 | 仓库不可用 | 厂商目录在仓库外；只存 hashes/manifest |
| 并行会话写入冲突 | 中 | 文件被覆盖 | Phase A 冻结；S17 隔离在 `Projects_M70q/armpi_ultra_ws_clean`，不合并；结束后单独审计 |
| Phase2 分支合并冲突 | 高 | Panda 回归 | 先归档，不自动合并；Board 决策 |
| R9000P 无法访问 GitHub | 高 | 无法 fetch/push | 用 M70q 中继（rsync/bundle），或修复网络 |
| Python/Node 版本差异 | 中 | Studio 不可运行 | 在 R9000P 重建 venv/node_modules，不复制依赖 |
| 文档再次漂移 | 高 | 状态不可信 | 每次合并更新 README 状态表 + 证据链接 |

## 9. 建议执行顺序与工作量

| 阶段 | 内容 | 预计 |
|---|---|---|
| 0 | Board 决策 + 快照 | 0.5 天 |
| A | 冻结并行活动 | 0.5 天 |
| B | 文档 + Studio 导入 | 0.5–1 天 |
| C | M2.2 rework1 + Gazebo 回归 | 1–2 天 |
| D | ArmPi 包导入 + 模型修正 + MoveIt 验证 | 2–4 天 |
| E | Panda phase2 处置 | 0.5 天（归档）/ 2–3 天（合并） |
| F | README/ROADMAP/docs | 1 天 |
| G | 清理 | 0.5 天 |

> 关键路径是 C 和 D；B 可以先做，低风险。

## 10. 审批门槛

在执行任何一步之前，需要 Board 对 `DECISION_REQUIRED.md` 的以下项给出决定：

- D-01 M2.2 版本策略；
- D-02 Studio 归属与提交；
- D-03 Studio G1 的验收口径；
- D-04 ArmPi 模型修正范围；
- D-05 Panda phase2 分支处置；
- D-06 硬件/容器访问授权。

没有这 6 项，合并计划不应启动。
