# JER-58 第 1 轮定向返工实施报告

时间：2026-11-12 UTC  
工作区：`/home/jerichocheng/projects/ros2-industrial-arm-learning-m2_2-v2`  
基线：`m2-2-v2`，`435f8ec1e891bfc22084536e16e4d61bdaa7e5a0`

## 实施范围

- 修复 V2 profile 的 gain scope：唯一 gain 叶为 `/gz_ros_control.ros__parameters.position_proportional_gain`，不再使用 JTC 每关节 PID。
- 将校验 helper 安装为 `arm_description.controller_config` Python package；launch 从该安装模块导入。
- 严格校验 V2/default schema、YAML 特性与路径；支持 `~`，拒绝环境变量展开字符、控制字符及前后空白。
- 每一阶段在实际加入启动链时才创建 60 秒 watchdog；成功完成后 watchdog 为 inert。`gz_sim.launch.py` 明确收到 `on_exit_shutdown:=true`。
- 恢复 Xacro 的 package-share 默认控制器配置；补全运行/测试依赖。
- 增加离线 schema、路径、YAML 错误、事件链、source/install launch 载入与模型检查回归。

普通 `config/arm_controllers.yaml` 未修改。

## R1–R6 对照

| 项 | 结果 | 证据 |
|---|---|---|
| R1 | 已实施 | 两 profile 哈希精确为批准值；测试断言唯一叶、float 类型和逐字节单变量差异。 |
| R2 | 已实施 | `ament_cmake_python` 安装 helper；受控 overlay 下真实 launch loader 载入 source/install 均成功。 |
| R3 | 已实施（离线） | stage callback 只在 code=0 时加入下一 stage + 新 watchdog；非零不推进；完成 watchdog inert；Gazebo include 显式 shutdown。未运行 Gazebo/ROS 节点。 |
| R4 | 已实施 | `expanduser`、严格根/节点 schema、固定布尔值、Xacro 默认值及缺失依赖已覆盖。 |
| R5 | 已实施（自检） | `colcon test-result`: 9 tests, 0 errors, 0 failures；另有 8 个直接离线 unittest。独立 QA 尚未进行。 |
| R6 | 已实施工程防复发 | 本轮 build/test 均以 `env -i` 白名单环境运行，未继承 Paperclip/API/SSH 凭据，也未读取旧敏感日志。历史凭据暴露的撤销/轮换仍由 Shuhua 协调授权责任人处理。 |

## 文件与 SHA-256

| 文件 | SHA-256 |
|---|---|
| `src/arm_description/CMakeLists.txt` | `8952926b8eaa93322e7e6c3d2e6e93276d8aa694cf8651aa207f83e8382b8456` |
| `src/arm_description/package.xml` | `b66ed73c50dfe90dd0c8d78103447de314ccdc7697b0edeaa9f42553fd6881b6` |
| `src/arm_description/urdf/simple_arm.xacro` | `a4f89f14aee8248bfc216e6b9929f1ae819d4b236e290c270aeb8111501314e1` |
| `src/arm_description/launch/gazebo.launch.py` | `61cb84159e68cf4fe880b3edf585aabdee201462d0e422e708adf3efa54dc3b4` |
| `src/arm_description/arm_description/controller_config.py` | `0a09592e78cdc878fac84e4062635db82acabb41d221129694a38e908ec0dfcb` |
| `src/arm_description/test/test_controller_config.py` | `4a6114c961282c3f7a58ae8eb7a13e060506b74ba51775081f96a12721dc7e00` |
| `config/arm_controllers_gain_0p1.yaml` | `708408e9d77dec8aacc6a657ccc2adaca932731e320b0aaf5a70355676d567a3` |
| `config/arm_controllers_gain_0p2.yaml` | `5ceea231197035fc41a08fce0dc3e9ccab2a77b00efd4620adfbfbf358abe0cd` |

安装产物的 `gazebo.launch.py`、`controller_config.py` 和两份 YAML 与上述相应源文件 SHA-256 相同。

## 可重放离线命令与结果

所有 subprocess 使用：`env -i PATH=/usr/bin:/bin HOME=/tmp/jer58-home LC_ALL=C.UTF-8 LANG=C.UTF-8 PYTHONDONTWRITEBYTECODE=1`，随后仅 source `/opt/ros/jazzy/setup.bash` 与本轮 install overlay。

| 命令 | 退出码 | 结果 |
|---|---:|---|
| `colcon --log-base log-gate2-rework1 build --packages-select arm_description --build-base build-gate2-rework1 --install-base install-gate2-rework1` | 0 | 1 package finished |
| `colcon --log-base log-gate2-rework1 test --packages-select arm_description --build-base build-gate2-rework1 --install-base install-gate2-rework1` | 0 | 1 package finished |
| `colcon test-result --test-result-base build-gate2-rework1 --verbose` | 0 | 9 tests, 0 errors, 0 failures, 0 skipped |
| overlay 下 `/tmp` 执行 `python3 test/test_controller_config.py` | 0 | 8 tests OK |
| overlay 下 `get_launch_description_from_python_launch_file` 载入 source/install `gazebo.launch.py` | 0 | source/install 均为 3 entities |

首次调用 `python3 -m unittest test/test_controller_config.py`（exit 1）因 unittest 将 filesystem path 解释成 import name；随后使用可执行测试文件完成重测。首次 install launch 检查（exit 1）使用了 merged-install 路径；实际安装为 isolated layout，修正到 `install-gate2-rework1/arm_description/share/...` 后通过。两项均为命令路径错误，不是源码回归。

## Xacro 离线检查

在内存使用 xacro Python API 展开默认、0.1、0.2 及映射路径变体；唯一 plugin `parameters` 元素指向选择路径。原始 URDF SHA-256 分别为：

- default: `1acd04b3c88d16ea78476b1b3e43928560b987b7b7b4ef75281bb2ca86cd6ec4`
- 0.1: `850e2bb199485d2a8d6c2d6a5e774bd2f5a8072631c9d6a9215489a6faa4a8cb`
- 0.2: `93b52f3f32931eff1a7a4d519910bac3be03fd2d62d991366bbe323ac9aa31f8`

将仅该 `parameters` 文本归一化后，模型字节结构一致。原始哈希因路径文本不同而不同，未声称相同。

## 未验证项目

- 未启动 Gazebo、ROS daemon、controller、bridge、轨迹或实体机器人；因此不构成 Gate 3、运行时 ready 或独立 QA PASS。
- 需 CTO 复审后由独立 QA 按 Gate 2 进行独立验收。
- 推荐 Commit Message：`fix(arm_description): align V2 gain scope and launch failure handling`。未执行 `git add`、`git commit` 或任何 Git 写操作。
