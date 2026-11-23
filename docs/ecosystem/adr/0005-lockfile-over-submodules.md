# ADR-0005：锁文件 + 契约哈希，不用 submodule

- 状态：提议
- 日期：2026-11-23

## 背景

多仓库组合需要可复现：同一锁文件必须能重建同一系统。ROS 2 的 message type hash
对字段变化敏感；Git tag 可以被移动；分支头会漂移；厂商代码不能公开。

## 决策

使用 `jcheng-system/locks/ecosystem.lock.yaml` 作为唯一组合清单：

- 每个仓库记录 `url`、`version`、`tag`、`commit`、`contract_hash`。
- 工具链记录 ROS 发行版、镜像摘要、Python 版本。
- 机器人配置记录 `profile_version` 与 `profile_sha256`。
- 用 `vcs import` 从锁文件生成 `.repos`；检出后再次校验实际提交哈希。
- **不使用 Git submodule。**
- 运行时不接受契约哈希不匹配的客户端。
- 发布附件包含锁文件、SBOM、校验和、验收报告。

## 后果

正面：

- 组合确定、可审计、可回滚。
- 不需要 submodule 的凭据/嵌套状态。
- 契约哈希能发现“版本号没变但 IDL 改了”。

负面：

- 每次发布要手动/自动更新锁文件。
- 契约变更要求所有消费者重建（ROS 2 类型哈希）。
- 需要维护 `verify_lock.py` 与兼容矩阵。

## 替代方案

1. **Git submodule。** 否决：嵌套仓库、CI 认证、忘记更新、工具链复杂。
2. **只钉 tag。** 否决：tag 可移动；无法表达契约哈希与镜像摘要。
3. **依赖解析器（rosdep/pip）自动求解。** 否决：ROS 2 生态没有跨仓库 SemVer
   求解器；自动求解会牺牲确定性。

## 证据

- `ros-ci.yml` 已用 digest-pinned 镜像与显式包列表。
- `scripts/release_check.sh` 已有脏树/校验和/证据门禁。
- ROS 2 类型哈希随字段变化。
