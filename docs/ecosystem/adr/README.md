# 架构决策记录（ADR）

| ID | 决策 | 状态 |
|---|---|---|
| [0001](0001-contract-first-interfaces-repo.md) | 契约优先，单独 `jcheng-interfaces` 仓库 | 提议 |
| [0002](0002-single-motion-entry-and-lease.md) | 单一运动入口 + 租约仲裁 | 提议 |
| [0003](0003-privileged-capability-planes.md) | 特权/能力平面与 DDS 分域 | 提议 |
| [0004](0004-fresh-history-frozen-baseline.md) | 新仓库全新历史，基线只读 | 提议 |
| [0005](0005-lockfile-over-submodules.md) | 锁文件 + 契约哈希，不用 submodule | 提议 |
| [0006](0006-moveit-plan-only.md) | MoveIt 只规划，执行归仲裁器 | 提议 |
| [0007](0007-fixed-cell-piper-first.md) | 固定单元优先，PiPER X 生产臂，leader 只读 | 已接受（2026-11-28） |

状态说明：`提议` -> `已接受` -> `已取代` / `已废弃`。
批准后由系统负责人更新状态并记录日期。
