# M1 执行记录

> 状态：**M1 完成（2026-11-23）。**
> 验证：`M1-VERIFICATION.json` **111/111 项通过**。
> 范围：组织/仓库初始化、文档、模板、CI 骨架。
> 基线：`jcheng-manipulation v1.0.0` 仍为 `b3411e4`，未改动。

## 1. 结果摘要

| 项 | 结果 |
|---|---|
| 组织 `JCHENG-Robotics` | 已存在且可读 |
| `.github` | 已创建、已推送、`main` 受保护 |
| `jcheng-interfaces` | 已创建、已推送、`main` 受保护 |
| `jcheng-system` | 已创建、已推送、`main` 受保护 |
| `m1-gate` | 三个仓库均运行成功 |
| 分支保护 | PR + CODEOWNERS + `m1-gate` + 禁止 force-push/删除 + 管理员受限 |
| M1 门禁 | 无 IDL、无 ROS 包、无运行时代码、无 LICENSE |
| 许可证 | `pending`；未添加 `LICENSE` |
| 实机 | 未连接、未启动 ROS、未发送运动命令 |

证据：[`evidence/org-init-20261123/README.md`](evidence/org-init-20261123/README.md)、
[`M1-VERIFICATION.json`](evidence/org-init-20261123/M1-VERIFICATION.json)。

## 2. GitHub 资源

| 仓库 | 初始提交 | 首次工作流 |
|---|---|---|
| https://github.com/JCHENG-Robotics/.github | `29e4aec4a429aeb980cc6279bffe16964968e592` | https://github.com/JCHENG-Robotics/.github/actions/runs/45126150641 |
| https://github.com/JCHENG-Robotics/jcheng-interfaces | `86344913d7b376f80d218b8a438d043ee3475278` | https://github.com/JCHENG-Robotics/jcheng-interfaces/actions/runs/45126177951 |
| https://github.com/JCHENG-Robotics/jcheng-system | `7600aadd9539b5a4e133c000ac5b9636f6d4fa31` | https://github.com/JCHENG-Robotics/jcheng-system/actions/runs/45126205095 |

三个仓库均为 public、独立 Git 历史、`main` 默认分支。

## 3. 本地交付物

暂存区：`/home/jerichocheng/jcheng/jcheng-ecosystem-bootstrap/`

| 仓库 | 本地目录 | 初始提交 |
|---|---|---|
| `.github` | `repos/.github` | `29e4aec` |
| `jcheng-interfaces` | `repos/jcheng-interfaces` | `8634491` |
| `jcheng-system` | `repos/jcheng-system` | `7600aad` |

辅助工具：

| 文件 | 作用 |
|---|---|
| `scripts/create_repos.sh` | 创建三个仓库；`--push` 推送初始提交 |
| `scripts/configure_branch_protection.sh` | 应用 `main` 保护与仓库设置 |
| `scripts/verify_m1.py` / `.sh` | 逐项验证并生成 `M1-VERIFICATION.json` |
| `settings/branch-protection.json` | PR/CODEOWNERS/`m1-gate`/禁 force-push/管理员受限 |
| `settings/repo-settings.json` | issues、squash-only、合并后删分支 |
| `docs/M1-RUNBOOK.md` | 执行顺序、失败处理、回滚 |
| `docs/M1-ACCEPTANCE.md` | M1 验收清单 |
| `docs/LICENSE-REVIEW.md` | 许可证/第三方审查（pending） |

## 4. 分支保护（实际验证）

三个仓库的 `main` 均通过 API 验证：

- 必需 PR，至少 1 个批准；CODEOWNERS 批准；stale review 失效。
- 必需状态检查：`m1-gate`。
- `enforce_admins=true`。
- 禁止 force-push、禁止删除。
- 必需对话解决、线性历史。
- 只允许 squash merge；合并后删除分支。

完整字段见 `M1-VERIFICATION.json` 的 `required_check`、`pr_reviews`、
`admins_restricted`、`no_force_push`、`no_delete` 等检查项。

## 5. M1 门禁与许可证

- `.github`：组织主页、社区健康文件、issue/PR 模板、6 个可复用工作流、
  1 个组织健康检查、3 个复合 Action 骨架。
- `jcheng-interfaces`：只有契约审计计划、原则与兼容性模板；**没有 IDL/Schema**。
- `jcheng-system`：只有架构/安全/验收文档与目录占位；**没有运行时代码**。
- 三个仓库均无 `LICENSE`；README 与 `LICENSE-REVIEW.md` 标注 `pending`。
- 许可证审查完成前，不添加 MIT、不对外宣称已采用 MIT。

## 6. 需要后续决定的两个治理项

### 6.1 单账号与强制评审

组织当前只有 `JerichoCheng` 一个账号；GitHub 不允许自我批准 PR。
M1 的初始提交是在保护开启前推送的。此后任何 `main` 改动都需要：

- 第二个评审人（推荐：新增一个 maintainer/reviewer 账号），或
- 书面记录的 break-glass：临时关闭保护、推送、重新开启并记录审计。

在 M2 开始前需要确定采用哪种方式；不得默认关闭保护。

### 6.2 组织级设置

`PATCH /orgs/JCHENG-Robotics` 返回 403：当前 fine-grained token 不含
`admin:org`。组织级 2FA、基础权限、成员创建权限尚未通过 API 应用。
仓库级设置与 `main` 保护已应用并验证。建议在网页端手动开启 2FA 与
`read` 基础权限，或提供 classic `admin:org` 令牌后补做。

## 7. 基线完整性

- `v1.0.0` 仍指向 `b3411e4`；`release/manifest.json` 未变化。
- 没有修改 `jcheng-manipulation` 的核心代码、证据或实验历史。
- 没有连接机器人、没有启动 ROS 节点、没有发送运动命令。
- 本次只在设计分支新增文档与证据文件。

## 8. 下一步

- M1 已完成；进入 M2 前先决定 6.1 的评审方式。
- M2 第一件事是契约审计（`jcheng-interfaces/docs/CONTRACT_AUDIT_PLAN.md`），
  审计签字前不定义最终 IDL。
- 许可证审查与第三方依赖清单在 M2 并行完成；审查通过后才添加 `LICENSE`。
- 任何实机运动仍需单独明确授权；M1 没有实机动作。
