# M1 组织与仓库初始化证据

- 日期：2026-11-23
- 组织：`JCHENG-Robotics`（https://github.com/JCHENG-Robotics）
- 操作者：`JerichoCheng`
- 范围：M1 组织/仓库初始化、文档、模板、CI 骨架
- 基线：`jcheng-manipulation v1.0.0` -> `b3411e4`（未改动）

## 1. 结果

| 项 | 结果 |
|---|---|
| 三个仓库创建 | 通过 |
| 初始提交推送 | 通过 |
| `main` 分支保护 | 通过（API 逐项验证） |
| `m1-gate` 工作流 | 三个仓库均 `success` |
| M1 门禁 | 通过（无 IDL、无 package、无运行时码、无 LICENSE） |
| 基线标签 | `v1.0.0` 仍为 `b3411e4` |
| 自动验证 | **111/111 checks passed** |
| 许可证 | `pending`，三个仓库均无 `LICENSE` |
| 实机 | 未连接、未发送任何命令 |

验证报告：[`M1-VERIFICATION.json`](M1-VERIFICATION.json)，
SHA-256 见 [`M1-VERIFICATION.sha256`](M1-VERIFICATION.sha256)。

## 2. 仓库与提交

| 仓库 | 初始提交 | 工作流 |
|---|---|---|
| https://github.com/JCHENG-Robotics/.github | `29e4aec4a429aeb980cc6279bffe16964968e592` | https://github.com/JCHENG-Robotics/.github/actions/runs/45126150641 |
| https://github.com/JCHENG-Robotics/jcheng-interfaces | `86344913d7b376f80d218b8a438d043ee3475278` | https://github.com/JCHENG-Robotics/jcheng-interfaces/actions/runs/45126177951 |
| https://github.com/JCHENG-Robotics/jcheng-system | `7600aadd9539b5a4e133c000ac5b9636f6d4fa31` | https://github.com/JCHENG-Robotics/jcheng-system/actions/runs/45126205095 |

机器可读摘要：[`M1-EVIDENCE.json`](M1-EVIDENCE.json)。

## 3. 分支保护（已实际配置并验证）

三个仓库的 `main` 均为：

- 必需 PR，至少 1 个批准；CODEOWNERS 批准；stale review 失效。
- 必需状态检查：`m1-gate`。
- `enforce_admins=true`。
- 禁止 force-push、禁止删除。
- 必需对话解决、线性历史。
- 只允许 squash merge；合并后删除分支。

验证方式：`GET /repos/JCHENG-Robotics/<repo>/branches/main/protection`
与三个仓库的 Actions check run。完整字段在 `M1-VERIFICATION.json`。

**单账号约束：** 组织当前只有 `JerichoCheng` 一个账号；GitHub 不允许自我批准 PR。
因此 M1 的初始提交是在保护开启前推送的；后续任何改动都需要第二个评审人，
或采用书面记录的 break-glass 流程。该问题需在 M2 前决定。

## 4. 许可证与第三方审查

- 三个仓库均无 `LICENSE`；README 与 `LICENSE-REVIEW.md` 明确标注 `pending`。
- 完整第三方清单与批准流程：`.github/docs/license-review.md`。
- 在审查完成前，不添加 MIT，不对外宣称已采用 MIT。

## 5. 组织级设置

- `PATCH /orgs/JCHENG-Robotics` 返回 403：当前 fine-grained token 不含
  `admin:org`，组织级设置（2FA、基础权限、成员创建权限）未通过 API 应用。
- 仓库级设置与分支保护已应用并验证，满足 M1 的“main 分支保护实际配置并验证”。
- 建议在网页端手动开启 2FA 要求与 `read` 基础权限，或在后续提供 classic
  `admin:org` 令牌后补做。

## 6. 约束遵守

- 未修改 `jcheng-manipulation` 核心代码、标签、证据或实验历史。
- 未定义 IDL/Schema；`jcheng-interfaces` 只有契约审计计划。
- 未创建 ROS 包、运行时代码、部署单元或机器人配置。
- 未连接机器人、未启动 ROS 节点、未发送运动命令。
- M1 只完成组织/仓库/文档/模板/CI 骨架。

## 7. 文件

| 文件 | 说明 |
|---|---|
| `M1-VERIFICATION.json` | 111 项逐项验证结果 |
| `M1-VERIFICATION.sha256` | 验证报告校验和 |
| `M1-EVIDENCE.json` | 组织/提交/工作流 URL 摘要 |
| `README.md` | 本文件 |

复现验证：

```bash
cd /home/jerichocheng/jcheng/jcheng-ecosystem-bootstrap
./scripts/verify_m1.sh
```
