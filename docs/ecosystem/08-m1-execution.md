# M1 执行记录

> 状态：**本地骨架完成；GitHub 侧待组织与 API 凭据解锁。**
> 日期：2026-11-23
> 范围：M1 只做组织/仓库初始化、文档、模板、CI 骨架。
> 基线：`jcheng-manipulation v1.0.0` 仍为 `b3411e4`，未改动。

## 1. 已遵守的附加约束

| 约束 | 执行情况 |
|---|---|
| 五层架构 + 三个首批仓库 | 本地骨架按 `.github`、`jcheng-interfaces`、`jcheng-system` 建立 |
| 单一运动入口 + 租约 + 安全否决 | 写入 `jcheng-system/docs/SAFETY.md` 与 M1 门禁；未实现运行时代码 |
| v1.0.0 基线不可变 | 未修改标签、清单、证据、实验历史 |
| 新仓库独立历史、公开、MIT 优先 | 三个独立 `git init`；许可证审查 pending，未添加 `LICENSE` |
| 分支保护实际配置并验证 | 配置/验证脚本已准备；GitHub 侧尚未解锁 |
| M1 不写核心代码、不实机 | 没有 ROS 包、运行时代码、部署单元或实机命令 |
| 契约审计后再定版 | `jcheng-interfaces/docs/CONTRACT_AUDIT_PLAN.md` 作为 M2 门禁 |

## 2. 本地交付物

暂存区：`/home/jerichocheng/jcheng/jcheng-ecosystem-bootstrap/`

| 仓库 | 本地目录 | 初始提交 |
|---|---|---|
| `.github` | `repos/.github` | `29e4aec` |
| `jcheng-interfaces` | `repos/jcheng-interfaces` | `8634491` |
| `jcheng-system` | `repos/jcheng-system` | `7600aad` |

三个仓库均：

- 独立 Git 历史，`main` 分支，无远程。
- 没有 `LICENSE`、没有 `package.xml`、没有 `.msg/.srv/.action`。
- 没有运行时代码、部署单元或机器人配置。
- 含 M1 门禁 `scripts/check_m1_gate.sh` 与 CI 骨架。

### 2.1 辅助工具

| 文件 | 作用 |
|---|---|
| `scripts/create_repos.sh` | 通过 GitHub API 创建三个仓库；可选 `--push` |
| `scripts/configure_branch_protection.sh` | 应用 `main` 保护与仓库设置 |
| `scripts/verify_m1.py` / `.sh` | 逐项验证组织、仓库、保护、检查、基线，生成 `M1-VERIFICATION.json` |
| `settings/branch-protection.json` | PR、CODEOWNERS、`m1-gate`、禁 force-push/删除、管理员受限 |
| `settings/repo-settings.json` | issues、squash-only、合并后删分支 |
| `docs/M1-RUNBOOK.md` | 解锁后的执行顺序与失败处理 |
| `docs/M1-ACCEPTANCE.md` | M1 验收清单 |
| `docs/LICENSE-REVIEW.md` | 许可证/第三方审查清单（pending） |

## 3. 本地验证结果

```text
.github gate:            OK
jcheng-interfaces gate:  OK (no IDL, no package, no LICENSE)
jcheng-system gate:      OK (no runtime code, no packages, no LICENSE)
JSON/YAML 语法:          0 errors
Markdown 相对链接:       0 broken
```

## 4. GitHub 侧阻塞

| 检查 | 结果 |
|---|---|
| `GET https://api.github.com/orgs/JCHENG-Robotics` | 404（未创建或私有不可见） |
| `gh` CLI | 未安装 |
| GitHub API 令牌 | 环境变量/常见凭据文件均未发现 |
| SSH | 可用，但只能推送到已存在仓库 |

因此本轮**没有创建**组织、仓库、远程、分支保护或任何 GitHub 资源。
这不是实现失败，而是缺少外部权限；本地骨架与脚本已准备好。

## 5. 解锁后的执行顺序

1. 在 GitHub 网页创建组织 `JCHENG-Robotics`（或确认私有组织存在）。
2. 创建 fine-grained PAT（`Administration: RW`、`Contents: RW`、`Metadata: R`），
   写入 `~/.config/jcheng/github-token`（`0600`）。
3. 运行：

```bash
cd /home/jerichocheng/jcheng/jcheng-ecosystem-bootstrap
./scripts/create_repos.sh --push
./scripts/configure_branch_protection.sh
./scripts/verify_m1.sh
```

4. 把 `M1-VERIFICATION.json`、首次 Actions 运行 URL、分支保护验证结果写入
   `jcheng-system/docs/evidence/org-init-YYYYMMDD/README.md`。
5. 许可证审查完成前，不添加 `LICENSE`、不宣称 MIT。

## 6. 基线完整性

- `v1.0.0` 仍指向 `b3411e4`；`release/manifest.json` 未变化。
- 本文件与设计文档仅新增到 `docs/ecosystem/`；没有修改核心代码。
- 没有连接机器人、没有启动 ROS 节点、没有发送运动命令。

## 7. 下一步

M1 剩余工作全部依赖 GitHub 组织与 API 凭据。解锁后按 `docs/M1-RUNBOOK.md`
执行并完成 `docs/M1-ACCEPTANCE.md`。M1 未完成前不进入 M2（契约定义）。
