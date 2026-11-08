<!-- 目标分支 piano/dev。请按 .github/CONTRIBUTING.md 逐项填写，不适用的写“不适用”和理由，提交前删掉这段提示。
Target piano/dev. Fill in every section per .github/CONTRIBUTING.md, write "N/A" with a reason where a section does not apply, and delete this comment. -->

## 问题与改动 / Problem and change

<!-- 解决什么问题（链接 issue 与上游讨论），改了什么 / What problem (link the issue and upstream discussion), what changed -->

## 范围与基线 / Scope and base

- 主题（一个 PR 一个）/ Topic (one per PR):
- 基于的 `piano/dev` 提交 / Base commit on `piano/dev`:
- 配套的集成仓 PR 与合入顺序 / Companion integration PR and merge order:
- [ ] 已基于本项目仓库最新的 `piano/dev`，检查过完整的最终差异 / Rebased on this project's latest `piano/dev`; complete final diff reviewed

## 来源与许可 / Sources and licensing

<!-- 上游或社区补丁的链接、原作者与原有尾注；本地修改；没有导入内容写“无” / Links, original authors and trailers of imported patches; local changes; "none" if nothing was imported -->

## AI 协助 / AI assistance

<!-- 实际使用的工具和模型（如 Claude Code + Claude Opus 5.5）及用在哪些部分，与提交尾注（`Co-Authored-By:` 或 `Assisted-by:`）一致；没用写“无” / Tools and models actually used (for example Claude Code with Claude Opus 5.5) and what for, matching the commit trailers (`Co-Authored-By:` or `Assisted-by:`); "none" if none -->

## 构建与检查 / Build and checks

- 配置 / Configuration: <!-- `kernel/source.json` 列出的发布配置片段 / release fragments listed in `kernel/source.json` -->
- [ ] 最终 `.config` 中本次需要的选项已核对生效 / Required options verified in the final `.config`
- [ ] 没有新增编译警告 / No new build warnings
- [ ] checkpatch：本次引入的错误已处理 / checkpatch errors introduced here are fixed
- [ ] `dt_binding_check` / `dtbs_check`（改了 binding 或 DTS 时 / when bindings or DTS changed）

## 真机验证 / Device validation

- 设备 / Device: 型号、屏型（CSOT/BOE）、内存与存储、原厂系统版本 / model, panel, memory and storage, stock build
- 被测内核 / Tested kernel: `uname -r`、提交、集成提交或镜像 / `uname -r`, commit, integration commit or image
- 方式 / Method: RAM 启动 / 已装系统冷启动 — RAM boot / installed cold boot
- 命令与结果、脱敏日志 / Commands, results, sanitised logs:
- [ ] 冷启动 / Cold boot
- [ ] `dmesg` 无新增 Oops、BUG、WARNING、hung task / No new Oops, BUG, WARNING or hung task in `dmesg`
- [ ] 挂起与唤醒（墙上时间）/ Suspend and resume (wall clock)
- [ ] 显示与触控生命周期 / Display and touch lifecycle
- [ ] 改动涉及的子系统；可选功能缺失或失败时无关功能不受影响；probe 失败与 remove 路径 / The changed subsystem; unrelated functions unaffected by an absent or failing optional feature; probe failure and remove paths

## 没测到或有限制的 / Untested or limited
