# 参与 piano 内核开发

[English](#english)

## 改动发到哪里

- 内核驱动、设备树、binding 与内核配置片段（`arch/arm64/configs/` 下的 `piano-*`、`sm8750-*` 片段）：本仓 [linux-sm8750-piano](https://github.com/yzddmr6/linux-sm8750-piano)，目标分支 `piano/dev`。
- 设备用户态、打包、文档，以及发布用哪些配置片段、按什么顺序（`kernel/source.json`）：[xiaomipad-8pro-mainline](https://github.com/yzddmr6/xiaomipad-8pro-mainline)（`main`）。两边都要改时分别提 PR，互相链接并写明合入顺序。

提交、DCO 与来源归属遵守目标内核的 [submitting-patches](../Documentation/process/submitting-patches.rst) 和 [coding-assistants](../Documentation/process/coding-assistants.rst)。AI 协助写明实际使用的工具与模型、影响范围和可公开的验证事实，不附提示词、会话记录或任务说明。下面九条与集成仓 [CONTRIBUTING](https://github.com/yzddmr6/xiaomipad-8pro-mainline/blob/main/CONTRIBUTING.md) 相同，这里写内核侧的具体要求；不适用的在 PR 里写“不适用”和理由。

## 提交前逐项核对

1. **一个 PR 一个主题。** 每个提交一个逻辑改动，单独能编译、能二分；binding、驱动、板级启用分开提交；删掉已合入、被替代或无关的提交。
2. **基于 `piano/dev` 的最新提交。** 你的 `origin` 多半是自己的 fork：先从本项目仓库取最新的 `piano/dev`（`git remote add upstream https://github.com/yzddmr6/linux-sm8750-piano.git` 后 `git fetch upstream`），再 rebase、自己解决冲突。检查完整的最终差异（`git diff upstream/piano/dev...HEAD`）；发生过冲突的文件，对照重放前后的内容确认没有遗漏。`--stat` 只用来看范围。
3. **说的和做的一致。** 提交说明用英文，标题 `subsystem: summary`，正文写问题和为什么要改；真实修复才写 `Fixes:`，公开依据写 `Link:`；功能与修复的说法与最终差异和实际调用路径一致；修 bug 给出在当前 `piano/dev` 上的复现方法或源码依据。测试结论写明被测版本、场景和结果，rebase 或改代码后说明哪些旧的测试结果仍然适用。
4. **按发布方式构建。** 按集成仓 `kernel/source.json` 列出的全部发布配置片段、按顺序构建 Image、DTB 与模块，模块与内核版本匹配；构建后核对最终 `.config` 里本次需要的选项确实生效（依赖不满足、布尔选项写成 `=m`、符号名拼错都会被静默丢掉），受影响的既有选项没有退化；DTS 新引用的驱动、模块和用户态提供方在交付组合中闭合；不引入新的编译警告；对自己的系列跑 `scripts/checkpatch.pl` 并处理本次引入的错误；改了 binding 跑 `dt_binding_check`，改了 DTS 跑 `dtbs_check`。
5. **硬件参数有依据。** 寄存器值、时序、电压电流和固定地址给出原厂或上游依据，并对所有已支持的变体成立：两种屏（CSOT、BOE）、不同内存与存储容量、外设有或没有。固定的保留内存要落在所有内存容量机型的 RAM 范围内。新增设备树子节点时，检查所有会遍历该父节点子节点的现有驱动。改 SoC 共用代码要说明对其他板子的影响。
6. **真机回测。** 在自己的设备上 RAM 启动或安装改动后的内核，至少确认：冷启动进入登录界面或桌面；`dmesg` 与改动前相比没有新增 `Oops`、`BUG`、`WARNING`、`hung task`、`soft lockup` 或 `Call trace`；挂起后能唤醒，睡眠时长按墙上时间核对；熄屏再亮屏、锁屏解锁、长时间熄屏后唤醒时显示与触控正常；改动涉及的子系统真实可用；新增的可选功能缺失或失败时，不拖垮与它无关的既有功能（开机、触控、挂起）；probe 失败、remove 和电源状态切换的回收路径正确，必要依赖失败时正确报错。错误路径优先用源码审查或离线故障注入核对，不为勾选去卸载或重绑正在使用的硬件驱动。改动涉及电源或充电时，在电池供电下也测一次。没有把握的新功能默认关闭；不加没有协议依据的轮询、重试、延时或唤醒。
7. **署名与来源。** 用你自己的身份提交，并由你本人加 `Signed-off-by:`（DCO）。用了 AI 的，每个相关提交用尾注写明实际使用的模型：`Co-Authored-By: <模型> <邮箱>`（如 `Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>`）或内核 7.2 文档 `coding-assistants.rst` 的 `Assisted-by: <工具>:<模型版本>`，用了几个模型写几条；PR 里写明工具、模型和用在哪些部分，不附提示词或会话记录；已合入的提交不为补这一行而改写。AI 不加 `Signed-off-by:`，不替任何人加 `Tested-by:`、`Reviewed-by:`，不声称做过没做的测试。导入上游或社区补丁时保留原作者、原日期、原有尾注和来源 `Link:`，本地修改另成提交。
8. **不越权。** 普通 PR 不分配内核 ABI 编号，不改集成仓的 ABI 账本（`kernel/abi.json`）和发布锁；ABI 由维护者在打包任何可安装内核（包括测试用的构建）时分配。不替维护者宣布批准、测试结论或发布。
9. **不带个人数据。** 提交和默认日志输出里不能有序列号、蓝牙/Wi-Fi 地址、逐机标定和凭据（不要在默认日志级别用 `%pM`/`%pMR` 打印设备地址）。贴日志前先脱敏。

## PR 里要写的

按 [PR 模板](pull_request_template.md) 填写：问题与改动；主题、基于的 `piano/dev` 提交、配套的集成仓 PR 与合入顺序；来源与许可；AI 协助（工具、模型与用途）；构建与检查结果；真机验证——机型与屏型、内存与存储、原厂系统版本、`uname -r` 与提交、集成提交或镜像、测试方式、命令与结果、脱敏日志；没测到或有限制的部分。编译、离线检查和真机测试分开写。

维护者每个版本审一次待合入的 PR；有小问题时可能合入后自己补修、拆分或只摘取一部分，并在 PR 里说明。维护者整合后的组合按同一份清单验收。

<a id="english"></a>
## English

Send kernel drivers, device trees, bindings and kernel configuration fragments (the `piano-*` and `sm8750-*` fragments under `arch/arm64/configs/`) to this repository, targeting `piano/dev`; device userspace, packaging, documentation and the list of release fragments and their order (`kernel/source.json`) go to [xiaomipad-8pro-mainline](https://github.com/yzddmr6/xiaomipad-8pro-mainline) (`main`). Open one PR in each when both change, link them and state the merge order. Commits, the DCO and attribution follow the target kernel's `submitting-patches.rst` and `coding-assistants.rst`; AI disclosure names the tools and models actually used, their scope and public verification facts, without prompts, session logs or task notes. The nine checks match the integration repository's [CONTRIBUTING](https://github.com/yzddmr6/xiaomipad-8pro-mainline/blob/main/CONTRIBUTING.en.md); write "N/A" with a reason where one does not apply.

1. **One topic per PR**; each commit is one logical change that builds and bisects; split bindings, drivers and board enablement; drop merged, superseded or unrelated commits.
2. **Rebase onto the latest `piano/dev` of this project's repository** (your `origin` is usually a fork: `git remote add upstream https://github.com/yzddmr6/linux-sm8750-piano.git`, `git fetch upstream`), resolve conflicts yourself, review the complete final diff (`git diff upstream/piano/dev...HEAD`), and for files that had conflicts compare the content before and after the replay. `--stat` only shows the scope.
3. **Claims match the diff.** English commit messages with a `subsystem: summary` subject and a body that explains the problem and why; `Fixes:` only for real fixes, `Link:` for public references; feature and fix statements match the final diff and the real call path; a bug fix comes with a reproduction on the current `piano/dev` or a source reference. Test statements name the tested build, scenario and result; after a rebase or code change, say which evidence still applies.
4. **Build as releases are built**: every release fragment listed in the integration repository's `kernel/source.json`, in order, with matching Image, DTB and modules; check in the final `.config` that the options you need are really set (unmet dependencies, `=m` for a bool and misspelt symbols are dropped silently) and that affected existing options did not regress; drivers, modules and userspace providers that new DTS references need are present in the delivered combination; no new warnings; `scripts/checkpatch.pl` on your series with the errors you introduced fixed; `dt_binding_check` for binding changes and `dtbs_check` for DTS changes.
5. **Hardware parameters have a source**: stock or upstream references for register values, timings, voltages, currents and fixed addresses, valid for both panels (CSOT, BOE), every memory and storage size, and with or without each peripheral. Fixed reserved memory must lie inside RAM on every memory size. When adding a device-tree child node, check every existing driver that walks that parent's children. State the effect of SoC-wide changes on other boards.
6. **Test on the device** (RAM boot or installed kernel): cold boot to the login screen or desktop; no new `Oops`, `BUG`, `WARNING`, `hung task`, `soft lockup` or `Call trace` in `dmesg` compared with the build before your change; suspend and resume with sleep time checked against the wall clock; display and touch after screen off and on, lock and unlock, and a long screen-off; the changed subsystem working for real; unrelated functions (boot, touch, suspend) unaffected when a new optional feature is absent or fails; correct errors and cleanup on probe failure, remove and power-state changes. Check error paths by source review or offline fault injection rather than unloading or rebinding drivers of hardware in use. For power or charging changes, also test on battery. Keep uncertain features off by default; no polling, retries, delays or wakeups without protocol evidence.
7. **Attribution**: your own identity and your own `Signed-off-by:` (DCO). If you used AI, add a trailer to each affected commit naming the model you actually used: `Co-Authored-By: <model> <email>` (for example `Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>`) or the `Assisted-by: <agent>:<model version>` format of the kernel 7.2 `coding-assistants.rst`, one per model; name the tools, models and what they were used for in the PR, without prompts or session logs; merged commits are not rewritten to add this. An AI never adds `Signed-off-by:`, never adds `Tested-by:` or `Reviewed-by:` for anyone, and never claims tests that did not happen. Imported patches keep their author, date, trailers and source `Link:`; local changes go in separate commits.
8. **Stay within your role**: ordinary PRs do not allocate kernel ABI numbers or edit the integration repository's ABI ledger (`kernel/abi.json`) or release locks; maintainers allocate ABIs when packaging any installable kernel, including candidates. Do not announce approvals, test conclusions or releases on a maintainer's behalf.
9. **No personal device data** in commits or default log output: no serial numbers, Bluetooth or Wi-Fi addresses, per-unit calibration or credentials (do not print device addresses with `%pM`/`%pMR` at default log levels). Sanitise logs before posting.

Fill in the [PR template](pull_request_template.md) and keep compilation, offline checks and device tests apart. Maintainers review open PRs once per release; for small problems they may merge and fix, split, or take part of a PR, and will say so. The combination a maintainer integrates is checked against the same list.
