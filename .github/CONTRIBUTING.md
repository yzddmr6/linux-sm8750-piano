# Contributing to the piano kernel

Send kernel issues and pull requests to
[linux-sm8750-piano](https://github.com/yzddmr6/linux-sm8750-piano), targeting `piano/dev`.
Device userspace, packaging and documentation belong in
[xiaomipad-8pro-mainline](https://github.com/yzddmr6/xiaomipad-8pro-mainline).

Follow the kernel's own `Documentation/process/`:
[submitting-patches](../Documentation/process/submitting-patches.rst),
[coding-assistants](../Documentation/process/coding-assistants.rst) and
[generated-content](../Documentation/process/generated-content.rst).
These govern commit attribution, DCO sign-off and tool-generated contributions.

Include the device model and panel variant, stock build, kernel release and commit,
integration commit or image identity, reproduction steps, results, sanitized logs and untested
conditions. Distinguish compilation and offline checks from tests on the device. Remove serials,
radio addresses, credentials and per-unit calibration before posting.

Maintainers allocate ABI numbers when packaging any installable kernel build, including
candidates. The integration repository's `kernel/abi.json` records all such allocations, not only
official releases. Ordinary PRs do not reserve numbers or edit that ledger or release locks;
coordinate packaging changes through an issue. See
[installable kernel packaging](https://github.com/yzddmr6/xiaomipad-8pro-mainline/blob/main/docs/RELEASE.md#installable-kernel-packaging-maintainers).
