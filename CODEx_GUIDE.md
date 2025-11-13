# CODEx Collaboration Guide

This repository now follows a lightweight workflow so Codex (GPT-5) can ramp into the context quickly. Keep this file updated whenever expectations change.

## 1. Issue-First Workflow
- Draft an issue on GitHub before any feature or content change. Include summary, target files, task list, and acceptance criteria (see issue #1 for format).
- Reference the issue number in branch names, commit messages, and PR text using the `(#<issue>)` pattern.

## 2. Branch & Commit Conventions
- Branch naming: `issue-<n>-<short-description>` (e.g., `issue-1-add-new-pid-variants`).
- Always work off `main` unless a different base is specified.
- Stage files with `git add <paths>`; commit with a descriptive message such as `Add tuned PID variant programs (#1)`.
- Keep commits scoped to a single issue whenever possible.

## 3. Pushing & PR Flow
1. `git push origin <branch>` once commits are ready.
2. Open a PR against `main`, link the issue, and summarize the variants or changes added.
3. Include any testing or field notes in the PR description or linked documents before requesting review.

## 4. Files & Variants Tracking
- Current tracked variants:  
  - `src/main_encoder_only_corrected.py` – corrected geometry, encoder-only.  
  - `src/main_final_tuned.py` – most aggressive tuning, 540 RPM profile.  
  - `src/main_improved_stable.py` – conservative gains for reliability.  
  - `src/main_with_imu_compensation.py` – encoder + IMU fusion with drift feedforward.
- When adding new drive code, document the purpose of each variant so drivers know which to deploy.

## 5. Communication Tips
- Mention this guide in future Codex requests (“Follow CODEx_GUIDE.md”) so the assistant reads it immediately.
- Keep instructions plain ASCII to avoid parsing issues.
- If sandbox limits block Git operations, Codex will request escalated permissions automatically—no manual workaround needed.

Update this guide whenever you change processes or naming so future sessions stay in sync.
