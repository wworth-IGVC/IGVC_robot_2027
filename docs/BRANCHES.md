# Branches in this fork

## Why there are 15 `2026/` branches

`Gold-Rush-Robotics/IGVC_robot_2026` has 15 branches and the competition work was
never merged into `main`. This fork originally descended from `main` alone, which
is a pre-competition snapshot from 2026-05-31.

Every upstream branch is mirrored here under a `2026/` prefix. They are **exact
mirrors**: nothing has been committed on top of them, so they can be diffed
against upstream at any time. The prefix keeps them clearly separate from active
2027 work and guarantees none can collide with `main`.

Mirroring cost nothing in disk terms; the objects were already present locally.

```bash
git fetch upstream --prune
for b in $(git for-each-ref --format='%(refname:strip=3)' refs/remotes/upstream/ | grep -v '^HEAD$'); do
    git branch --force "2026/$b" "upstream/$b"
done
git push origin 'refs/heads/2026/*:refs/heads/2026/*'
```

## Inventory

`isaac/exts` is pinned to three different commits across these branches, which is
the single most useful thing to know before checking one out.

| Branch | `isaac/exts` | ahead of `2026/main` | last commit |
| --- | --- | --- | --- |
| `2026/more_diverging_changes` | 72ff9c21 | **61** | 2026-06-01 |
| `2026/gui_testing` | 3d856e5c | 40 | 2026-05-26 |
| `2026/test_cases` | 72ff9c21 | 34 | 2026-05-22 |
| `2026/tuning_parameters` | 72ff9c21 | 34 | 2026-05-20 |
| `2026/real_robot_nav2` | 72ff9c21 | 30 | 2026-04-20 |
| `2026/test_comp_changes_isaac_smi` | 72ff9c21 | 15 | 2026-05-29 |
| `2026/yolo_ros` | 3d856e5c | 14 | 2026-05-22 |
| `2026/zed` | 72ff9c21 | 13 | 2026-05-12 |
| `2026/obk_and_no_mans_fixes` | 72ff9c21 | 10 | 2026-05-29 |
| `2026/outdoor_testing` | 72ff9c21 | 5 | 2026-05-25 |
| `2026/full_self_driving` | 375eddd1 | 3 | 2026-05-27 |
| `2026/yolo26` | 72ff9c21 | 1 | 2026-05-22 |
| `2026/lidar_testing` | 72ff9c21 | 1 | 2026-04-04 |
| `2026/main` | 375eddd1 | 0 | 2026-05-31 |
| `2026/vision_sense` | 375eddd1 | 0 | 2026-05-24 |

`2026/more_diverging_changes` is 61 ahead of `main` and 0 behind, ending
"final push +1" on 2026-06-01, and is a strict superset of
`test_comp_changes_isaac_smi`. It is the leading candidate for what ran at the
2026 competition. That question is still open for the team lead.

## The `isaac/exts` pin, and why it is fragile

`.gitmodules` points `isaac/exts` at `https://github.com/stereolabs/zed-isaac-sim`
on every branch. Ten branches pin it to `72ff9c21`, a commit that exists only in
`Gold-Rush-Robotics/zed-isaac-sim`, the organisation's fork.

**This resolves today.** Verified from a clean clone of
`2026/more_diverging_changes` with no config overrides:
`git submodule update --init --recursive` exits 0 and lands `isaac/exts` at
`72ff9c21`. GitHub shares object storage across a fork network, so a fetch of a
specific SHA succeeds even from the parent repository's URL.

It is worth knowing what that depends on:

- **It breaks under `--depth 1`.** Shallow fetch of an arbitrary SHA is refused.
  Any CI job or clone script using shallow checkout will fail on these branches.
- **It breaks if the organisation's fork is deleted.** The commit exists in no
  other repository. Nothing in this fork preserves it.
- **It does not survive mirroring off GitHub.** Push these branches to GitLab, a
  self-hosted server, or a `git bundle`, and the submodule can no longer resolve.

So the setup is not broken, but it rests on a fork this team does not control.

## Submodule pointers do not follow a checkout

Pointers differ between branches and git will not update them when you switch.
Set this once:

```bash
git config submodule.recurse true
```

Without it, `git checkout 2026/more_diverging_changes` leaves `isaac/exts` at
whatever `main` pinned, and the ZED extension silently loads the wrong code.

## The permanent fix, when a baseline is chosen

Do not repoint `isaac/exts` at the organisation's fork. It is 2 ahead but **8
behind** upstream, so adopting it trades eight upstream commits for two, and one
of those two lowers the OmniGraph `TARGET_VERSION` from (2, 184, 5) to
(2, 184, 2), pinning to an older Isaac Sim.

The change actually needed is roughly 25 lines in
`exts/sl.sensor.camera/sl/sensor/camera/annotators.py`, which extracts a
`left_`/`front_`/`right_` prefix from the prim name so the extension can find
three cameras in a URDF-imported hierarchy instead of the hardcoded
`/base_link/<model>/CameraLeft`.

Fork `stereolabs/zed-isaac-sim` under an account this team controls, apply that
one file on top of the current pin, and point `isaac/exts` there. Upstream
history is kept, every branch resolves from a single URL this team owns, and the
dependency on the organisation's fork continuing to exist goes away.
