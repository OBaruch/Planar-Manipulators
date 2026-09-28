# Repository History

[← Back to README](../README.md)

## Original upload (20 February 2021)

| Commit | Message |
|---|---|
| `4185069` | Initial commit (MIT `LICENSE`) |
| `c9179ed` | Add files via upload (the four MATLAB scripts) |

Original layout:

```
.
├── LICENSE
├── Planar manipulator 2dof/
│   └── PlanarManipulator2DOF.m
├── Planar manipulator 2dof diferent Configuration/
│   └── PlanarManipulator2DOF_2.m
├── Planar manipulator 3dof/
│   └── act8.m
└── Planar manipulator 3dof Complicated/
    └── Act9.m
```

## Reorganization

The files were moved with `git mv`, so `git log --follow` still shows their full history. **The content of every file is unchanged.** SHA-256 checksums were verified before and after the move:

| Original path | New path | SHA-256 |
|---|---|---|
| `Planar manipulator 2dof/PlanarManipulator2DOF.m` | [`src/PlanarManipulator2DOF.m`](../src/PlanarManipulator2DOF.m) | `aff95b1b0da63cbf79c3c324f5e4630d55642769ca4abd708c8267eb0b76a984` |
| `Planar manipulator 3dof/act8.m` | [`src/act8.m`](../src/act8.m) | `6ef152871096a31ed84f7c06b41d52eb82f1577af08eec455df892354f7355cb` |
| `Planar manipulator 3dof Complicated/Act9.m` | [`src/Act9.m`](../src/Act9.m) | `442402257ebe89376d5c1128314021460271306be89a3432a4bafe3b52b45778` |
| `Planar manipulator 2dof diferent Configuration/PlanarManipulator2DOF_2.m` | [`archive/PlanarManipulator2DOF_2.m`](../archive/PlanarManipulator2DOF_2.m) | `f279e8768a3223821caa7ab184f30d7a5d22788cadeceb6c289b56a113eeddcf` |

To verify:

```bash
sha256sum src/*.m archive/*.m
```

### Why this layout

- **`src/`**: the three distinct scripts. The original folder names were misleading (e.g. "Planar manipulator 2dof" also held 3-DOF arms), so they were not reused. Keeping the three scripts together in one flat folder is simplest for a project this size.
- **`archive/`**: `PlanarManipulator2DOF_2.m` differs from `src/PlanarManipulator2DOF.m` only by the missing author line. It is kept as a historical copy, not deleted, because its original folder name ("diferent Configuration") suggests it may once have been meant as a distinct version.
- **No `data/`, `assets/` or `docs/original/`**: the repository has no datasets, images or original documents to put there.

### Added files

`README.md`, `AGENTS.md`, `.gitignore`, `.gitattributes`, `docs/*.md`, `specs/*.md`. None of them affect how the scripts run.
