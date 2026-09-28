# Robot Manipulator Kinematics (MATLAB)

MATLAB scripts that model four classic serial robot manipulators (**planar**, **anthropomorphic**, **cylindrical** and **spherical**). They cover three kinematics topics in order: **analytical inverse kinematics**, **symbolic Jacobian derivation**, and **Jacobian-based numerical inverse kinematics**. All scripts use Peter Corke's Robotics Toolbox for MATLAB.

> **Original implementation.** This repository keeps the project's original implementation. The source code was intentionally not refactored or modernized so that it keeps its historical context and the way it was first developed. All comments, identifiers and console messages are in Spanish, as originally written.

---

## Project Overview

| Script | Topic | Manipulators covered |
|---|---|---|
| [`src/PlanarManipulator2DOF.m`](src/PlanarManipulator2DOF.m) | Closed-form (analytical) inverse kinematics, checked with forward kinematics and a 3D plot | Planar 2-DOF (elbow up and elbow down), anthropomorphic 3-DOF (2 elbow configurations), cylindrical 3-DOF, spherical 3-DOF |
| [`src/act8.m`](src/act8.m) | Symbolic **geometric Jacobian** (linear and angular parts) | Planar 3-DOF, anthropomorphic 3-DOF, cylindrical 3-DOF, spherical 3-DOF |
| [`src/Act9.m`](src/Act9.m) | Symbolic linear-velocity Jacobian and **iterative inverse kinematics** using the Jacobian pseudo-inverse, with trajectory and joint-velocity plots | Planar 3-DOF, anthropomorphic 3-DOF, cylindrical 3-DOF, spherical 3-DOF |

## Project Context

- **Project origin:** Coursework / Assignment (**Inferred**, medium confidence). The file names `act8.m` and `Act9.m` match the Spanish *"Actividad 8 / Actividad 9"* naming used for graded activities. The content follows a standard undergraduate robotics syllabus: DH modeling, inverse kinematics, Jacobians and differential kinematics. No assignment statement, university name or course name is in the repository.
- **Author:** Omar Baruch Morón López (**Confirmed**, from the header comment in `PlanarManipulator2DOF.m` and `act8.m`, and from the `LICENSE` copyright holder "Baruch Lopez").
- **Date:** Uploaded to GitHub on **20 February 2021** (**Confirmed** by the git history). When the code was actually written cannot be determined.

See [`docs/project-context.md`](docs/project-context.md) for how confident each claim is and what evidence supports it.

## Problem Statement

For a given serial manipulator and a desired end-effector position, find the joint variables that reach it. Then study how joint velocities map to end-effector velocities through the Jacobian. The scripts answer this for four standard arm geometries, first in closed form and then numerically.

## Objective

*Inferred from the code:* practice and demonstrate core manipulator-kinematics techniques using the Robotics Toolbox:

1. Describe each robot with Denavit–Hartenberg (DH) parameters (`Revolute`, `Prismatic`, `SerialLink`).
2. Solve inverse kinematics analytically and check it with forward kinematics (`fkine`).
3. Derive Jacobians symbolically (Symbolic Math Toolbox).
4. Use the Jacobian to solve inverse kinematics numerically and look at how the joints converge.

## Repository Structure

```
.
├── README.md                 ← this file
├── LICENSE                   ← original MIT license (2021)
├── src/                      ← original MATLAB scripts (unchanged)
│   ├── PlanarManipulator2DOF.m
│   ├── act8.m
│   └── Act9.m
├── archive/                  ← near-duplicate historical copy (unchanged)
│   └── PlanarManipulator2DOF_2.m
├── docs/                     ← documentation added during the reorganization
│   ├── project-context.md
│   ├── code-overview.md
│   ├── kinematics-models.md
│   ├── possible-improvements.md
│   └── repository-history.md
├── specs/                    ← intent / spec / plan (spec-driven workflow)
│   ├── intent.md
│   ├── spec.md
│   └── plan.md
└── AGENTS.md                 ← rules for automated contributors (code is read-only)
```

[`docs/repository-history.md`](docs/repository-history.md) maps every original path to its new location.

## Original Implementation

The `.m` files in `src/` and `archive/` are **byte-for-byte identical** to the files uploaded in 2021. They were only moved, never edited. That includes their original CRLF line endings and ISO-8859-1 (Latin-1) encoding. The encoding is why accented words such as *antropomórfico* can look garbled in editors that assume UTF-8. A `.gitattributes` rule turns off end-of-line normalization for `*.m` files so they stay unchanged.

Known issues and possible modernizations are listed separately in [`docs/possible-improvements.md`](docs/possible-improvements.md). **None of them have been applied.**

## Technologies

| Technology | Evidence |
|---|---|
| MATLAB | `.m` scripts, cell-mode sections (`%%`), `disp`, `pause`, `figure`, `plot` |
| [Robotics Toolbox for MATLAB](https://petercorke.com/toolboxes/robotics-toolbox/) (Peter Corke) | `Revolute`, `Prismatic`, `SerialLink`, `fkine`, `A`, `plot`, `transl`, `tr2rt` |
| MATLAB Symbolic Math Toolbox | `sym`, `syms`, `simplify`, `diff`, `subs` |

**Toolbox version: Unknown.** In `Act9.m`, `fkine` results are indexed as plain 4×4 matrices (`Ti(1:3,4)`), which *suggests* a Robotics Toolbox release before 10. In release 10, `fkine` returns an `SE3` object. This cannot be confirmed.

## How It Works

1. **Model:** Every script builds each robot from its DH links with `SerialLink([L1 L2 L3])`.
2. **Analytical IK** (`PlanarManipulator2DOF.m`): the law of cosines gives the elbow angle (both ± branches, i.e. elbow up/down). `atan2`/`asin` give the shoulder angle. The script checks reachability, plots the arm at the solution and prints the forward-kinematics transform.
3. **Symbolic Jacobian** (`act8.m`): `Jv` is found by differentiating the end-effector position symbolically with respect to each joint variable. `Jw` is built from the z-axes of the joint frames (zero columns for prismatic joints). The script prints `J = [Jv; Jw]`.
4. **Numerical IK** (`Act9.m`): starting from an initial guess `q`, it repeats `q ← q + Δt · pinv(J) · K · (p_desired − p_current)` for 100 iterations with `Δt = 0.1` and `K = I`. Then it compares the desired and reached positions, animates the arm and plots joint positions and velocities.

The DH tables, formulas and numeric parameters for each robot are in [`docs/kinematics-models.md`](docs/kinematics-models.md).

## Inputs and Outputs

- **Inputs:** hard-coded at the top of each section (link lengths, target position `tx, ty, tz` or `td`, and the initial guess `q`). The scripts read no files.
- **Outputs:** console output (joint angles, transforms, symbolic Jacobians, desired vs. reached positions) and MATLAB figures (robot plots, joint-position and joint-velocity plots). The scripts write no files, and the repository contains no saved output.

## Running the Project

**Requirements (Confirmed by the code):** MATLAB, the Robotics Toolbox for MATLAB (Peter Corke) on the MATLAB path, and the Symbolic Math Toolbox (for `act8.m` and `Act9.m`).

```matlab
cd src
PlanarManipulator2DOF   % press any key at each pause() to go to the next manipulator
act8
Act9
```

Each script is split into `%%` sections and stops at `pause` between manipulators. You can also run them section by section in the MATLAB editor ("Run Section").

The exact MATLAB and toolbox versions used originally are **not documented**. The scripts have **not** been re-run as part of this reorganization.

## Documentation

- [Project context](docs/project-context.md): origin, scope and evidence (Confirmed / Inferred / Unknown)
- [Code overview](docs/code-overview.md): what each script does, section by section
- [Kinematics models](docs/kinematics-models.md): DH tables, equations and parameters per manipulator
- [Possible improvements](docs/possible-improvements.md): observations only, not applied
- [Repository history](docs/repository-history.md): original to new path mapping
- [Intent](specs/intent.md) · [Spec](specs/spec.md) · [Plan](specs/plan.md): spec-driven documentation of the project and of this reorganization

## Historical Note

This repository was later reorganized and documented to make it easier to read and to preserve the historical context of the original project. The original source code is unchanged. The files were only moved into `src/` and `archive/`, and all documentation was added afterwards.

## License

[MIT](LICENSE) © 2021 Baruch Lopez
