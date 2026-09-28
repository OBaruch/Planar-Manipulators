# Spec

[← Back to README](../README.md) · [Intent](intent.md) · **Spec** · [Plan](plan.md)

> This spec is reverse-engineered from the existing code. Part A describes the behavior of the original scripts **as implemented**, including known quirks. Part B specifies the repository reorganization. Nothing here asks for the original code to change.

---

## Part A: Functional specification of the original scripts

### A.1 System context

| Item | Value | Status |
|---|---|---|
| Runtime | MATLAB (interactive, script mode) | Confirmed |
| Libraries | Robotics Toolbox for MATLAB (P. Corke); Symbolic Math Toolbox | Confirmed |
| Library versions | Probably Robotics Toolbox < 10 (matrix-returning `fkine`) | Inferred |
| Inputs | Hard-coded constants in each script section | Confirmed |
| Outputs | Console text and MATLAB figures; no files | Confirmed |

### A.2 Manipulator models

The scripts model four manipulators. DH tables are in [kinematics-models.md](../docs/kinematics-models.md).

| ID | Manipulator | Joint types |
|---|---|---|
| M1 | Planar | R-R (2-DOF) and R-R-R (3-DOF) |
| M2 | Anthropomorphic | R-R-R |
| M3 | Cylindrical | R-P-P |
| M4 | Spherical | R-R-P |

### A.3 Functional requirements (as implemented)

**`src/PlanarManipulator2DOF.m`: analytical IK**

- **FR-1.1** For M1 (2-DOF) and M2, compute the joint angles for a fixed target with the closed-form law-of-cosines solution, once per elbow branch (2 configurations each).
- **FR-1.2** For M3 and M4, compute the joint variables for a fixed target in closed form (one solution each).
- **FR-1.3** Before solving, check an approximate reachability condition and print a Spanish warning if it fails. Execution continues either way.
- **FR-1.4** Build the robot with `SerialLink`, plot it at the solution and print the homogeneous transform `bot.fkine(q)`.
- **FR-1.5** Pause between manipulators until a key is pressed.

**`src/act8.m`: symbolic Jacobian**

- **FR-2.1** For M1 (3-DOF), M2, M3 and M4, build the robot with symbolic DH parameters and joint variables.
- **FR-2.2** Compute `Jv` by symbolic differentiation of the end-effector position from `fkine`.
- **FR-2.3** Compute `Jw` from the joint z-axes (`z0`, `z1`, `z2`), using zero columns for prismatic joints.
- **FR-2.4** Substitute the twist angles numerically and print `J = [Jv; Jw]` (6×3).

**`src/Act9.m`: numerical IK**

- **FR-3.1** For each manipulator, derive and print the symbolic `Jv`.
- **FR-3.2** Use hand-written numeric `J(q)` and `p(q)` functions and the numeric DH robot.
- **FR-3.3** Iterate `q ← q + Δt · pinv(J)·K·(p_d − p(q))` with `Δt = 0.1`, `K = I₃`, `N = 100`, where `J` is evaluated once at `q₀`.
- **FR-3.4** Print the final `q` and a 3×2 matrix `[p_d, p(q_final)]`.
- **FR-3.5** Animate the robot along the joint history and plot the joint positions ("Posiciones") and joint velocities ("Velocidades") per iteration.

### A.4 Known behavioral quirks (to be preserved, not fixed)

Documented in [possible-improvements.md](../docs/possible-improvements.md): wrong link length in the anthropomorphic plot model, the cylindrical `d3` formula, one-sided reachability checks, the fixed Jacobian in the numerical IK, and the synthetic third row in the planar Jacobian.

### A.5 Non-functional characteristics

- Language of UI and comments: Spanish.
- File encoding: ISO-8859-1, CRLF line endings.
- Structure: cell-mode scripts; no functions, no tests, no configuration files.

---

## Part B: Specification of the repository reorganization

### B.1 Requirements

| ID | Requirement | Acceptance criterion |
|---|---|---|
| RR-1 | Original `.m` files are unchanged | SHA-256 of each file equals the value in [repository-history.md](../docs/repository-history.md) |
| RR-2 | History of moved files is kept | Files moved with `git mv`; `git log --follow src/<file>` shows the 2021 commit |
| RR-3 | EOL/encoding cannot be normalized by git | `.gitattributes` sets `*.m -text` |
| RR-4 | Source, archive and documentation are separated | `src/`, `archive/`, `docs/`, `specs/` exist; no original file left in the root except `LICENSE` |
| RR-5 | README gives a full overview | README has Overview, Context, Problem, Objective, Structure, Original Implementation, Technologies, How It Works, I/O, Running, Documentation, Historical Note |
| RR-6 | Evidence levels are explicit | Context claims are labeled Confirmed / Inferred / Unknown |
| RR-7 | No invented context | No university, course or date appears unless it is found in the repository |
| RR-8 | Improvements are kept separate | Suggestions are only in `docs/possible-improvements.md`, which is marked "not applied" |
| RR-9 | No unnecessary infrastructure | No CI, Docker, build or test tooling added |
| RR-10 | All relative links in the Markdown resolve | Every relative link resolves to an existing file |
| RR-11 | Guardrails for automated contributors | `AGENTS.md` states that `src/` and `archive/` are read-only |
| RR-12 | Stale branches are audited | Remote branches reviewed; obsolete ones removed |
