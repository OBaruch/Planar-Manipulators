# Code Overview

[← Back to README](../README.md)

This page explains what each original script does. **The code itself was not modified.** Line numbers refer to the files as they are in `src/` and `archive/`.

All three scripts share the same structure:

- They start with `clear all` / `close all` / `clc` (or `clear; close; clc;`).
- They are split into MATLAB cell sections (`%%`), one per manipulator.
- Each section is self-contained: it redefines parameters, rebuilds the robot with `SerialLink` and prints its results.
- `pause` between sections waits for a key press before moving to the next manipulator.
- The scripts do not call each other, do not define functions and read or write no files.

---

## `src/PlanarManipulator2DOF.m`: analytical inverse kinematics

**Purpose (Confirmed):** compute closed-form inverse kinematics (IK) for a target position, then check the result by plotting the robot and computing forward kinematics (FK) with the Robotics Toolbox.

| Section (line) | Manipulator | What it does |
|---|---|---|
| L5 | Planar 2-DOF, configuration 1 | Target `(0.4, 0.4)`, `a1 = 0.35`, `a2 = 0.25`. Reachability check. `theta_2 = -acos(...)` (one elbow branch), `theta_1 = atan2(ty,tx) - asin(a2·sin θ2 / r)`. Plots the arm and prints `T02 = bot.fkine(q)`. |
| L28 | Planar 2-DOF, configuration 2 | Same target. Uses the other branch `theta_2 = +acos(...)`. |
| L49 | Anthropomorphic 3-DOF, configuration 1 | Target `(0.3, 0.2, 0.35)`, `d1 = 0.35`, `a2 = 0.3`, `a3 = 0.25`. `theta_1 = atan2(ty,tx)`. `theta_3 = +acos(...)`. `theta_2` from `atan2`/`asin` in the arm plane. Plot + `T03`. |
| L77 | Anthropomorphic 3-DOF, configuration 2 | Same, with `theta_3 = -acos(...)`. |
| L102 | Cylindrical 3-DOF (R-P-P) | Target `(0.5, 0.25, 0.8)`, `d1 = 0.35`, joint offsets `d2off = d3off = 0.15`. Computes `theta_1`, `d2`, `d3`. Plots with a fixed workspace `[-1 1 -1 1 -1 1]`. |
| L131 | Spherical 3-DOF (R-R-P) | Target `(0.5, 0.25, 0.8)`, `d1 = 0.35`, `d3off = 0.35`. Computes `theta_1`, `theta_2`, `d3`. Plots with a fixed workspace. |

**Observed details:**

- The alternative elbow solution in each section is kept as a commented-out line, so the two configurations are made by toggling the sign of `acos`.
- In the reachability checks, `disp` prints a warning in Spanish. Execution still continues, because there is no `return`.
- For the cylindrical and spherical plots, `q + 0.00000001` is passed to `bot.plot`. *Inferred:* this avoids an exact-zero joint value while plotting. The reason is not documented.
- Some IK assignments have no semicolon (e.g. `theta_1=...`), so their values are echoed to the console.

## `archive/PlanarManipulator2DOF_2.m`: near-duplicate

It is identical to `src/PlanarManipulator2DOF.m` except that the line `%% OMAR BARUCH MORON LOPEZ` is missing. It came from the original folder *"Planar manipulator 2dof diferent Configuration"*. See [project-context.md](project-context.md#contradictions-and-ambiguities-found) for why it was kept.

## `src/act8.m`: symbolic geometric Jacobian ("Actividad 8", inferred)

**Purpose (Confirmed):** build the 6×3 geometric Jacobian `J = [Jv; Jw]` symbolically for four manipulators.

Common procedure in every section:

1. Declare the joint variables and link parameters as symbolic (`sym('theta_1')`, `sym('a1')`, …).
2. Build the robot with symbolic DH parameters.
3. Compute `T01 = bot.A(1,q)`, `T02 = bot.A(1:2,q)` and `T03 = simplify(bot.fkine(q))`.
4. **Linear part `Jv`:** extract `[x,y,z] = transl(T03)` and differentiate each coordinate with respect to each joint variable (`diff`).
5. **Angular part `Jw`:** columns are `z0 = [0;0;1]`, and `z1`, `z2` are the third columns of the rotation parts of `T01` and `T02` (via `tr2rt`). For prismatic joints the script writes the column as `[0;0;0]`.
6. Substitute the twist angles (`alpha`) with numeric values via `subs` and print `J`.

| Section (line) | Manipulator | Joint variables | Symbolic parameters | Substitutions |
|---|---|---|---|---|
| L5 | Planar 3-DOF (R-R-R) | θ1, θ2, θ3 | a1, a2, a3 | none |
| L38 | Anthropomorphic 3-DOF (R-R-R) | θ1, θ2, θ3 | d1, a2, a3, al1 | `al1 = π/2` |
| L75 | Cylindrical 3-DOF (R-P-P) | θ1, d2, d3 | d1, al2 | `al2 = π/2`; `Jw = [z0 0 0]` |
| L108 | Spherical 3-DOF (R-R-P) | θ1, θ2, d3 | d1, al1, al2 | `al1 = π/2`, `al2 = -π/2`; `Jw = [z0 z1 0]` |

The script prints the symbolic Jacobian of each manipulator to the console. It makes no plots. The robot `plot` call is left commented out at the end of the `q = ...` lines.

## `src/Act9.m`: Jacobian-based numerical inverse kinematics ("Actividad 9", inferred)

**Purpose (Confirmed):** for each manipulator, (a) derive the linear-velocity Jacobian `Jv` symbolically, then (b) solve inverse kinematics numerically with the Jacobian pseudo-inverse and plot how the joints converge.

Every section has two parts.

**(a) Symbolic part:** as in `act8.m`, but only `Jv` is computed, from `T03(1:3,4)` after substituting the twist angles. The robot is named `'T800'`.

**(b) Numerical part ("CINEMÁTICA INVERSA"):**

```text
J    = anonymous function @(q) with the Jv expression typed in by hand
txyz = anonymous function @(q) with the end-effector position typed in by hand
rebuild the robot with numeric DH parameters
t = 0.1  (step),  K = eye(3)  (gain),  N = 100  (iterations)
J = J(q0)                       % Jacobian evaluated once at the initial guess
for i = 1:N
    e  = td - position(fkine(q))
    qp = pinv(J) * (K * e)      % joint velocities
    q  = q + qp * t             % Euler integration
    store q and qp
end
print final q and [td  txyz(q)]      % desired vs reached position
bot.plot(Q)                          % animate the joint trajectory
plot(Q)  -> "Posiciones",  plot(Qp) -> "Velocidades"
```

| Section (line) | Manipulator | Numeric parameters | Initial `q` | Target `td` |
|---|---|---|---|---|
| L1 | Planar 3-DOF | a1 = 0.35, a2 = 0.35, a3 = 0.25 | `[0, π/6, π/3]` | `[0.6, 0.5, 0.0]` |
| L64 | Anthropomorphic 3-DOF | α1 = π/2, d1 = 0.35, a2 = 0.3, a3 = 0.25 | `[π/6, π/6, π/3]` | `[0.25, 0.25, 0.5]` |
| L130 | Cylindrical 3-DOF | d1 = 0.35, α2 = π/2 | `[π/2, 0.8, 0.5]` | `[0.5, 0.25, 0.8]` |
| L197 | Spherical 3-DOF | α1 = π/2, α2 = -π/2, d1 = 0.35 | `[π, π/2, 0.45]` | `[0.5, 0.25, 0.5]` |

**Observed details:**

- `J = J(q)` is evaluated **once, before the loop**, and overwrites the function handle. The Jacobian is therefore not updated at each iteration (see [possible-improvements.md](possible-improvements.md)).
- For the planar arm the third row of the hand-written `J` is `[0, 0, 1]`. The true `z` row of `Jv` is all zeros for a planar arm, so this row keeps the matrix full rank. The reason is not documented (**Inferred**).
- The variable `t` is used both as the time step and, at the end, for the desired-vs-reached comparison matrix.
- In the spherical section the numerical part starts with a new cell (`%% CINEMÁTICA INVERSA`, line 221). The other sections use a plain comment (`% CINEMÁTICA INVERSA`).
- The final section does not end with `pause`.

## Dependencies observed

| Function / class | Provided by |
|---|---|
| `Revolute`, `Prismatic`, `SerialLink`, `.fkine`, `.A`, `.plot`, `transl`, `tr2rt` | Robotics Toolbox for MATLAB (Peter Corke) |
| `sym`, `syms`, `simplify`, `diff`, `subs` | Symbolic Math Toolbox |
| `pinv`, `eye`, `zeros`, `atan2`, `acos`, `asin`, `plot`, `figure`, `legend` | Core MATLAB |
