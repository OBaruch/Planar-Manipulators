# Possible Improvements

[← Back to README](../README.md)

> **Not applied.** The items below are observations made while documenting the repository. **None of them were implemented.** The source code is kept exactly as originally written, to preserve the historical implementation. If you want to act on these, do it in a separate, clearly labeled derivative, not in `src/`.

The items are written as *observations*. They have not been confirmed by running the code, because the scripts were not executed during this reorganization.

## Possible defects

| # | File / location | Observation | Likely effect |
|---|---|---|---|
| 1 | `PlanarManipulator2DOF.m`, anthropomorphic sections (L69, L94) | Link 3 is built with `'a',a2` instead of `'a',a3`. | The plotted robot and `fkine` use a third link of 0.30 m while the IK assumed 0.25 m, so the FK check will not reproduce the target. |
| 2 | `PlanarManipulator2DOF.m`, cylindrical section (L118) | `d3=sqrt(tx^2+ty^2-d3off)` subtracts the offset inside the square root. | The consistent form would probably be `sqrt(tx^2+ty^2)-d3off`. The computed extension is slightly wrong. |
| 3 | `PlanarManipulator2DOF.m`, reachability checks (2-DOF/anthropomorphic) | Only the outer workspace boundary is checked (`> 2·a1·a2`). The inner boundary (`< −2·a1·a2`) is not. | Targets that are too close to the base are not reported and make `acos` return complex values. |
| 4 | `PlanarManipulator2DOF.m`, all checks | An unreachable target only triggers `disp`. Execution continues. | Complex or `NaN` joint values can reach `plot`/`fkine`. |
| 5 | `Act9.m`, all sections | `J=J(q)` is evaluated once before the loop. | The iteration uses a fixed Jacobian. It may converge slowly, not at all, or to a nearby point instead of the target. The standard algorithm re-evaluates `J(q)` at each step. |
| 6 | `Act9.m`, planar section | The hand-written Jacobian has a third row `[0, 0, 1]`, which is neither the `z` linear row nor the `ωz` row `[1 1 1]`. | It works as a regularizing constraint on θ3 rather than a physical quantity. |
| 7 | `Act9.m`, all sections | `t` is the time step and is then overwritten with the comparison matrix `[td txyz(q)]`. | Harmless within each section, but confusing. |
| 8 | `Act9.m` | The Jacobian and position expressions are typed in by hand even though the symbolic versions were just computed. | Transcription errors are possible. `matlabFunction(Jv, ...)` would generate them automatically. |

## Code quality and maintainability

- **Duplication:** each manipulator section repeats the same robot definition, Jacobian construction and IK loop. Parameterized functions (e.g. `jacobian_ik(bot, J, q0, td)`) would remove this.
- **Script-level `clear all`:** clears breakpoints and cached functions, and slows execution. `clear` or function workspaces are usually preferred.
- **Magic numbers:** step size, gain, iteration count and link lengths are hard-coded in every section.
- **No stopping criterion:** the IK loop always runs 100 iterations instead of stopping when `norm(e)` is below a tolerance.
- **`q + 0.00000001` plotting workaround:** undocumented. A comment or an explicit tolerance would make the intent clear.
- **Mixed naming:** `theta_1` in `act8.m` / `PlanarManipulator2DOF.m` and `theta1` in `Act9.m`. Spanish and English are mixed in identifiers and folder names.
- **Encoding:** ISO-8859-1 source files show garbled accents on GitHub. Converting to UTF-8 would fix this, but it would change the original bytes, so it was not done.

## Modernization options

- **Robotics Toolbox 10+ / Robotics System Toolbox:** newer RTB releases return `SE3` objects from `fkine` and `A`. Code that indexes the result as a matrix (`Ti(1:3,4)`) would need `.t` / `.T`. The MathWorks Robotics System Toolbox (`rigidBodyTree`, `inverseKinematics`) is another option.
- **Python port:** the same models could be written with [`roboticstoolbox-python`](https://github.com/petercorke/robotics-toolbox-python) (`DHRobot`, `RevoluteDH`, `PrismaticDH`) and SymPy for the symbolic Jacobians.
- **Damped least squares:** replacing `pinv(J)` with `Jᵀ(JJᵀ + λ²I)⁻¹` would make the numerical IK robust near singularities.
- **Automated checks:** an FK-of-IK round-trip test per manipulator would catch items 1 and 2 above.
