# Intent

[← Back to README](../README.md) · **Intent** · [Spec](spec.md) · [Plan](plan.md)

> Part of a spec-driven workflow: **intent** (why) → **spec** (what) → **plan** (how). These documents were written **after the fact**, reconstructed from the existing repository. They describe the original project and the later reorganization. They do not describe new functionality.

## 1. Intent of the original project (reconstructed)

**Status: Inferred.** No original assignment statement exists in the repository.

> Learn and demonstrate the kinematics of serial robot manipulators by modeling four standard arm geometries in MATLAB and solving their position and velocity kinematics analytically, symbolically and numerically.

### Why it existed

- To practice the core tools of an introductory robotics course: DH modeling, forward and inverse kinematics, and Jacobians. The inference comes from the `act8`/`Act9` "activity" naming and the textbook set of manipulators (see [project-context.md](../docs/project-context.md)).
- To check hand-derived formulas against the Robotics Toolbox (`fkine`, `plot`).

### Who it was for

- The author, Omar Baruch Morón López (Confirmed), and presumably a course instructor (Inferred).

### What success looked like (inferred)

- For a target point, each manipulator is plotted at a configuration that reaches it.
- The symbolic Jacobian of each manipulator is printed.
- The numerical IK drives the end effector toward the target, with plots of joint positions and velocities.

## 2. Intent of the repository reorganization (2026)

> Modernize the repository, not the project.

### Goals

1. Make the project understandable without opening every file.
2. Present it professionally as a historical entry in a technical portfolio.
3. Preserve the original implementation **byte for byte**, including its bugs, style and encoding.
4. Separate original material from documentation added later.
5. Record clearly what is Confirmed, Inferred and Unknown.

### Non-goals

- Fixing, optimizing, reformatting or re-encoding any `.m` file.
- Adding infrastructure (CI, Docker, test frameworks, package managers, linters).
- Inventing context (university, course, dates) that the repository does not support.
- Making the project look bigger or more "enterprise" than it is.

### Stakeholders

- **Owner / author:** wants a clean, honest portfolio artifact.
- **Readers** (recruiters, engineers, students): want to understand quickly what was built and how.
- **Future contributors, human or automated:** need explicit guardrails. See [AGENTS.md](../AGENTS.md).
