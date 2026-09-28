# Project Context

[← Back to README](../README.md)

This page records what can and cannot be determined about the origin of the project. Each statement is labeled:

- **Confirmed**: backed directly by a file, the code or the git history.
- **Inferred**: a reasonable deduction from the repository that is not explicitly stated anywhere.
- **Unknown**: the repository does not give enough information to decide.

## Sources available

The original repository contained only:

- four MATLAB scripts (`.m`),
- an MIT `LICENSE`,
- two git commits ("Initial commit" and "Add files via upload"), both dated 20 February 2021.

It contained **no** PDF, Word, PowerPoint, image, dataset, notebook, generated output or README. So the context below comes entirely from the code, its comments, file and folder names, the license and the git metadata.

## Summary

| Aspect | Finding | Status |
|---|---|---|
| Author | Omar Baruch Morón López (`%% OMAR BARUCH MORON LOPEZ` in two scripts; `LICENSE`: "Baruch Lopez") | Confirmed |
| Upload date | 20 February 2021 (git history) | Confirmed |
| Date written | Could be earlier than the upload | Unknown |
| Language of code and comments | Spanish (e.g. *Cinemática inversa*, *Manipulador antropomórfico*) | Confirmed |
| Platform | MATLAB + Robotics Toolbox (Peter Corke) + Symbolic Math Toolbox | Confirmed |
| Project type | **Coursework / Assignment** | Inferred |
| Institution, course, instructor | Not mentioned anywhere | Unknown |
| Original assignment statement | Not included | Unknown |

## Why "Coursework / Assignment"?

Evidence for the classification (all **Inferred**):

1. `act8.m` and `Act9.m` follow the *"Actividad N"* naming commonly used for graded activities in Spanish-language university courses.
2. The four manipulators (planar, anthropomorphic, cylindrical, spherical) are the standard textbook set used in introductory robotics courses.
3. The scripts follow a typical syllabus order: analytical inverse kinematics → Jacobian → Jacobian-based inverse kinematics.
4. The same template is repeated for every manipulator. The author's name is printed as a header, as is usual for work handed in to an instructor.
5. The folder named "Planar manipulator 3dof **Complicated**" hints that `Act9.m` was seen as the more advanced exercise.

There is **no direct evidence** (syllabus, assignment PDF, university name) confirming a university context. The institution and course name remain **Unknown**.

## Scope of the project

**Confirmed from the code:**

- Kinematic modeling of four serial manipulators with DH parameters.
- Closed-form inverse kinematics with both elbow configurations where they apply.
- Simple reachability checks before solving.
- Symbolic derivation of the geometric Jacobian (linear and angular parts).
- Iterative inverse kinematics using the Moore–Penrose pseudo-inverse of the Jacobian.
- Visualization with the Robotics Toolbox (`SerialLink.plot`) and MATLAB plots of joint positions and velocities.

**Not in scope** (nothing in the repository covers these): dynamics, trajectory planning between waypoints, orientation control, hardware/ROS integration and automated tests.

## Relationship between the scripts

**Inferred:** the three main scripts are consecutive exercises from one course.

```
PlanarManipulator2DOF.m   → position level: analytical IK + FK check
        │
act8.m  (Activity 8)      → velocity level: symbolic geometric Jacobian
        │
Act9.m  (Activity 9)      → uses the Jacobian (derived again inside Act9.m) for numerical IK
```

The scripts do not call each other. Each one runs on its own and repeats the robot definitions it needs.

## Contradictions and ambiguities found

These are recorded here rather than resolved:

1. **Folder names do not match their content.** The original folder "Planar manipulator 2dof" also contained anthropomorphic, cylindrical and spherical 3-DOF sections. "Planar manipulator 3dof" (`act8.m`) also covered non-planar arms.
2. **"Different configuration" copy.** The folder "Planar manipulator 2dof diferent Configuration" held `PlanarManipulator2DOF_2.m`. It is identical to `PlanarManipulator2DOF.m` except that the author header line is missing. The folder name suggests a different configuration, but the file has the same content (both scripts already include both elbow configurations). Maybe a different version was meant to be uploaded, or maybe the copy is only a duplicate. This **cannot be determined**. The file is kept in [`archive/`](../archive/).
3. **Header of `Act9.m`.** Its first section title is "MANIPULADOR PLANAR DE 3 DOF", but the file covers all four manipulators, just like the other scripts.

## Encoding note

All original scripts use **ISO-8859-1 (Latin-1)** encoding and **CRLF** line endings. This is typical of MATLAB on Windows at the time. Accented characters (e.g. *antropomórfico*, *cilíndrico*, *CINEMÁTICA*) show up garbled in UTF-8 viewers such as the GitHub web UI. The files were intentionally left unchanged.
