# AGENTS.md

Guidance for automated coding assistants and other contributors working in this repository.

## Project in one line

A historical set of MATLAB scripts (2021) on robot manipulator kinematics: analytical inverse kinematics, symbolic Jacobians and Jacobian-based numerical inverse kinematics. See [README.md](README.md).

## Hard rules

1. **`src/` and `archive/` are read-only.** Do not edit, reformat, re-encode, rename or "fix" any `.m` file. They are preserved byte for byte, including known bugs.
2. **Do not change line endings or encoding.** The `.m` files are ISO-8859-1 with CRLF. `.gitattributes` (`*.m -text`) must stay in place.
3. **Verify integrity** after any change that touches these folders:
   ```bash
   sha256sum src/*.m archive/*.m   # must match docs/repository-history.md
   ```
4. **Do not invent context.** Label claims as *Confirmed*, *Inferred* or *Unknown*, as in [docs/project-context.md](docs/project-context.md).
5. **Do not add infrastructure** (CI, Docker, build or test tooling) unless there is an explicit intent for it.

## Where things go

| Change | Location |
|---|---|
| Documentation about the original project | `docs/` |
| Suggested fixes or modernizations (as text only) | `docs/possible-improvements.md` |
| New intent / spec / plan for future work | `specs/` |
| Any modernized or fixed code | A new, separate folder (e.g. `modernized/`) or repository, never `src/` |

## Workflow

Follow the spec-driven flow: update [specs/intent.md](specs/intent.md) → [specs/spec.md](specs/spec.md) → [specs/plan.md](specs/plan.md) before implementing anything non-trivial. Work on a feature branch and open a pull request against `main`.

## Running

Requires MATLAB, the Robotics Toolbox for MATLAB (Peter Corke) and the Symbolic Math Toolbox. There is no automated test suite. The scripts are interactive (`pause` between sections).
