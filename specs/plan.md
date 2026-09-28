# Plan

[← Back to README](../README.md) · [Intent](intent.md) · [Spec](spec.md) · **Plan**

> Implementation plan for the repository reorganization described in [spec.md](spec.md) Part B. Every task leaves the original source code unchanged.

## Phase 0: Audit (read-only)

- [x] List every file in the repository and the git history (2 commits, 20 Feb 2021).
- [x] Look for PDFs, Word/PowerPoint files, images, datasets, notebooks and outputs. **None found.**
- [x] Read all four MATLAB scripts in full, including comments and section headers.
- [x] Diff the two 2-DOF scripts. They are identical except for the author line.
- [x] Detect the encoding and line endings (ISO-8859-1, CRLF).
- [x] Record SHA-256 checksums of all `.m` files.
- [x] Audit branches and pull requests. Remote: only `main`. No open PRs. The local working branch had no unique commits, so it was deleted as obsolete.

## Phase 1: Restructure (RR-1 to RR-4)

- [x] Create the `docs/repository-refactor` branch from `main`.
- [x] `git mv` the three distinct scripts to `src/`.
- [x] `git mv` the near-duplicate `PlanarManipulator2DOF_2.m` to `archive/`.
- [x] Remove the now-empty original folders (automatic after the moves).
- [x] Add `.gitattributes` (`*.m -text`) to prevent EOL normalization.
- [x] Add a MATLAB-oriented `.gitignore`.
- [x] Check that the checksums are unchanged after the moves.

## Phase 2: Document (RR-5 to RR-8)

- [x] `README.md`: overview, context, structure, how it works, running, historical note.
- [x] `docs/project-context.md`: origin and evidence table (Confirmed / Inferred / Unknown), contradictions.
- [x] `docs/code-overview.md`: section-by-section walkthrough of each script.
- [x] `docs/kinematics-models.md`: DH tables and equations taken from the code.
- [x] `docs/possible-improvements.md`: observations, explicitly not applied.
- [x] `docs/repository-history.md`: old to new path mapping with checksums.
- [x] `specs/intent.md`, `specs/spec.md`, `specs/plan.md`: this document set.
- [x] `AGENTS.md`: guardrails for automated contributors (RR-11).

Deliberately **not** created:

- `docs/assignment.md`: no assignment statement exists.
- `docs/architecture.md`: three standalone scripts with no shared components do not justify an architecture document. The relationships are described in `project-context.md`.
- `docs/original/`, `data/`, `assets/`: there is no content for them.

## Phase 3: Verify

- [x] `sha256sum src/*.m archive/*.m` matches [repository-history.md](../docs/repository-history.md).
- [x] `git diff --stat main -- '*.m'` shows renames only (100% similarity).
- [x] All relative Markdown links resolve (RR-10).
- [x] No CI, Docker or tooling files added (RR-9).

## Phase 4: Deliver

- [x] Commit with descriptive messages.
- [x] Push the `docs/repository-refactor` branch and open a pull request against `main`.

## Future work (out of scope, needs a separate intent)

A modernized derivative, such as a fixed MATLAB version or a Python port, should be developed **outside `src/`** (for example in a separate `modernized/` folder or repository), with its own intent/spec/plan. [possible-improvements.md](../docs/possible-improvements.md) is the input backlog.
