# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

NUbots is the software stack for a team of RoboCup humanoid soccer-playing robots (currently the Booster K1). It is a
C++20 codebase built on [NUClear](https://github.com/Fastcode/NUClear), a reactive/pub-sub robotics framework, plus
NUsight2, a TypeScript/React web app for visualising and debugging robots in real time.

## Toolchain: everything goes through `./b`

Almost all development commands run through `./b` (a thin wrapper around `nuclear/b.py`, itself extended by scripts
in `tools/`). Most `b` subcommands are decorated with `@run_on_docker`, meaning **they transparently re-exec inside
the project's Docker build container** — you do not need to manually enter a container. `./b target <platform>`
selects/builds that container image (e.g. `./b target orinnx` for the robot's NVIDIA Orin NX, or `generic` for a dev
machine); it only needs to be run once until the image needs rebuilding.

### Build

```sh
./b configure                      # Configure via CMake (generates into ../build by default)
./b configure -i                   # Interactive ccmake TUI to pick which roles/modules to build
./b build                          # Build with ninja
./b build -j4                      # Limit parallel jobs (useful on memory-constrained machines)
./b build -m                       # Also delete/regenerate compiled protobuf message files
```

CMake cache options worth knowing: `-DBUILD_TESTS=ON`, `-DCI_BUILD=ON` (enables `-Werror`), `-DSUBCONTROLLER=Booster`
(the only current option), `-DBUILD_ALL_MODULES=ON` (build a role containing every module, used by `./b unused`).
Pass them after `--`, e.g. `./b configure -- -DBUILD_TESTS=ON`.

### Test (C++)

Tests use CTest and live under `shared/tests/` (utility/library tests) and per-module `test/` directories, wired up
via `nuclear/tests`. Requires configuring with `-DBUILD_TESTS=ON` first.

```sh
./b tests list                     # List available tests
./b tests run                      # Run all tests
./b tests run <substring>          # Run only tests whose name contains <substring>
./b tests run -j8                  # Parallel test execution
./b tests run -V / -VV             # Verbose / extra-verbose ctest output
```

(CI instead runs `ninja test` directly from the build directory after configuring with `-DBUILD_TESTS=ON`.)

### Run / install

```sh
./b run <role_binary> [args...]    # Run a built role binary inside the docker container
./b run --gdb <role_binary>        # Run under gdb
./b install -t -co -u booster <ROBOT_IP>   # Cross-compile-and-install a role onto a robot over ssh/rsync
```

Roles are declared as `<name>.role` files in `roles/` (e.g. `roles/robocup.role`) — see "Roles" below. On the robot,
run the resulting binary directly, e.g. `./robocup`.

### Format / lint

```sh
./b format                         # Auto-format C++ (clang-format), CMake (cmake-format), Python (black/isort), etc.
./b format --check --verbose       # Check-only mode, as used in CI
```

C++ style: 4-space indent, see `.clang-format` (also enforced via `.clang-tidy`, not currently run in CI). Python is
formatted with black (line-length 120) + isort; see `pyproject.toml`.

### Other useful `b` subcommands

```sh
./b module generate <path>              # Scaffold a new NUClear module at module/<path>
./b module generate <path> --director   # Scaffold a module wired up for the Director (Provide/Start/Stop/Needs)
./b edit <file>                         # Edit a file from inside the docker container context
./b shell                               # Open an interactive shell in the docker container
./b unused                              # Generate a role containing every module (used to find dead modules)
./b yarn <args>                         # Run yarn for NUsight2 inside docker, e.g. `./b yarn install`
```

## NUsight2 (`nusight2/`)

A yarn-managed TypeScript project. Run its scripts via `./b yarn <script>` so they execute inside the same Docker
environment as everything else; see `nusight2/CLAUDE.md` for details.

## Architecture

### NUClear reactors and the module tree

The system is a graph of independent **reactors** (C++ classes deriving from `NUClear::Reactor`) that communicate
purely by emitting and reacting to typed messages on a central "powerplant" — there is no direct calling between
modules. Each reactor lives in its own module directory under `module/`, organised by architectural layer — the
subdirectory names under `module/` are the layers, ordered roughly sensors → perception → decision → actuation.

A module `Foo` in namespace `module::layer` must live at `module/layer/Foo`, with a `src/Foo.hpp`/`.cpp` declaring
`class Foo : public NUClear::Reactor` (or `BehaviourReactor`, see below), plus optional `data/` (non-code files
copied to the build dir, typically a per-module YAML config in `data/config/`) and `test/` directories. Directory
structure is enforced by CMake globbing (see `nuclear/README.md` for the full spec) — after adding/removing a
module's files, rerun `./b configure`.

Most non-trivial modules have a `README.md` documenting their Description/Usage/Consumes/Emits/Dependencies — check
a module's README before modifying it, and keep it up to date when you change what a module consumes or emits.

### Roles: composing modules into executables

A **role** (`roles/*.role`) is a CMake-ish file listing which modules to compile into one executable, in
dependency/startup order (e.g. `roles/robocup.role` is the full on-robot game binary; `roles/test/*.role` are
narrow single-purpose roles for exercising one subsystem; `roles/webots/` builds sim variants). Order matters:
foundational modules (FileWatcher, SignalCatcher, log handlers, config) go first, hardware/platform IO modules go
last.

### The Director: behaviour orchestration

Higher-level behaviour (`strategy`, `purpose`, some `skill`/`planning` modules) is built on top of NUClear using the
**Director** extension (`module/extension/Director`, DSL in `shared/extension/Behaviour.hpp`), which layers a
priority-based behaviour-tree-like task system on top of plain reactors. For the DSL reference
(`Provide`/`Start`/`Stop`/`Task`/`Needs`/`When`/`Causing`/`Uses`) invoke the `nuclear-director` skill.

Modules using this pattern derive from `extension::behaviour::BehaviourReactor` rather than `NUClear::Reactor`
directly. Scaffold new ones with `./b module generate <path> --director`. This is the mechanism behind the
purpose/strategy/skill layers deciding, e.g., "walk to ball" vs "fall recovery" vs "get up" at any given moment.

### Shared code

Prefer reusing something from `shared/utility` (cross-module C++ helpers, tests mirrored under
`shared/tests/utility/...`) or `shared/extension` (NUClear DSL extensions) over duplicating logic in a module.
