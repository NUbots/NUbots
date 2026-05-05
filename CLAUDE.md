# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project

NUbots is the University of Newcastle's RoboCup humanoid soccer team codebase. The robots run a NUClear-based C++ system; a TypeScript/React debugging UI (NUsight2) is bundled alongside. High-level docs live at https://nubook.nubots.net.

## The `./b` tool — entry point for everything

Almost all developer workflows go through the `./b` script in the repo root (it execs `nuclear/b.py` with `uv`). Subcommands are auto-discovered from `tools/*.py` (user) and `nuclear/tools/*.py` (NUClear). Run `./b` with no args to see the full list.

The build is intended to run inside the project Docker image — `./b` automatically dispatches the build/run/test commands into a container built from `docker/Dockerfile`. Don't try to compile on the host; configure and build through `./b`.

### Core workflow

```bash
./b target <name>           # Set the build target (e.g. generic, nugus1..7, webots). Persists in CMake cache.
./b configure               # Run CMake configure inside the docker image. Add -i for interactive ccmake.
./b build [-j N]            # Build the currently configured target. Pass extra args after `--`.
./b run <role>              # Build then run a role binary (e.g. `./b run robocup`, `./b run webots`).
./b test [filter]           # Build with BUILD_TESTS=ON and run the C++ test suite.
./b install <robot>         # rsync the built binaries + config to a robot (e.g. `./b install nugus1`).
./b shell                   # Drop into a shell in the build container.
./b format                  # Format C++/Python/CMake/protobuf in-place (uses clang-format, black, isort, cmake-format).
./b multi <role> ...        # Run multiple instances (e.g. for multi-robot simulation).
./b edit <module>           # Open a module in your editor.
./b module generate <path>  # Scaffold a new NUClear module under `module/<path>`.
```

`./b nbs ...` provides NUbook-format binary log (`.nbs`) tooling: `fuse`, `filter`, `trim`, `stats`, `json`, `video`, `extract_images`, `calibrate_cameras`, `rename_type`. Recordings live in `recordings/`.

`./b yarn ...` proxies yarn into the NUsight2 container. Localisation/odometry tuning entry points: `./b optimise_localisation`, `./b optimise_odometry`.

### Targets

The build target is set with `./b target <name>` and stored in the CMake cache. Common targets: `generic` (host/dev), `nugus<N>` (per-robot cross compile), `webots` (simulator). Re-run `./b configure` after changing target.

## Repository layout

- **`module/`** — C++ NUClear modules grouped by responsibility: `input/`, `output/`, `vision/`, `localisation/`, `actuation/`, `skill/`, `purpose/`, `strategy/`, `planning/`, `behaviour/`, `platform/`, `network/`, `support/`, `extension/`, `nbs/`, `tools/`. Each module is a self-contained directory with `src/`, `data/config/<Module>.yaml`, `tests/`, `CMakeLists.txt`, and a `README.md`.
- **`roles/`** — `.role` files declare which modules compose a runnable binary (`nuclear_role(... module::Foo ...)`). Order matters: `extension::FileWatcher`, `support::SignalCatcher`, and `support::logging::ConsoleLogHandler` always come first. Add a new module to a role here to make it run.
- **`shared/message/`** — Protobuf message definitions. These are the IPC currency between modules; running `./b configure` regenerates C++ headers + the NUsight2 TS bindings.
- **`shared/utility/`** — Header-only/library utilities shared across modules (math, kinematics, NUClear helpers, etc.).
- **`shared/extension/`** — NUClear extensions used by the codebase (e.g. `Configuration`, `Behaviour`).
- **`nuclear/`** — Vendored NUClearRoles build system: provides the `nuclear_role()`, `nuclear_module()`, `nuclear_message()` CMake macros, plus the base `b.py` runner.
- **`module/extension/Director/`** — The Director: NUbots' behaviour-coordination framework. `purpose/`, `skill/`, `strategy/`, and `planning/` modules emit/consume Director Tasks. Read `module/extension/Director/README.md` before editing behaviour modules.
- **`nusight2/`** — TypeScript/React debugging and visualisation UI (Vite + Rollup, vitest, Storybook). Connects to robots/Webots over the NUClearNet UDP protocol.
- **`docker/`** — The build/run image. `Dockerfile` plus baked-in system config under `docker/etc`, `docker/usr`, `docker/home`.
- **`tools/`** — `./b` subcommand implementations. Add a new top-level subcommand by dropping a `register(parser)` + `run(**kwargs)` Python module here.
- **`cmake/`** — Project CMake modules (Find* scripts for vendored deps).
- **`recordings/`** — `.nbs` log files; consumed by `./b nbs` tooling and NUsight2.

## Architectural concepts

**NUClear roles + reactions.** A role binary is a fixed set of NUClear modules linked together. Modules don't call each other directly — they `emit` and `on<Trigger<T>>` protobuf messages. Adding behaviour usually means: define/extend a message in `shared/message/`, write or modify a module in `module/`, then add it to one or more roles in `roles/`.

**Director-driven behaviour.** Higher-level decision making (`purpose`, `strategy`, `planning`, `skill`) is composed via the Director extension, which arbitrates task ownership and priority instead of using raw NUClear triggers. The Director is the right reading-target before changing soccer behaviour.

**Configuration.** Each module reads its YAML from `module/<Path>/<Module>/data/config/<Module>.yaml` via `extension::Configuration`. `extension::FileWatcher` (always loaded first) hot-reloads these on disk changes. Per-target overrides live alongside (e.g. robot-specific config files).

**Cross-language message contract.** Anything that needs to be visible in NUsight2, recorded to `.nbs`, or shared between modules must be a protobuf message in `shared/message/`. The build generates C++ and TypeScript bindings; both sides must be regenerated together.

## NUsight2 (the `nusight2/` subdirectory)

Yarn 1.x, Node ≥20. Common scripts (run from `nusight2/`, or via `./b yarn ...`):

```bash
yarn dev            # Dev server with virtual robots
yarn start          # Dev server connecting to real robots on the network
yarn test           # vitest run
yarn tscheck        # TypeScript type-check
yarn eslint         # Lint TS/TSX
yarn format         # prettier write
yarn protobuf       # Regenerate TS message bindings (auto-runs on install)
yarn storybook      # Component explorer
```

## Conventions

- C++17, gcc-13, CMake ≥ 3.20. Code is formatted with clang-format; run `./b format` rather than hand-formatting.
- Python tooling is pinned in `pyproject.toml` and run by `uv` via the `./b` shebang — adding a `dependencies.register(...)` call inside a tool module is how you pull in new Python deps for `./b` subcommands.
- Module READMEs are expected; the PR template asks you to update them when behaviour changes.
- Tests for a C++ module live in `module/<Path>/<Module>/tests/` and are included via `BUILD_TESTS=ON` (set automatically by `./b test`).
