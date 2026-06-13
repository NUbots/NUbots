# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

NUbots is a humanoid soccer-playing robot system for the RoboCup competition, developed at the University of Newcastle. The stack has three major layers:
- **C++20** robot control modules built on the NUClear reactive framework
- **TypeScript/React** visualization and debugging UI (NUsight2)
- **Python** build tooling via the `./b` script

External documentation: [NUbook handbook](https://nubook.nubots.net/) | [API docs](https://codedocs.nubots.net/)

---

## Build Commands (`./b` script)

All development goes through the `./b` script (symlink to `nuclear/b.py`, run via `uv`).

```bash
# Configure and build
./b configure --build-dir build    # Run CMake
./b build --build-dir build        # Compile (Ninja)

# Testing
./b tests run                      # Run all C++ tests
./b tests run --test-name <name>   # Run a specific test by name

# Code formatting
./b format --check                 # Check formatting (all languages)
./b format                         # Auto-fix formatting

# Module scaffolding
./b module generate <path>         # Create boilerplate for a new module
                                   # e.g.: ./b module generate input/MyCamera

# NUsight2 (from nusight2/ or via ./b)
./b yarn                           # Install JS dependencies
./b yarn dev                       # Dev server with virtual robots
./b yarn dev -t                    # Dev server without type-checking
./b yarn start                     # Production mode
./b yarn start --address <ip>      # Connect to a real robot
./b yarn storybook                 # Component library browser
./b yarn test                      # Run JS/TS tests

# Other tools
./b run <role>                     # Execute a role binary
./b unused                         # Find unused modules
./b docs                           # Generate Doxygen docs
./b install <target>               # Install to robot via rsync
```

**Important**: NUClear Roles uses CMake globbing to discover source files. If you **add or remove a file**, re-run `./b configure` so CMake picks up the change.

---

## Architecture

### NUClear Reactor Pattern

All robot logic is structured as **NUClear Reactors** (modules). A reactor:
- Lives in `module/<subsystem>/<Name>/src/<Name>.hpp` and `.cpp`
- Inherits from `NUClear::Reactor`
- Subscribes to messages via `on<Trigger<MessageType>>(...)` handlers
- Emits messages via `emit(std::make_unique<MessageType>(...))` 

Modules are loosely coupled through the message system — they never call each other directly.

### Module Directory Structure

```
module/<subsystem>/<Name>/
├── src/          # C++ source (.hpp required, named same as module)
├── data/         # Config files and non-source assets (copied to build dir)
└── test/         # Catch2 unit tests
```

### Roles (Executables)

A **role** is a `.role` file in `roles/` that declares which modules to compose into an executable:

```cmake
NUCLEAR_ROLE(
    support::logging::ConsoleLogHandler
    input::Camera
    vision::VisualMesh
    # ...
)
```

Key roles: `robocup.role` (competition), `keyboardwalk.role` (manual control), `demo.role`.

### Module Subsystem Hierarchy

Data flows roughly: **Input → Vision → Localisation → Strategy/Planning → Skill/Actuation**

| Subsystem | Purpose |
|---|---|
| `module/input/` | Cameras (GigE/Aravis), IMU, joint feedback, GameController |
| `module/vision/` | VisualMesh detector, YOLO, ball/goal/field line detection |
| `module/localisation/` | Robot pose estimation (NLopt), ball tracking, odometry |
| `module/strategy/` | Mid-level tactics (find ball, position on field) |
| `module/planning/` | Motion planning (walk path, kick, look, get-up) |
| `module/skill/` | Low-level skills (walk engine, kick execution) |
| `module/actuation/` | Kinematics, servo control, foot controller |
| `module/purpose/` | Top-level role behaviors (Soccer, Goalie, Defender) |
| `module/network/` | Multi-robot comms, NUsight data forwarding |
| `module/extension/` | NUClear DSL extensions (Director behavior graph, FileWatcher) |
| `module/support/` | Logging, configuration, signal handling |

### Shared Code (`shared/`)

- `shared/message/` — Protobuf `.proto` files that define all inter-module messages. These are compiled to C++ structs, Python bindings, and TypeScript types.
- `shared/utility/` — Header-only and compiled utility libraries (math, kinematics, algorithms) used across modules.
- `shared/extension/` — NUClear DSL extensions available to all modules.

### Message System (Neutron)

Messages are defined as `.proto` files in `shared/message/`. The build system generates:
- Simplified C++ structs (not raw protobuf classes) for use in reactors
- Python bindings via pybind11
- TypeScript types for NUsight2

### NUsight2 (`nusight2/`)

Browser-based visualization tool. Key structure:
- `src/client/` — React frontend (Three.js 3D visualization, charts)
- `src/server/` — Node.js server bridging NUClearNet to browser WebSocket
- `src/shared/` — Proto message types shared between client and server
- `src/virtual_robots/` — Simulated robot for development without hardware

### NBS Files

`.nbs` (NUClear Binary Stream) is the log file format for recorded message streams. Each frame: `☢ header | uint32 size | uint64 timestamp | uint64 type_hash | payload bytes` (all little-endian). Use `./b nbs` tools to inspect/decode them.

---

## Code Style

| Language | Formatter | Linter | Config |
|---|---|---|---|
| C++ | clang-format (Google style) | clang-tidy | `.clang-format`, `.clang-tidy` |
| Python | Black (120 char) + isort | pylint | `pyproject.toml` |
| TypeScript | Prettier (120 char) | ESLint | `.prettierrc` |
| CMake | cmake-format | — | `.cmake-format.py` |

Run `./b format --check` in CI; `./b format` to auto-fix locally.

---

## Key CMake Options

```bash
-DCMAKE_BUILD_TYPE=Release|Debug
-DBUILD_TESTS=ON          # Enable Catch2 unit tests
-DSUBCONTROLLER=NUSense   # Robot subcontroller (NUSense or OpenCR)
-DENABLE_CLANG_TIDY=ON    # Enable static analysis (slow)
-DUSE_ASAN=ON             # Address sanitizer
```

## Development Environment

The project is designed to build inside Docker (Arch Linux base). A `.devcontainer.json` is provided for VS Code remote container development. CI runs in the same Docker image.
