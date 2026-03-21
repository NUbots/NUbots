# FileWatcher

## Description

`FileWatcher` provides the `on<extension::FileWatch>(path, events)` DSL so modules can react to file-system changes.

It supports watching:

- A specific file
- A directory
- A glob pattern

The extension tracks matching files and runs your reaction when files are discovered, modified, renamed/created, or deleted (depending on the event mask you subscribe to).

## Usage

Install `module::extension::FileWatcher` in your powerplant, then use the `extension::FileWatch` DSL in any reactor.

### Basic file watch

```cpp
#include "extension/FileWatch.hpp"

on<extension::FileWatch>("config/MyModule.yaml", extension::FileWatch::CHANGED)
	.then([](const extension::FileWatch& watch) {
		// watch.path is the absolute path to the file that changed
		// watch.events is one of extension::FileWatch::Event
	});
```

### Watch a directory

Passing a directory path watches all files below that directory recursively.

```cpp
on<extension::FileWatch>("config", extension::FileWatch::CHANGED)
	.then([](const extension::FileWatch& watch) {
		// Triggered when a matching file in config/ changes
	});
```

### Watch a glob

Passing a non-existent path is treated as a glob pattern. Examples:

- `config/*.yaml`
- `module/**/README.md` (single `/**/` recursive segment supported)

The glob is translated internally to an ECMAScript regex.

### Event mask

Available events:

- `extension::FileWatch::NO_OP`: Initial discovery event for files already present when the watch is added
- `extension::FileWatch::RENAMED`: File created or renamed into a watched location
- `extension::FileWatch::CHANGED`: File contents changed
- `extension::FileWatch::DELETED`: File deleted or renamed out of a watched location

Use bitwise OR to subscribe to multiple event types.

```cpp
on<extension::FileWatch>(
	"config/**/*.yaml",
	extension::FileWatch::CHANGED | extension::FileWatch::RENAMED | extension::FileWatch::DELETED)
	.then([](const extension::FileWatch& watch) {
		// Handle create/rename, modify, and delete events
	});
```

### Startup synchronisation

`FileWatcher` emits `extension::FileWatcherReady` once its libuv loop is initialised. If you need to delay file operations until watches are active, trigger your startup logic from this event.

```cpp
on<Trigger<extension::FileWatcherReady>>().then([this] {
	// Safe place to kick off logic that depends on active file watches
});
```

## Emits

- `extension::FileWatcherReady`: emitted once when the watcher is initialised and ready

Note: `extension::FileWatch` is provided via the NUClear DSL/thread store and invokes matching reactions directly. It is not broadcast as a normal global message type.

## Dependencies

- `libuv` for cross-platform file-system notifications
