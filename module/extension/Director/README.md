# Director

## Description

See the Director NUbook page for details on how to use the Director in modules.

This module triggers on `BehaviourTasks` from the Behaviour extension and turns them into a `DirectorTask` TaskPack for the `run_task_pack` function. This function:

- Checks if the Provider is active, otherwise it cannot make subtasks
- Handles Done tasks
- Schedules any Wait tasks
- Clears any null tasks, clears root tasks
- Check priority changes and test challenges in the tree
- Removes tasks that were in the last task pack for this Provider, but are now gone
- Handle optional tasks
- Update subtasks for the Provider

## Usage

The easiest way to use the Director in a module is to generate the module using the `--director` flag:

```bash
./b module generate <module_name> --director
```

Check the Director NUbook page for information on functionality.

## Dependencies

- The Behaviour extension
