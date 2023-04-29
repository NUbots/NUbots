# KeyboardWalk

## Description

Starts the Director graph for the KeyboardWalk scenario.

Keyboard walk uses **keyboard inputs** to control the robot. The inputs available are detailed in the following table.

| Command      | Description                                                                      |
| ------------ | -------------------------------------------------------------------------------- |
| <kbd>e</kbd> | Toggles the walk on and off. Initially it is off.                                |
| <kbd>w</kbd> | Adds 0.01 to the walk command x-value. This value is in meters/second.           |
| <kbd>s</kbd> | Adds -0.01 to the walk command x-value. This value is in meters/second.          |
| <kbd>a</kbd> | Adds 0.01 to the walk command y-value. This value is in meters/second.           |
| <kbd>d</kbd> | Adds -0.01 to the walk command y-value. This value is in meters/second.          |
| <kbd>z</kbd> | Adds 0.1 to the walk command rotational value. This value is in radians/second.  |
| <kbd>x</kbd> | Adds -0.1 to the walk command rotational value. This value is in radians/second. |
| <kbd>,</kbd> | Runs the kick with the left foot.                                                |
| <kbd>.</kbd> | Runs the kick with the right foot.                                               |
| <kbd>g</kbd> | Runs the get up.                                                                 |
| <kbd>←</kbd> | Head turns to the left.                                                          |
| <kbd>→</kbd> | Head turns to the right.                                                         |
| <kbd>↑</kbd> | Head turns upwards.                                                              |
| <kbd>↓</kbd> | Head turns downwards.                                                            |
| <kbd>r</kbd> | Resets keyboardwalk. Head rotation is set to 0. Walk command is set to 0.        |
| <kbd>q</kbd> | Quits keyboardwalk.                                                              |

## Usage

Include this in your role to start the Director tree to run KeyboardWalk.

## Consumes

## Emits

- `message::strategy::StandStill` to make the robot still while not walking
- `message::behaviour::state::Stability` to set the robot's initial stability state
- `message::strategy::FallRecovery` to enable getting up when fallen
- `message::skill::Kick` to kick with left/right leg
- `message::skill::Look` to look in desired direction
- `message::skill::Walk` to walk with desired velocity

## Dependencies
