# KeyboardSkills

## Description

Plays one K1 from the keyboard with the mjlab policies that `skill::Walk` runs: the walk and the six ball skills
(receive, dribble, ground kick, air shot, directional first touch, first-time pass). It is the NUbots counterpart of
NUSim's `./b skills`, but runs the policies on the NUbots side, over DDS, with NUbots' own ball observation.

A terminal has no key-release events, so keys step a command up and down rather than being held, as in
`purpose::KeyboardWalk`.

| Key             | Does                                                                                                 |
| --------------- | ---------------------------------------------------------------------------------------------------- |
| `1`             | Walk                                                                                                 |
| `2`             | Receive                                                                                              |
| `3`             | Dribble, starting held still (`start.dribble`)                                                       |
| `4` / `5`       | Ground kick / air shot, starting with `start.kick`. Switch from a dribble or with the ball in front  |
| `6` / `7`       | First touch / first-time pass of the next pass, starting with `start.first_touch` / `first_pass`     |
| `w` `s` `a` `d` | Walk: vx / vy by `walk_step`. Ball skills: the ball velocity command by `ball_step`                  |
| `z` `x`         | Walk: turn by `turn_step`                                                                            |
| `space`         | Zero the command (stop walking, or hold the ball)                                                    |
| `p`             | Roll a pass at the robot (NUSim only), and receive it unless a first touch or first-time pass is set |
| `b`             | Put the ball at rest at the robot's feet (NUSim only)                                                |
| arrows          | Head, while walking                                                                                  |
| `q`             | Quit                                                                                                 |

Ball commands are in the robot's yaw frame (x forward, y left). `skill::Walk` clamps them to the trained ranges.

## Usage

Run `nusim/keyboardskills` against NUSim (`./b run sim/soccer` there). With `interactive: false` and a `script` in
`KeyboardSkills.yaml` it plays the scripted keys with no terminal, printing the log.

## Consumes

Nothing; the keys and the script drive it.

## Emits

- `message::skill::Walk` Task, or `message::skill::BallSkill` Task, never both
- `message::skill::Look` Task while walking
- `message::strategy::FallRecovery` Task
- `message::platform::NUSimBallCommand` for `p` and `b`, which `input::NUSimGroundTruth` sends to NUSim

## Dependencies

- ncurses
- `skill::Walk` for both tasks
