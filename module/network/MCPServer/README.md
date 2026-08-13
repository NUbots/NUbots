# MCPServer

## Description

MCP, or Model Context Protocol is a system of defining tools in a way that AI agents can access and use them.
Similar to an API, MCP allows you to create a 'black box' that the agent can interact with for information or for actions.

MCP in our system is this module - a network host for MCP that can do such things as:

- Let AI see the camera
- Let AI emit `Walk` commands
- Let AI run commands onboard the Linux system

This allows the AI and user to collaboratively debug the NUbots system by actually testing the output, live in real time.

## Roadmap

- Getting localisation info, like the `Htw`

- Speak, using the implemented `GPT` skills

- Get gyro and accelerometer data?

## Usage

Including this module will start an MCP endpoint on the machine that allows Claude (or any other AI product) to arbitrarily emit messages and run commands on the machine.
Hence this module should not be included in roles where reliability is required, such as `robocup.role`, given AI will act stochastically and as it wants.

## Tools

- `get_status` - Returns a fixed "I am online." text response. No parameters.
- `walk` - Emits a `Walk` task. Parameters: `speed` (m/s, required, clamped to 0-0.3), `angle` (strafe angle in rad, +clockwise, required), `rotation` (yaw rate in rad/s, +clockwise, clamped to -1.0-1.0, defaults to 0).
- `look` - Emits a `Look` task at a point in torso space. Parameters: `x`, `y`, `z` (all required).
- `get_image` - Debayers the latest cached camera `Image`, JPEG-encodes it, and returns it as a base64 `ImageContent` block. No parameters. Returns a text message if no image has been received yet.
- `cmd` - Runs an arbitrary shell command on the robot via `popen` and returns its stdout. Parameters: `command` (required). Only executes if `allow_ace` is enabled in `MCPServer.yaml`; otherwise returns a text message telling the user to enable it.
- `log_to_user` - Logs a message via NUClear's logger at a chosen level. Parameters: `message` (required), `log_level` (required, one of `TRACE`/`DEBUG`/`INFO`/`WARN`/`ERROR`/`FATAL`).

## Consumes

- `message::input::Image`

## Emits

- `message::skill::Walk`
- `message::skill::Look`

## Dependencies

`nlohmann::json`
`mcp-cpp`
