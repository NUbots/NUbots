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

## Usage

Including this module will start an MCP endpoint on the machine that allows Claude (or any other AI product) to arbitrarily emit messages and run commands on the machine.
Hence this module should not be included in roles where reliability is required, such as `robocup.role`, given AI will act stochastically and as it wants.

## Consumes

- `message::input::Image`

## Emits

- `message::skill::Walk`
- `message::skill::Look`

## Dependencies

`nlohmann::json`
`mcp-cpp`
