# NUbots Codebase

![Main Build](https://github.com/NUbots/NUbots/actions/workflows/nubots.yaml/badge.svg?branch=main)

![NUsight2 Build](https://github.com/NUbots/NUbots/actions/workflows/nusight.yaml/badge.svg?branch=main)

![Image Build](https://github.com/NUbots/NUbots/actions/workflows/images.yaml/badge.svg?branch=main)

NUbots is a team in the University of Newcastle's robotics research group focused on developing humanoid soccer-playing robots for the international RoboCup competition.
The multidisciplinary team of students and academics develops both hardware and software for the robots.

This codebase is the current state of our development efforts to compete in the competition as well as active research projects within the laboratory.

# NUbots Documentation and Handbook

[NUbook](https://nubook.nubots.net/) is the handbook and high-level documentation for the NUbots team.
It contains guides for [getting started with development](https://nubook.nubots.net/guides/main/getting-started) and [information about the team](https://nubook.nubots.net/team/introduction).

We also have generated documentation from the code available at [codedocs](https://codedocs.nubots.net/).

# Quick Setup

IMPORTANT: [NUbridge](https://github.com/nubots/nubridge), our ROS2 bridge for this codebase, will need to be setup before running any NUbots code on the robot.

1. Make sure [uv](https://docs.astral.sh/uv/getting-started/installation/) and [Docker](https://docs.docker.com/engine/install/) are setup on your computer, as our tools and compilation toolchain won't run without them.
2. Setup the Docker build environment:

```sh
./b target orinnx
```

3. Configure the repository via CMake:

```sh
./b configure
./b configure -i # This runs CCMake, a TUI interface for CMake that lets you choose which roles to build for and which build type to use (Release, Debug, etc.)
```

The important role for a game will be `ROLE_robocup`; make sure that role remains ON and disable any others.

4. Build the code

```sh
./b build # Optionally, for memory-challenged computers, use the -jN argument, where N is the maximum number of workers
```

5. Deploy the code

```sh
./b install -t -co -u booster <ROBOT_IP>
```

6. On the robot, run the compiled binary:

```sh
./robocup
```

If running a test role (e.x. test/localisation), then run it from the home directory as follows:

```sh
test/localisation
```
