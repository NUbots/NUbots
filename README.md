NUbots Codebase [![Build status](https://badge.buildkite.com/85cb206a2615c85981c4e0089b0abb0c6bcd775b3d946ede40.svg?branch=master)](https://buildkite.com/nubots/nubots)
===============

TODO describe what this codebase is

# Setting up the Development Environment
The NUbots codebase uses Docker containers to build the codebase.
These docker containers are managed by a python script called `b`.
This `b` script provides the tools for working with the NUbots codebase and related artifacts such as recordings.

The recommended editor for working with the NUbots codebase is [Visual Studio Code](https://code.visualstudio.com/).
When opening the code folder project with vscode, it should suggest extensions that are helpful for working with the NUbots code.
Additionally one of these will be the Remote Development Containers extension.
This extension allows vscode to open the docker container and operate within it.

Below are instructions on how to setup a computer to build the codebase ready to develop either on your own computer or on a robot.

## Ubuntu

---
**NOTE**
If you are using anything other than Ubuntu we assume that you know what you are doing and know how to modify the following commands appropriately

---

1. Add the universe and docker repositories
```sh
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -
sudo add-apt-repository \
   "deb [arch=amd64] https://download.docker.com/linux/ubuntu \
   $(lsb_release -cs) \
   stable"
sudo add-apt-repository universe
sudo apt update
```
2. Install docker, git, and python3
```sh
sudo apt install docker-ce docker-ce-cli containerd.io python3 python3-pip git
```
3. Add current user to the docker group (to allow non-root usage of docker)
```sh
sudo usermod -aG docker "${USER}"
```
4. Reboot (or log out and log back in) to make the group change take effect
5. Download the codebase
```sh
git clone https://github.com/NUbots/NUbots.git
```
6. Install the required python dependencies
```sh
sudo -H pip3 install -r NUbots/requirements.txt
```

## Mac OSX

The easiest way to install most of the requirements for developing on Mac OSX is to use [Homebrew](https://brew.sh/) to install the dependencies.

1. Install homebrew by following the directions at https://brew.sh/
2. Install git, python3, and docker
```sh
brew install git python3
brew cask install docker
```
4. Run the docker app that was installed which should add a docker icon to the menu bar.
5. By default the docker install will allocate 2GB of memory to docker images. When building the images this is insufficent for some of the libraries and will result in an error. In the menu bar go to advanced settings and increase the memory available.
6. Download the codebase
```sh
git clone https://github.com/NUbots/NUbots.git
```
7. Install the required python dependencies
```sh
pip3 install -r NUbots/requirements.txt
```

## Windows

TODO

# Building and Installing
Once you have the codebase and prerequisites installed you should be ready to start working with the codebase.
The NUbots codebase can be built for more than one target.
Depending on if you want to run the code on your personal computer or one of the robots you will need to select a target platform that you will be executing the code on.

## Selecting a platform

To select the platform you use the target command in the `b` script. For example to select the `generic` platform which is used when running code on your own computer. Run the following:
```sh
./b target generic
```
This will download and setup the generic image and set it as your default image when running new commands. To change to build code to execute on the robot you would execute:
```sh
./b target nuc7i7bnh
```
This would build code that is optimised specificially to run on the platform that is currently used in the NUgus robots. Note that when building for other platforms than generic, the code may not be able to execute on your own computer depending on the CPU that you have.

## Configuring the code
To configure the code for building run:
```sh
./b configure
```
This will configure the code for building.
If you want to change settings you can add the cmake flags to the end of the configure call such as the following command that will build the code with static libraries.
```sh
./b configure -DSTATIC_LIBRARIES=On
```
Additionally if you wish to perform interactive configuration (with ccmake) you can run
```sh
./b configure -i
```

## Building the code
To compile the code you can run:
```sh
./b build
```
This will build the codebase for the currently selected platform.
If you had not run configure first, this will also configure the code with a default set of options.

## Running the code on the local computer
Once you have built code in the local docker container, you may want to run binaries that do not depend on the robots hardware.
For example those that use gazebo simulations.
To run commands like these you use the run command of b.
For example, to run the nusighttest binary run:
```sh
./b run nusighttest
```

## Installing the code on a robot
To install the code on a robot you should use the install command of the b script.
For example, to install the code onto the robot igus1 you should run the following.
```
./b install igus1
```
Note that you can replace igus1 with any of the preconfigured robot names, or an IP address of a target robot.
