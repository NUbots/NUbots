# NUbots Codebase [![Build status](https://badge.buildkite.com/85cb206a2615c85981c4e0089b0abb0c6bcd775b3d946ede40.svg?branch=master)](https://buildkite.com/nubots/nubots)

NUbots is a team in the University of Newcastle's robotics research group focused on developing humanoid soccer-playing robots for the international RoboCup competition.
The team is made up of a multidisciplinary team of students and academics. The team develops both hardware and software for the robots.
This codebase is the current state of our development efforts to compete in the competition as well as active research projects within the laboratory.

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
5. By default the docker install will allocate 2GB of memory to docker images. When building the images this is insufficient for some of the libraries and will result in an error. In the menu bar go to advanced settings and increase the memory available.
6. Download the codebase

```sh
git clone https://github.com/NUbots/NUbots.git
```

7. Install the required python dependencies

```sh
pip3 install -r NUbots/requirements.txt
```

## Windows

Windows support is coming soon™

# Building and Installing

Once you have the codebase and prerequisites installed you should be ready to start working with the codebase.
The NUbots codebase can be built for more than one target.
Depending on if you want to run the code on your personal computer or one of the robots you will need to select a target platform that you will be executing the code on.

## Selecting a platform

To select the platform you use the `target` command in the `b` script. For example to select the `generic` platform which is used when running code on your own computer. Run the following:

```sh
./b target generic
```

This will download and setup the generic image and set it as your default image when running new commands. To change to build code to execute on the robot you would execute:

```sh
./b target nuc7i7bnh
```

This would build code that is optimised specifically to run on the platform that is currently used in the NUgus robots. Note that when building for other platforms than generic, the code may not be able to execute on your own computer depending on the CPU that you have.

## Configuring the code

To configure the code for building run:

```sh
./b configure
```

This will configure the code for building.
If you want to change settings you can add the cmake flags to the end of the configure call such as the following command that will build the code with static libraries. The `--` is needed to stop python from processing the arguments and allow them to be passed on to cmake.

```sh
./b configure -- -DSTATIC_LIBRARIES=On
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
To run commands like these you use the `run` command of `b`.
For example, to run the `nusighttest` binary run:

```sh
./b run nusighttest
```

## Installing the code on a robot

To install the code on a robot you should use the `install` command of the `b` script.
For example, to install the code onto the robot nugus1 you should run the following.

```sh
./b install nugus1
```

Note that you can replace nugus1 with any of the pre-configured robot names, or an IP address of a target robot.

`./b install` also accepts the following options

| Option | Description                                                                    |
| :----: | :----------------------------------------------------------------------------- |
|   -u   | The user to install to on the target. Defaults to the user in the Docker image |
|   -t   | Install toolchain to the target                                                |
|  -cn   | Only install new config files. This is the default                             |
|  -cu   | Update config files on the target that are older than the local files          |
|  -co   | Overwrite all config files on the target                                       |
|  -ci   | Ignore all changes to config files (installs no config files)                  |

## Editing config files in the build directory

When running code on your local machine, it may be necessary to edit a configuration file (or other data file). The preferred method for doing this is to edit the file in the NUbots source directory and rebuilding (`./b build`). However, certain modules generate their configuration at build time, meaning there is no file in the source directory to edit.

To work around this, you can use the `edit` command of `b`. For example, to edit the `DataLogging` configuration file run:

```sh
./b edit config/DataLogging.yaml
```

The `edit` command will use the editor that is defined in your host shell (check the `EDITOR` enviroment variable). If this is not set then it will default to the nano text editor. Supported values for `EDITOR` (for `./b edit`) are `nano` and `vim`.

# Flashing a new robot

To install Arch Linux (our OS of choice) on to a robot perform the following instructions

1. Download the latest [Arch Linux LiveUSB](https://www.archlinux.org/download/) image and burn it on to a [USB thumb drive](https://wiki.archlinux.org/index.php/USB_flash_installation_media#In_GNU/Linux)
1. Boot into the LiveUSB environment on the robot and ensure the robot has an active network connection

   - Run `ip addr` and look for a `inet` line that has a valid IP address on it. If you can't see one you don't have a network connection. Alternatively, run `ping google.com` and look for a response time.
   - If you have no connection and need to set up the WiFi interface, see below.

1. Download the installation script [https://git.io/JeWaF](https://git.io/JeWaF) and make sure the script is executable

   - The installation script is located in this repo at [doc/ArchInstall/arch_install.sh](doc/ArchInstall/arch_install.sh)
   - ```sh
     wget https://git.io/JeWaF -O ./arch_install.sh
     chmod +x ./arch_install.sh
     ```

1. Execute the script and follow the instructions
   - ```sh
     ./arch_install.sh
     ```

The installation script will end by downloading a secondary script and providing you with a command that you must run

```sh
ROBOT_NUMBER=<N> arch-chroot /mnt ./arch-chroot_install.sh
```

Substitute `<N>` with the number of the robot that you are building. This will influence the IP address of the robot as well as the robots' hostname.

Finally, once that script has finished, you must run one more command.

```sh
/mnt/arch-post_install.sh
```

This command will end by rebooting the robot. When this happens be sure to remove the USB installation drive from robot so that you may boot into the new system.

The final two scripts can be found at [doc/ArchInstall/arch-chroot_install.sh](doc/ArchInstall/arch-chroot_install.sh) and [doc/ArchInstall/arch-post_install.sh](doc/ArchInstall/arch-post_install.sh)

## Setting up WiFi manually

To set up the WiFi interface you first need to know the name of the interface. To find this, run

```sh
ip link
```

and look for an interface starting with `wl`. On our robots it is usually `wlp58s0`.

The rest of these instructions assume a network using WPA2.

1. Make sure the `wpa_supplicant` configuration directory exists

   - ```sh
     mkdir /etc/wpa_supplicant
     ```

1. Setup the network configuration

   - ```sh
     wpa_passphrase <Network SSID> <Network Passphrase> > /etc/wpa_supplicant/wpa_supplicant-<WiFi Interface>.conf
     ```
   - Be sure to replace `<Network SSID>` with the SSID of the wireless network to connect to, `<Network Passphrase>` with the password for the network, and `<WiFi Interface>` with the name of the interface found earlier.

1. Now start all of the necessary services

   - ```sh
     systemctl start dhcpcd.service
     systemctl start wpa_supplicant.service
     systemctl start wpa_supplicant@<Wifi Interface>.service
     ```
   - Be sure to replace `<WiFi Interface>` with the name of the interface found earlier.
   - Wait a handful of seconds and run `ip addr` to check that the WiFi interface has an IP address

---

<details>
  <summary>Undocumented options</summary>

## Shell access

For some reason, if you really, really, _REALLY_ need to access a terminal inside of Docker you can run

```sh
./b shell
```

---

**NOTE**
Any changes you make outside of the `/home/nubots/NUbots` or `/home/nubots/build` directories will not persist after exiting the Docker shell.

---

</details>
