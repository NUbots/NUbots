NUbots Codebase [![Build Status](https://travis-ci.org/NUbots/NUbots.svg?branch=master)](https://travis-ci.org/NUbots/NUbots)
==========================

Vagrant
--------


1. Install the following prerequisites on your machine (packages/installers are available for Windows, OSX, and Linux):
    * [Git][]
    * [Virtualbox][]
    * [Vagrant][]

    e.g. Linux: Installation should be done via the apt-get repositories. Open a console with `Ctrl+Alt+T` and type:

        $ sudo apt-get install git
        $ sudo apt-get install virtualbox

        NOTE: Ubuntu users should get the debian for Vagrant from the vagrant website.

    Windows and OSX installation can be done with installers from the program sites above.


2. Clone this git repository onto your machine. First, open a terminal. In Ubuntu, the default shortcut is `Ctrl+Alt+T`. In Windows you will need to open a Git Bash terminal (this is installed when you install Git; search for Git Bash in the Start Menu). (Todo: Terminal in OSX). Then enter the following:

        $ git clone https://github.com/NUbots/NUbots.git

    **Note** We recommend [Sublime Text 3][] for editing the code. The file 'NUbots.sublime-project' can be opened with Sublime ('Project -> Open Project...', then browse to your NUbots repository).
    The NUbots codebase also applies a formating style guide provided by [clang-format][].
    This can be used from within Sublime using the [ClangFormat][] package from PackageControl.

3. Create vagrant machine

        $ cd NUbots
        $ vagrant up

4. To set up vagrant and simulator

    **Note** Multiple build targets are supported. You must tell CMake which target you would like to build for.
    The currently supported targets are `nuc7i7bnh` (used for the iGus), `fitpc2i` (used for the DARwIn-OP), and `native`.
    Due to the code optimisations done to the `nuc7i7bnh` and `fitpc2i` builds it is possible that neither of these two builds will be able to run inside the vm, this is why `native` exists.
    In the commands that follow, `{target}` is one of the supported build targets.

        $ vagrant ssh
        $ cd NUbots
        $ ./b platform select {target}
        $ cd build
        $ ninja

4. If quex permission error occurs run

        $ chmod +x /usr/local/bin/quex
        $ quex
        $ ninja

5. Open a second terminal, go to [NUsight][] (after cloning) and run

        $ node app

then in internet browser go to

    localhost:9090

and then localization window
go back to the first terminal and run

    $ nano config/NUsight.yaml

6. Select which data is to be sent to NUsight (Usually just localization and sensors)

    $ bin/soccersimulator

and you're done!

Simulation parameters can be found in

    $ nano config/SoccerSimulatorConfig.yaml

TroubleShooting Vagrant
--------
1. If the shared folders are not mounted you may need to run the following command on the host machine.

        $ vagrant plugin install vagrant-vbguest

2. If you have multiple nubotsvm in NUsight you can modify the name property in the following file

        $ cd build/
        $ nano config/NetworkConfiguration.yaml

3. If dpkg is locked during vagrant provision:
   This procedure should resolve most issues that may cause the dpkg to be locked (the lock file is like a mutex).
   Pay attention to the output of the apt-get commands and look for any further errors.

        $ vagrant ssh
        $ sudo rm /var/lib/dpkg/lock
        $ sudo apt-get install -f
        $ sudo apt-get update
        $ sudo apt-get upgrade
        $ exit
        $ vagrant provision

   Optional step:
   Run this after the "apt-get upgrade" command.
   This isn't necessary to resolve any problems, it will just free up some hard drive space.

        $ sudo apt-get autoremove --purge

[NUbots]:                 http://nubots.net/                                      "NUbots"
[NUClear]:                https://github.com/Fastcode/NUClear                     "NUClear"
[NUsight]:                https://github.com/NUbots/NUsight                       "NUsight web robot debugger"
[Sublime Text 3]:         http://www.sublimetext.com/                             "Sublime Text 3"
[Homebrew]:               http://brew.sh/                                         "Homebrew"
[Git]:                    https://git-scm.com/                                    "Git version control"
[Vagrant]:                https://www.vagrantup.com/                              "Virtual machine wrapper"
[VirtualBox]:             https://www.virtualbox.org/                             "Virtual machine"
[clang-format]:           https://clang.llvm.org/docs/ClangFormat.html            "Clang Format"
[ClangFormat]:            https://packagecontrol.io/packages/Clang%20Format       "Clang Format"
