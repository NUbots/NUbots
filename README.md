LICENCE
=======

GNU GPLv3 licence file

NUbots Robocup Project
==========================

The purpose of this software is to provide a software architecture and tools to enable a humanoid robot to play soccer against other robots. In particular, play soccer in Robocup's Kid-Size League.

Vagrant
--------

The NUbots use [Vagrant][] to manage and version the build environment for the Robocup project.

The following is a guide to getting you set up and ready to contribute to the Robocup project.

1. Install the following prerequisites on your machine (packages/installers are available for Windows, OSX, and Linux):
	* [Git][]
	* [Virtualbox][]
	* [Vagrant][vagrant_download]

2. Clone this git repository onto your machine:
	e.g.

		$ git clone git@github.com:nubots/robocup.git ~/robocup


3. Run `vagrant up` from within the directory just created by the clone operation:
	e.g.

		$ cd ~/robocup
		$ vagrant up

	The `vagrant up` command tells Vagrant to create and start a VM for the Robocup project 
	based on the project's `Vagrantfile`.

	**Note:** The very first time `vagrant up` is run on your computer, it will initiate
	a 282 MB download ([the base box for the VM][precise_32_box]).
	Vagrant will store the box locally in a special location, and will not need to download it again
	(_see the [boxes page][] of Vagrant's Getting Started guide, or Vagrant's [boxes][] documentation
	if you want to know more about boxes_).

	When given a choice of network interface, e.g.:

		[default] Available bridged network interfaces:
		1) en0: Wi-Fi (AirPort)
		2) p2p0

	Select which adapter the VM will use for its network connection by 
	entering a number (if in doubt, the first option is likely to be the best choice).

	(While your VM is being created, you might want to learn a little more about Vagrant by 
	reading the [Getting Started Guide][] or the [Command-Line Interface][] documentation)

4.  Just type `$ vagrant ssh` to ssh into your new VM!

	Vagrant will sync the `~/nubots/robocup` directory on the VM with the root of your Robocup repository.
	This allows for easy editing of code on your machine, and building on the VM.

	To build Robocup, just run the following commands on the VM:
  
		$ cd ~/nubots/robocup
		$ make

6. Make robots do awesome stuff!

	**Important:** Make sure to set your git identity correctly before committing to the project.
	
		$ git config --global user.name "Your Name"
		$ git config --global user.email you@example.com

		$ git config --global color.ui auto

* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 

Copyright (c) NUbots, University of Newcastle

bq.. This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.


[git]:                    http://git-scm.com/                                     "Git"
[NUClearPort]:            https://github.com/nubots/NUClearPort                   "NUClearPort Repository"
[NUbots]:                 http://nubots.net/                                      "NUbots"
[robocup]:                https://github.com/nubots/robocup                       "Robocup"
[NUClear]:                https://github.com/Fastcode/NUClear                     "NUClear"
[Vagrant]:                http://www.vagrantup.com/                               "Vagrant"
[Virtualbox]:             https://www.virtualbox.org/wiki/Downloads               "Virtualbox"
[vagrant_download]:       http://downloads.vagrantup.com/                         "Vagrant Download Page"
[precise_32_box]:         http://files.vagrantup.com/precise32.box                "Ubuntu 12.04 Box for Vagrant"
[Getting Started Guide]:  http://docs.vagrantup.com/v2/getting-started/index.html "Vagrant's Getting Started Guide"
[Command-Line Interface]: http://docs.vagrantup.com/v2/cli/index.html             "Vagrant Command-Line Interface Documentation"
[boxes page]:             http://docs.vagrantup.com/v2/getting-started/boxes.html "The Boxes section of Vagrant's Getting Started guide"
[boxes]:                  http://docs.vagrantup.com/v2/boxes.html                 "Vagrant's Boxes documentation"
