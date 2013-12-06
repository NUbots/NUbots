NUbots NUClearPort Project
==========================

The [NUClearPort][] project is an effort to port the [NUbots][]' [robocup][] 
codebase to use the new [NUClear][] framework.

Vagrant
--------

The NUbots use [Vagrant][] to manage and version the build environment for the NUClearPort project.

The following is a guide to getting you set up and ready to contribute to the NUClearPort project.

1. Install the following prerequisites on your machine (packages/installers are available for Windows, OSX, and Linux):
	* [Git][]
	* [Virtualbox][]
	* [Vagrant][vagrant_download] (Note: The version of Vagrant that is available in the Ubuntu 12.04 repositories is an older version that is incompatible with the NUbots' Vagrantfile. If you're using Ubuntu 12.04, please install the latest version of Vagrant via a .deb from the [Vagrant download page][vagrant_download])

2. Clone this git repository onto your machine:
	e.g.

		$ git clone git@github.com:nubots/NUClearPort.git ~/NUClearPort


3. Run `vagrant up` from within the directory just created by the clone operation:
	e.g.

		$ cd ~/NUClearPort
		$ vagrant up

	The `vagrant up` command tells Vagrant to create and start a VM for the NUClearPort project 
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

	Vagrant will sync the `~/nubots/NUClearPort` directory on the VM with the root of your NUClearPort repository.
	This allows for easy editing of code on your machine, and building on the VM.

	To build NUClearPort, just run the following commands on the VM:
  
		$ mkdir ~/nubots/NUClearPort/build
		$ cd ~/nubots/NUClearPort/build
		$ cmake ..
		$ make -j

6. Make robots do awesome stuff!

	Also, read the steps in the [NUClearPort Startup Guide][nuclearport-startup-guide] about
	additional Vagrant config.

	**Important:** Make sure to set your git identity correctly before committing to the project.
	
		$ git config --global user.name "Your Name"
		$ git config --global user.email you@example.com

		$ git config --global color.ui auto


[git]:                    http://git-scm.com/                                     "Git"
[NUClearPort]:            https://github.com/nubots/NUClearPort                   "NUClearPort Repository"
[nuclearport-startup-guide]: http://confluence.nubots.net/display/NUB/NUClearPort+Startup+Guide
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
