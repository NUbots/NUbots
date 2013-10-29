NUbots NUClearPort Project
==========================

The [NUClearPort](https://github.com/nubots/NUClearPort) project is an effort to
port the [NUbots](http://nubots.net/)' [robocup](https://github.com/nubots/robocup) codebase
to use the new [NUClear](https://github.com/Fastcode/NUClear) framework.

Vagrant
--------

The NUbots use [Vagrant](http://www.vagrantup.com/) to manage and version the 
build environment for the NUClearPort project.

The following is a guide to getting you set up and ready to contribute to the NUClearPort project.

1. Install the following prerequisites on your machine (packages/installers are available for Windows, OSX, and Linux):
	* [Git](http://git-scm.com/)
	* [Virtualbox](https://www.virtualbox.org/wiki/Downloads)
	* [Vagrant](http://downloads.vagrantup.com/)

2. Clone this git repository onto your machine:
	e.g.

		$ git clone git@github.com:nubots/NUClearPort.git ~/NUClearPort


3. Run `vagrant up` from within the directory just created by the clone operation:
	e.g.

		$ cd ~/NUClearPort
		$ vagrant up

  The `vagrant up` command tells Vagrant to create and start a VM for the NUClearPort project based on the project's `Vagrantfile`.

	Warning: The very first time this command is run on your computer, it will initiate
	a 282 MB download ([the base box for the VM](http://files.vagrantup.com/precise32.box)).
	Vagrant will store the box locally in a special location, and will not require you to download it again.

	When given a choice of network interface, e.g.:

		[default] Available bridged network interfaces:
		1) en0: Wi-Fi (AirPort)
		2) p2p0

	Select which adapter the VM will use for its network connection by 
	entering a number (if in doubt, the first option is likely to be the best choice).

  (While your VM is generated, you might want to learn a little more about Vagrant by 
  reading the [Getting Started Guide](http://docs.vagrantup.com/v2/getting-started/index.html) 
  or the [Command-Line Interface](http://docs.vagrantup.com/v2/cli/index.html) documentation)

4. Just type `$ vagrant ssh` to ssh into your new VM!

  Vagrant will sync the `~/nubots/NUClearPort` directory on the VM with the root of your NUClearPort repository.
  This allows for easy editing of code on your machine, and building on the VM.

  To build NUClearPort, just run the following commands on the VM:
  
    $ mkdir ~/nubots/NUClearPort/build
    $ cd ~/nubots/NUClearPort/build
    $ cmake ..
    $ make -j

6. Make robots do awesome stuff!

	Important: Make sure to set your git identiy correctly before committing to the project.
	
		$ git config --global user.name "Your Name"
		$ git config --global user.email you@example.com

		$ git config --global color.ui auto
