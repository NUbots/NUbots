NUbots Codebase
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

		$ git clone git@github.com:nubots/NUbots.git ~/NUbots
		
	**Note** We recommend [Sublime Text 3][] for editing the code. The file 'NUbots.sublime-project' can be opened with Sublime ('Project -> Open Project...', then browse to your NUbots repository). This provides a shortcut to compiling the code from sublime with `Ctrl+B`, but don't try this until you have finished the instructions in this readme. Installers for Windows/OSX can be found on the Sublime site, while ubuntu users will need to use an installer from the command line:
		
		$ sudo add-apt-repository ppa:webupd8team/sublime-text-3
		$ sudo apt-get update
		$ sudo apt-get install sublime-text-installer

3. Create vagrant machine
	
		$ cd NUbots
		$ vagrant up
	
4. To set up vagrant and simulator

	**Note** Multiple build targets are supported. You must tell CMake which target you would like to build for. The currently supported targets are `NimbroOp` (aka igus), `DarwinOp`, and `native`. Due to the code optimisations done to the `NimbroOp` and `DarwinOp` builds it is possible that neither of these two builds will be able to run inside the vm, this is why `native` exists. In the commands that follow, `{target}` is one of the supported build targets.

		$ vagrant ssh
		$ mkdir NUbots/build_{target}
		$ cd NUbots/build_{target}
		$ cmake -DCMAKE_TOOLCHAIN_FILE=/nubots/toolchain/{target}.cmake .. -G Ninja
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

	$ nano config/NUbugger.yaml

6. Select which data is to be sent to NUsight (Usually just localization and sensors)

	$ bin/soccersimulator
	
and you're done!

Simulation parameters can be found in

	$ nano config/SoccerSimulatorConfig.yaml

TroubleShooting Vagrant
--------
####1 	Shared folders not mounted
	Run 
	
		$ vagrant plugin install vagrant-vbguest
	
	on the host machine. You may need to restart the host machine for it to work.



[nuclearport-travis]:     https://travis-ci.org/nubots/NUClearPort                "NUClearPort's Travis Page"
[travis-develop-image]:   https://travis-ci.org/nubots/NUClearPort.png?branch=develop "Travis-CI build status for the develop branch"
[git]:                    http://git-scm.com/                                     "Git"
[Python]:                 https://www.python.org/                                 "Python"
[NUClearPort]:            https://github.com/nubots/NUClearPort                   "NUClearPort Repository"
[NUsight]:                https://github.com/nubots/NUsight 	                  "NUsight Repository"
<!-- [nuclearport-startup-guide]: http://confluence.nubots.net/display/NUB/NUClearPort+Startup+Guide -->
[NUbots]:                 http://nubots.net/                                      "NUbots"
[robocup]:                https://github.com/nubots/robocup                       "Robocup"
[NUClear]:                https://github.com/Fastcode/NUClear                     "NUClear"
[NUsight]:                https://github.com/NUbots/NUsight                       "NUsight web robot debugger"
[Docker]:                 https://www.docker.com/                                 "Docker"
[Boot2Docker]:            http://boot2docker.io/                                  "Boot2Docker"
[Sublime Text 3]:         http://www.sublimetext.com/                             "Sublime Text 3"
[docker_download]:	  https://docs.docker.com/installation/                   "Docker Installation Page"
[What is Docker]:  	  https://www.docker.com/whatisdocker/ 			  "Docker's Getting Started Guide"
[Command-Line Interface]: https://docs.docker.com/reference/commandline/cli/	  "Docker Command-Line Interface Documentation"
[Homebrew]: 	          http://brew.sh/					  					  "Homebrew"
[Git]: 	                  https://git-scm.com/					  				  "Git version control"
[Vagrant]: 	              https://www.vagrantup.com/					          "Virtual machine wrapper"
[VirtualBox]: 	          https://www.virtualbox.org/					          "Virtual machine"
