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

		$ vagrant ssh
		$ cd nubots/NUbots/build
		$ cmake .. -G Ninja
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


Docker
--------

Alternative to vagrant. The NUbots use [Docker][] to manage the build environment for the NUbots project.

The following is a guide to getting you set up and ready to contribute to the NUbots project.

1. Install the following prerequisites on your machine (packages/installers are available for Windows, OSX, and Linux):
	* [Git][]
	* [Python][] (Use the 32bit version; stability on 64bit version needs improvement)
	* Linux: [Docker][docker_download]
	* Windows/OSX: [Boot2Docker][] (Windows and OSX cannot run the native docker client and must use Boot2Docker. Instructions on how to install Boot2Docker are on the [Docker Installation page][docker_download])
		 
	Linux: Installation should be done via the apt-get repositories. Open a console with `Ctrl+Alt+T` and type:

		$ sudo apt-get install git
		$ sudo apt-get install python
		$ sudo apt-get install docker.io
		$ sudo addgroup $(whoami) docker

	Windows and OSX installation can be done with installers from the program sites above.

2. Clone this git repository onto your machine. First, open a terminal. In Ubuntu, the default shortcut is `Ctrl+Alt+T`. In Windows you will need to open a Git Bash terminal (this is installed when you install Git; search for Git Bash in the Start Menu). (Todo: Terminal in OSX). Then enter the following:

		$ git clone git@github.com:nubots/NUbots.git ~/NUbots
		
	**Note** We recommend [Sublime Text 3][] for editing the code. The file 'NUbots.sublime-project' can be opened with Sublime ('Project -> Open Project...', then browse to your NUbots repository). This provides a shortcut to compiling the code from sublime with `Ctrl+B`, but don't try this until you have finished the instructions in this readme. Installers for Windows/OSX can be found on the Sublime site, while ubuntu users will need to use an installer from the command line:
		
		$ sudo add-apt-repository ppa:webupd8team/sublime-text-3
		$ sudo apt-get update
		$ sudo apt-get install sublime-text-installer

3. Run `docker build` from the b script in the NUbots directory:

		$ cd ~/NUbots

	and then, if you are using Linux or OSX:

		$ ./b compile

	and if you are using Windows:

		$ python b compile

	The `./b compile` command tells Docker to build a container for the NUbots project 
	based on the project's `Dockerfile`.(While your container is being built, you might want to learn a little more about Docker by reading the article [What is Docker][] or the [Command-Line Interface][] documentation.)

	**Note:** The very first time `./b docker build` is run on your computer, it needs to download a streamlined Ubuntu 14.04 image and install all the required dependencies. This will take around 15 minutes.

  	**Note:** The `./b` command above is a collection of python scripts to make docker
  usage easier when working with the NUbots codebase.

  	**Note** Docker will sync the `/nubots/NUbots` directory on the container with the root of your NUbots repository. This allows for easy editing of code on your machine, and building on the container.

	**Note:** If you are on Windows/OSX your code must be in /c/Users/ or /Users/
  respectively.

5. Try running a binary!
	
	OSX and Linux:
	
		$ ./b role run keyboardwalkfake
	
	Windows:

		$ python b role run keyboardwalkfake
		
	You can see the result of this binary by installing [NUsight][]. You will also have to edit the file config/NUbugger.yaml in the build directory so that sensors is set to `true`.

**Important:** Make sure to set your git identity correctly before committing to the project.

	$ git config --global user.name "Your Name"
	$ git config --global user.email you@example.com
	$ git config --global color.ui auto
Troubleshooting Docker
--------

Check out the `docker` file to see the actual commands that are being run if you
are having issues. 

####1.  Cannot connect to the Docker daemon. Is 'docker -d' running on this host? [Ubuntu]
	Try to stop docker service and then restart it again.
	$ sudo service docker stop
	$ sudo service docker start
	


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
