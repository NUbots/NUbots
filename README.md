NUbots Codebase
==========================

<!-- The [NUbots][] project is an effort to port the [NUbots][]' [robocup][] 
codebase to use the new [NUClear][] framework. -->

Docker
--------

The NUbots use [Docker][] to manage and version the build environment for the NUbots project.

The following is a guide to getting you set up and ready to contribute to the NUbots project.

1. Install the following prerequisites on your machine (packages/installers are available for Windows, OSX, and Linux):
	* [Git][]
	* [Docker][docker_download] (Note: Windows and OSX cannot run the native docker client and must use Boot2Docker. Instructions on how to install Boot2Docker are on the [Docker Installation page][docker_download])

2. Clone this git repository onto your machine:
	e.g.

		$ git clone git@github.com:nubots/NUbots.git ~/NUbots

3. Run `docker build -t="nubots/nubots" .` from the NUbots directory:
	e.g.

		$ cd ~/NUbots
		$ docker build -t="nubots/nubots" .

	The `docker build` command tells Docker to build a container for the NUbots project 
	based on the project's `Dockerfile`.

	**Note:** The very first time `docker build` is run on your computer, it needs to download a streamlined Ubuntu 14.04 image and install all the required dependencies. This will take around 15 minutes.

	(While your container is being built, you might want to learn a little more about Docker by 
	reading the article [What is Docker][] or the [Command-Line Interface][] documentation.

4.  Type `$ docker run -t -i -v /path/to/NUbots:/nubots/NUbots nubots/nubots /bin/bash` to access the command line for the container!

	Docker will sync the `/nubots/NUbots` directory on the container with the root of your NUbots repository.
	This allows for easy editing of code on your machine, and building on the container.

	To build NUbots, just run the following commands on the container:
  
		$ mkdir /nubots/NUbots/build
		$ cd /nubots/NUbots/build
		$ cmake .. -G Ninja
		$ ninja -j

5. Make robots do awesome stuff!

	**Important:** Make sure to set your git identity correctly before committing to the project.
	
		$ git config --global user.name "Your Name"
		$ git config --global user.email you@example.com

		$ git config --global color.ui auto

Troubleshooting
--------

Nothing here yet

[nuclearport-travis]:     https://travis-ci.org/nubots/NUClearPort                "NUClearPort's Travis Page"
[travis-develop-image]:   https://travis-ci.org/nubots/NUClearPort.png?branch=develop "Travis-CI build status for the develop branch"
[git]:                    http://git-scm.com/                                     "Git"
[NUClearPort]:            https://github.com/nubots/NUClearPort                   "NUClearPort Repository"
<!-- [nuclearport-startup-guide]: http://confluence.nubots.net/display/NUB/NUClearPort+Startup+Guide -->
[NUbots]:                 http://nubots.net/                                      "NUbots"
[robocup]:                https://github.com/nubots/robocup                       "Robocup"
[NUClear]:                https://github.com/Fastcode/NUClear                     "NUClear"
[Docker]:                 https://www.docker.com/                                 "Docker"
[docker_download]:	  https://docs.docker.com/installation/                   "Docker Installation Page"
[What is Docker]:  	  https://www.docker.com/whatisdocker/ 			  "Docker's Getting Started Guide"
[Command-Line Interface]: https://docs.docker.com/reference/commandline/cli/	  "Docker Command-Line Interface Documentation"
