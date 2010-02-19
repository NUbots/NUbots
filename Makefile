# Welcome to the Makefile for the NUbot Code
#
#    Copyright (c) 2009 Jason Kulk
#    This file is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    This file is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
# Targets: NAO, NAOWebots, Cycloid, NUView

CUR_DIR = $(shell pwd)

# Make directories
MAKE_DIR = $(CUR_DIR)/Make
MAKE_OPTIONS = --no-print-directory

# Build directories
NAO_BUILD_DIR = Build/NAO
NAOWEBOTS_BUILD_DIR = Build/NAOWebots
CYCLOID_BUILD_DIR = Build/Cycloid

# Aldebaran build tools
ALD_CC_SCRIPT = $(AL_DIR)/tools/crosscompile.sh
ALD_CTC = $(AL_DIR)/crosstoolchain

# External source directory
EXT_SOURCE_DIR = $(CUR_DIR)/Install

.PHONY: default_target all 
.PHONY: NAO NAOConfig NAOClean NAOVeryClean
.PHONY: NAOExternal
.PHONY: NAOWebots NAOWebotsConfig NAOWebotsClean NAOWebotsVeryClean
.PHONY: Cycloid CycloidConfig CycloidClean CycloidVeryClean
.PHONY: NUView NUViewConfig NUViewClean NUViewVeryClean
.PHONY: clean veryclean

# We export an environment variable TARGET_ROBOT which tells everything
# which robot we want this build to run on.
TARGET_ROBOT = NAOWEBOTS				# by default the target is Webots
NAOWebots: TARGET_ROBOT=NAOWEBOTS
NAOWebotsConfig: TARGET_ROBOT=NAOWEBOTS
NAO: TARGET_ROBOT=NAO
NAOConfig: TARGET_ROBOT=NAO
Cycloid: TARGET_ROBOT=CYCLOID
CycloidConfig: TARGET_ROBOT=CYCLOID
NUView: TARGET_ROBOT=VIRTUAL
NUViewConfig: TARGET_ROBOT=VIRTUAL
export TARGET_ROBOT

# I need to determine which platform this makefile is run on
SYSTEM = $(strip $(shell uname -s))

# If I choose to use an external machine to do the compiling; these are the username and ip address used
LOGNAME = $(strip $(shell logname))
VM = $(strip $(vm))

# If I choose to install it on a particular robot this is the ip used
ROBOT = $(strip $(robot))
default_target: NAOWebots

all: NAO NAOWebots Cycloid NUView

################ NAO ################
NAO:
	@echo "Targetting NAO";
ifeq ($(SYSTEM),windows32)				## if this file is run on windows using gnumake, use a virtual machine to compile
	@make NAOWindowsExternal
endif
ifeq ($(SYSTEM),Darwin)
	@make NAODarwinExternal				## if this file is run on Darwin also using a virtual machine (for now)
endif
ifeq ($(SYSTEM),Linux)					## if it is Linux then compile the source here!
	@if [ -f $(CUR_DIR)/$(NAO_BUILD_DIR)/Makefile ]; then \
		echo "Already configured"; \
		cd $(NAO_BUILD_DIR); \
		tput sgr0; \
		make $(MAKE_OPTIONS); \
	else \
		mkdir -p $(NAO_BUILD_DIR); \
		cd $(NAO_BUILD_DIR); \
		sh $(ALD_CC_SCRIPT) $(ALD_CTC) $(MAKE_DIR); \
		ccmake .; \
		echo "Configuration complete"; \
		make $(MAKE_OPTIONS); \
	fi
endif

NAODarwinExternal:
	@echo "Using virtual machine to target NAO";
#log into vm.local and make the project dir
	@ssh $(LOGNAME)@$(VM) mkdir -p naoqi/projects/robocup;
#copy everything in this directory except the existing .*, Build, Documentation directories
	@scp -prC $(filter-out Build Documentation Autoconfig, $(wildcard *)) $(LOGNAME)@$(VM):naoqi/projects/robocup;
#run make inside the vm
	@ssh -t $(LOGNAME)@$(VM) "cd naoqi/projects/robocup; make NAO;"
#copy the binary back
	@mkdir -p ./$(NAO_BUILD_DIR)
	@scp -prC $(LOGNAME)@$(VM):naoqi/projects/robocup/$(NAO_BUILD_DIR)/libnubot.so ./$(NAO_BUILD_DIR)/libnubot.so
#forward the binary to the robot, if a robot was specified
ifneq ($(robot),)
	@ssh root@$(ROBOT) /etc/init.d/naoqi stop
	@scp -C ./$(NAO_BUILD_DIR)/libnubot.so root@$(ROBOT):/opt/naoqi/modules/lib/
	@ssh -f root@$(ROBOT) /etc/init.d/naoqi start
endif
	
NAOWindowsExternal:
	@echo "Using virtual machine to target NAO"
	@echo "TODO"
	

NAOConfig:
	@echo "Configuring NAO Build";
ifeq ($(SYSTEM),windows32)				## if this file is run on windows using gnumake, use a virtual machine to compile
	@echo "TODO configure on a windows machine"
endif
ifeq ($(SYSTEM),Darwin)
	@ssh -t $(LOGNAME)@$(VM) "cd naoqi/projects/robocup; make NAOConfig;"
endif
ifeq ($(SYSTEM),Linux)					## if it is Linux then configure the source here!
	@set -e; \
		cd $(NAO_BUILD_DIR); \
		ccmake .; \
		make $(MAKE_OPTIONS);
endif

NAOClean:
	@echo "Cleaning NAO Build";
ifeq ($(SYSTEM),windows32)				## if this file is run on windows using gnumake, use a virtual machine to compile
	@echo "TODO clean on a windows machine"
endif
ifeq ($(SYSTEM),Darwin)
	@ssh -t $(LOGNAME)@$(VM) "cd naoqi/projects/robocup; make NAOClean;"
endif
ifeq ($(SYSTEM),Linux)					## if it is Linux then configure the source here!
	@set -e; \
		cd $(NAO_BUILD_DIR); \
		make $(MAKE_OPTIONS) clean;
endif

NAOVeryClean:
	@echo "Hosing NAO Build";
ifeq ($(SYSTEM),windows32)				## if this file is run on windows using gnumake, use a virtual machine to compile
	@echo "TODO very clean on a windows machine"
endif
ifeq ($(SYSTEM),Darwin)
	@ssh -t $(LOGNAME)@$(VM) "cd naoqi/projects/robocup; make NAOVeryClean;"
endif
ifeq ($(SYSTEM),Linux)					## if it is Linux then configure the source here!
	@set -e; \
		cd $(NAO_BUILD_DIR); \
		rm -rf ./*;
endif

################ NAOWebots ################

NAOWebots:
	@echo "Targetting NAOWebots";
	@if [ -f $(CUR_DIR)/$(NAOWEBOTS_BUILD_DIR)/Makefile ]; then \
		cd $(NAOWEBOTS_BUILD_DIR); \
		make $(MAKE_OPTIONS); \
	else \
		mkdir -p $(NAOWEBOTS_BUILD_DIR); \
		cd $(NAOWEBOTS_BUILD_DIR); \
		cmake $(MAKE_DIR); \
		ccmake .; \
		make $(MAKE_OPTIONS); \
	fi

NAOWebotsConfig:
	@set -e; \
		cd $(NAOWEBOTS_BUILD_DIR); \
		cmake $(MAKE_DIR); \
		ccmake .; \
		make $(MAKE_OPTIONS);

NAOWebotsClean:
	@echo "Cleaning NAOWebots Build";
	@set -e; \
		cd $(NAOWEBOTS_BUILD_DIR); \
		make $(MAKE_OPTIONS) clean;

NAOWebotsVeryClean:
	@echo "Hosing NAOWebots Build";
	@set -e; \
		cd $(NAOWEBOTS_BUILD_DIR); \
		rm -rf ./*;

################ Cycloid ################

Cycloid:
	@echo "Targetting Cycloid";
	@cmake $(MAKE_DIR);

CycloidClean:
	@echo "Cleaning Cycloid Build"

CycloidVeryClean:
	@echo "Hosing Cycloid Build";
	@set -e;
	
################ NUView ################
NUView:
	@echo "Building NUView"
# now qmake and then make the NUview project
	@set -e; \
		cd $(CUR_DIR)/NUview; \
		qmake -spec macx-g++; \
		make all; \
	
NUViewConfig:
	@echo "Configuring NUView Build"
	
NUViewClean:
	@echo "Cleaning NUView Build"
	@set -e; \
		cd $(CUR_DIR)/NUview; \
		make clean; \
		rm -rf NUview.app; \
	
NUViewVeryClean:
	@echo "Hosing NUView Build"
	@set -e; \
		cd $(CUR_DIR)/NUview; \
		make clean; \
		rm -rf NUview.app; \
		rm -f Makefile; \

########################################

clean: NAOClean NAOWebotsClean CycloidClean NUViewClean

veryclean: NAOVeryClean NAOWebotsVeryClean CycloidVeryClean NUViewVeryClean


# Helpful tips:
# 1. Makefile uses actual tabs as separators, so you'll get a missing separator
#    with editors that use spaces instead
# 2. Make sure there is no white space after the \
