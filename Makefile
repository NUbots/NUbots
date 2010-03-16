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
ALD_CTC = $(AL_DIR)/crosstoolchain/toolchain-geode.cmake

# Source directory on external machine
SOURCE_EXT_DIR = naoqi/projects/robocup

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
ifeq ($(strip $(user)), )
	LOGNAME = $(strip $(shell logname))
else
	LOGNAME = $(strip $(user))
endif
VM_IP = $(strip $(vm))

# If I choose to install it on a particular robot this is the ip used
ROBOT_IP = $(strip $(robot))
default_target: NAOWebots

all: NAO NAOWebots Cycloid NUView

################ NAO ################
NAO:
ifeq ($(VM_IP), )							## if we have not given a virtual machine IP then use this machine to compile
    ifeq ($(SYSTEM),Linux)					## can only cross-compile on linux
		@echo "Cross-compiling for NAO"
        ifeq ($(findstring Makefile, $(wildcard $(CUR_DIR)/$(NAO_BUILD_DIR)/*)), )		## check if the project has already been configured
			@set -e; \
				echo "Configuring for first use"; \
				mkdir -p $(NAO_BUILD_DIR); \
				cd $(NAO_BUILD_DIR); \
				cmake -DCMAKE_TOOLCHAIN_FILE=$(ALD_CTC) $(MAKE_DIR); \
				ccmake .; \
				make $(MAKE_OPTIONS);
        else
			@set -e; \
				cd $(NAO_BUILD_DIR); \
				make $(MAKE_OPTIONS);
        endif
		@echo $(ROBOT_IP)
        ifneq ($(ROBOT_IP),)
			@ssh nao@$(ROBOT_IP) /etc/init.d/naoqi stop
			@scp -pC ./$(NAO_BUILD_DIR)/sdk/lib/naoqi/libnubot.so nao@$(ROBOT_IP):/home/nao/naoqi/lib/naoqi/
			@ssh nao@$(ROBOT_IP) /etc/init.d/naoqi start
        endif
    else
		@echo "Cannot cross-compile on this machine"
    endif
else
	@make NAOExternal
endif

NAOExternal:
	@echo "Send source to external machine $(LOGNAME)@$(VM_IP)";
#log into VM_IP and make the project dir
	@ssh $(LOGNAME)@$(VM_IP) mkdir -p naoqi/projects/robocup;
#copy everything in this directory except the existing .*, Build, Documentation directories
	@scp -prC $(filter-out Build Documentation Autoconfig NUview, $(wildcard *)) $(LOGNAME)@$(VM_IP):$(SOURCE_EXT_DIR);
#run make inside the vm
	@ssh -t $(LOGNAME)@$(VM_IP) "cd $(SOURCE_EXT_DIR); make NAO robot=$(ROBOT_IP);"
#copy the binary back
	@mkdir -p ./$(NAO_BUILD_DIR)
	@scp -prC $(LOGNAME)@$(VM_IP):$(SOURCE_EXT_DIR)/$(NAO_BUILD_DIR)/sdk/lib/naoqi/libnubot.so ./$(NAO_BUILD_DIR)/libnubot.so
	

NAOConfig:
ifeq ($(VM_IP), )
    ifeq ($(SYSTEM),Linux)
		@set -e; \
			cd $(NAO_BUILD_DIR); \
			ccmake .;
    endif
else
	@ssh -t $(LOGNAME)@$(VM_IP) "cd $(SOURCE_EXT_DIR); make NAOConfig;"
endif


NAOClean:
ifeq ($(VM_IP), )
    ifeq ($(SYSTEM),Linux)
		@set -e; \
			echo "Cleaning NAO Build"; \
			cd $(NAO_BUILD_DIR); \
			make $(MAKE_OPTIONS) clean;
    endif
else
	@ssh $(LOGNAME)@$(VM_IP) "cd $(SOURCE_EXT_DIR); make NAOClean;"
endif

NAOVeryClean:
ifeq ($(VM_IP), )
    ifeq ($(SYSTEM),Linux)
		@set -e; \
			echo "Hosing NAO Build"; \
			cd $(NAO_BUILD_DIR); \
			rm -rf ./*;
    endif
else
	@ssh $(LOGNAME)@$(VM_IP) "cd $(SOURCE_EXT_DIR); make NAOVeryClean;"
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
