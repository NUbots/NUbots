# Top-level source list
#   - add your source directory's sources.cmake here
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

# list the project's subdirectories here:
SET(NUBOT_DIRS  NUPlatform
                NUbot
                Infrastructure
                Tools
                Kinematics
                ConfigSystem
)
IF(NUBOT_USE_VISION)
	LIST(APPEND NUBOT_DIRS Vision)
ENDIF()

IF(NUBOT_USE_LOCALISATION)
	LIST(APPEND NUBOT_DIRS Localisation)
ENDIF()

LIST(APPEND NUBOT_DIRS Behaviour)

IF(NUBOT_USE_MOTION)
	LIST(APPEND NUBOT_DIRS Motion)
ELSE()
    LIST(APPEND NUBOT_SRCS ../Motion/Walks/WalkParameters.cpp ../Motion/Walks/WalkParameters.h)
    LIST(APPEND NUBOT_DIRS Motion/Tools)
ENDIF()

# list the top-level files here
LIST(APPEND NUBOT_SRCS  ../NUbot.cpp ../NUbot.h
)

# I will add the cmake/sources.cmake to the specified directories in NUBOT_DIRS
INCLUDE_DIRECTORIES(../)
INCLUDE_DIRECTORIES(../Autoconfig)
FOREACH (loop_var ${NUBOT_DIRS})
    INCLUDE(../${loop_var}/cmake/sources.cmake)
ENDFOREACH (loop_var ${NUBOT_DIRS})

# Include some libraries!
LIST(APPEND NUBOT_LINK_LIBRARIES boost_filesystem-mt)
LIST(APPEND NUBOT_LINK_LIBRARIES boost_iostreams-mt)
LIST(APPEND NUBOT_LINK_LIBRARIES zmq)
LIST(APPEND NUBOT_LINK_LIBRARIES protobuf)
#LIST(APPEND NUBOT_LINK_LIBRARIES boost_date_time)

# Build protocol buffer classes
#find_package(Protobuf REQUIRED)
#include_directories(${PROTOBUF_INCLUDE_DIRS})
#file(GLOB ProtoFiles "/home/darwin/robocup/NUPlatform/NUAPI/proto/*.proto")
#PROTOBUF_GENERATE_CPP(ProtoSources ProtoHeaders ${ProtoFiles})
