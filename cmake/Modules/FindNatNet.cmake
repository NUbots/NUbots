# - Try to find NatNet
# Once done this will define
#
#  NATNET_FOUND - system has NATNET
#  NATNET_INCLUDE_DIRS - the NATNET include directory
#  NATNET_DEFINITIONS - Compiler switches required for using NATNET
#
#  Copyright (c) 2011 Lee Hambley <lee.hambley@gmail.com>
#
#  Redistribution and use is allowed according to the terms of the New
#  BSD license.
#  For details see the accompanying COPYING-CMAKE-SCRIPTS file.
#

if (NATNET_INCLUDE_DIRS)
  # in cache already
  set(NATNET_FOUND TRUE)
else (NATNET_INCLUDE_DIRS)

  find_path(NATNET_INCLUDE_DIR
    NAMES
      NatNetLinux/CommandListener.h
      NatNetLinux/FrameListener.h
      NatNetLinux/NatNet.h
      NatNetLinux/NatNetPacket.h
      NatNetLinux/NatNetSender.h
    PATHS
      /usr/include
      /usr/local/include
      /opt/local/include
      /sw/include
  )

  set(NATNET_INCLUDE_DIRS
    ${NATNET_INCLUDE_DIR}
  )
  include(FindPackageHandleStandardArgs)
  find_package_handle_standard_args(NATNET DEFAULT_MSG NATNET_INCLUDE_DIRS)

  # show the NATNET_INCLUDE_DIRS variables only in the advanced view
  mark_as_advanced(NATNET_INCLUDE_DIRS)

endif (NATNET_INCLUDE_DIRS)

