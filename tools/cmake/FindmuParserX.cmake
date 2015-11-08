# - Try to find libMuParser
# Once done this will define
#
#  MUPARSER_FOUND - system has libMuParser
#  MUPARSER_INCLUDE_DIRS - the libMuParser include directory
#  MUPARSER_LIBRARIES - Link these to use libMuParser
#
#  Copyright (c) 2013 Trent Houliston <trent@houliston.me>
#
#  Redistribution and use is allowed according to the terms of the New
#  BSD license.
#  For details see the accompanying COPYING-CMAKE-SCRIPTS file.
#

INCLUDE(ToolchainLibraryFinder)
ToolchainLibraryFinder(NAME muParserX
                       HEADER mpParser.h
                       LIBRARY muparserx
)