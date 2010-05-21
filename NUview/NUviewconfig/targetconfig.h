/*! @file targetconfig.h
    @brief A configuration file that sets platform target variables.
    
    The following preprocessor variables are set here:
        - TARGET_IS_
        - MY_OS_IS_
        - TARGET_OS_IS_

    @author Jason Kulk
 */
#ifndef TARGETCONFIG_H
#define TARGETCONFIG_H

// define the target robotic platform
#define TARGET_IS_NUVIEW            //!< Preprocessor define for determine the target platform. Will be TARGET_IS_NAO, TARGET_IS_NAOWEBOTS, TARGET_IS_CYCLOID, etc

// define the os the code is being compiled on
#if defined(WIN32)
    #define MY_OS_Windows
#elif defined(__APPLE__)
    #define MY_OS_Darwin
#else
    #define MY_OS_Linux
#endif

// define the right MY_OS_IS_XXXXXXXX
#ifdef MY_OS_Windows
    #define MY_OS_IS_WINDOWS                    //!< This will be defined only this is being configured on a Windows machine
#else
    #undef MY_OS_IS_WINDOWS
#endif
#ifdef MY_OS_Darwin
    #define MY_OS_IS_DARWIN                     //!< This will be defined only when the build is configured on a OS-X machine
#else
    #undef MY_OS_IS_DARWIN
#endif
#ifdef MY_OS_Linux
    #define MY_OS_IS_LINUX                      //!< This will be defined only when the build is configured on a Linux machine
#else
    #undef MY_OS_IS_LINUX
#endif

#ifdef MY_OS_IS_WINDOWS
    #define TARGET_OS_IS_WINDOWS            //!< This will be defined if the target's os is Windows
#else
    #undef TARGET_OS_IS_WINDOWS
#endif
#ifdef MY_OS_IS_DARWIN
    #define TARGET_OS_IS_DARWIN             //!< This will be defined if the target's os is OS-X
#else
    #undef TARGET_OS_IS_DARWIN
#endif
#ifdef MY_OS_IS_LINUX
    #define TARGET_OS_IS_LINUX              //!< This will be defined if the target's os is Linux
#else
    #undef TARGET_OS_IS_LINUX
#endif

#endif // !TARGETCONFIG_H

