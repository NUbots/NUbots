INCLUDEPATH += /usr/local/include/opencv2/
INCLUDEPATH += /usr/include/boost/

LIBS += -lopencv_core -lopencv_highgui -lopencv_imgproc

DEFINES += QT_NO_DEBUG_STREAM

PLATFORM = pc

contains(PLATFORM, "darwin") {
    message("Compiling for Darwin")
    DEFINES += TARGET_IS_DARWIN

    ROBOCUP_DIR = ${HOME}/robocup/            #change to darwin directory later

    INCLUDEPATH += $$ROBOCUP_DIR
    INCLUDEPATH += $$ROBOCUP_DIR/Autoconfig/
    
    HEADERS += \
        NUPlatform/Platforms/Darwin/DarwinCamera.h \
        VisionWrapper/datawrapperdarwin.h \
        ../Autoconfig/debug.h \
        ../Autoconfig/nubotdataconfig.h \
        VisionWrapper/visioncontrolwrapperdarwin.h \
    
    SOURCES += \
        ../NUPlatform/Platforms/Darwin/DarwinCamera.cpp \
        VisionWrapper/datawrapperdarwin.cpp \
        VisionWrapper/visioncontrolwrapperdarwin.cpp \
}

contains(PLATFORM, "pc") {
     message("Compiling for PC")
    DEFINES += TARGET_IS_PC

    ROBOCUP_DIR = ${HOME}/robocup/

    INCLUDEPATH += $$ROBOCUP_DIR/
    INCLUDEPATH += $$ROBOCUP_DIR/Vision/Debug/
  
    HEADERS += \
        VisionTools/pccamera.h \
        VisionWrapper/datawrapperpc.h \
        VisionWrapper/visioncontrolwrapperpc.h \
        ../Vision/Debug/debugverbosityvision.h \
        ../Vision/Debug/debug.h \
        ../Vision/Debug/nubotdataconfig.h \

    SOURCES += \
        VisionTools/pccamera.cpp \
        VisionWrapper/datawrapperpc.cpp \
        VisionWrapper/visioncontrolwrapperpc.cpp\
}

contains(PLATFORM, "rpi") {
     message("Compiling for RPi")
    DEFINES += TARGET_IS_RPI

    ROBOCUP_DIR = ${HOME}/robocup/

    INCLUDEPATH += $$ROBOCUP_DIR/
    INCLUDEPATH += $$ROBOCUP_DIR/Vision/Debug/

    HEADERS += \
        VisionTools/pccamera.h \
        VisionWrapper/datawrapperrpi.h \
        VisionWrapper/visioncontrolwrapperrpi.h \
        ../Vision/Debug/debugverbosityvision.h \
        ../Vision/Debug/debug.h \
        ../Vision/Debug/nubotdataconfig.h \

    SOURCES += \
        VisionTools/pccamera.cpp \
        VisionWrapper/datawrapperrpi.cpp \
        VisionWrapper/visioncontrolwrapperrpi.cpp\
}


#this
HEADERS += \
    ../Vision/VisionTypes/*.h \
    ../Vision/VisionTypes/VisionFieldObjects/*.h \
    VisionWrapper/datawrappercurrent.h \
    VisionTools/classificationcolours.h \
    VisionTools/GTAssert.h \
    VisionTools/lookuptable.h \
    ../Vision/Modules/*.h \
    ../Vision/Modules/LineDetectionAlgorithms/*.h \
    basicvisiontypes.h \
    valgorithm.h \
    visionblackboard.h \
    visioncontroller.h \ 
    visionconstants.h \
    VisionTypes/histogram1D.h \
    Modules/GoalDetectionAlgorithms/transitionhistogramming1d.h
    #Threads/SaveImagesThread.h

SOURCES += \
    ../Vision/VisionTypes/*.cpp \
    ../Vision/VisionTypes/VisionFieldObjects/*.cpp \
    VisionTools/lookuptable.cpp \
    ../Vision/Modules/*.cpp \
    ../Vision/Modules/LineDetectionAlgorithms/*.cpp \
    visionblackboard.cpp \
    visioncontroller.cpp \
    visionconstants.cpp \
    main.cpp \
    Modules/GoalDetectionAlgorithms/transitionhistogramming1d.cpp
    #Threads/SaveImagesThread.cpp

##robocup
HEADERS += \
    ../Tools/FileFormats/LUTTools.h \
    ../Tools/Optimisation/Parameter.h \
    ../Tools/Math/Line.h \
    ../Tools/Math/LSFittedLine.h \
    ../Tools/Math/Matrix.h \
    ../Tools/Math/TransformMatrices.h \
    ../Infrastructure/NUImage/NUImage.h \
    ../Infrastructure/NUImage/ColorModelConversions.h \
    ../NUPlatform/NUCamera/CameraSettings.h \
    ../NUPlatform/NUCamera/NUCameraData.h \
    ../Kinematics/Horizon.h \
    ../NUPlatform/NUCamera.h \
    ../Infrastructure/FieldObjects/Object.h \
    ../Infrastructure/FieldObjects/AmbiguousObject.h \
    ../Infrastructure/FieldObjects/MobileObject.h \
    ../Infrastructure/FieldObjects/StationaryObject.h \
    ../Kinematics/Kinematics.h \
    ../Kinematics/EndEffector.h \
    ../Kinematics/Link.h \

SOURCES += \
    ../Tools/FileFormats/LUTTools.cpp \
    ../Tools/Optimisation/Parameter.cpp \
    ../Tools/Math/Line.cpp \
    ../Tools/Math/LSFittedLine.cpp \
    ../Tools/Math/Matrix.cpp \
    ../Tools/Math/TransformMatrices.cpp \
    ../Infrastructure/NUImage/NUImage.cpp \
    ../NUPlatform/NUCamera/CameraSettings.cpp \
    ../NUPlatform/NUCamera/NUCameraData.cpp \
    ../Kinematics/Horizon.cpp \
    ../NUPlatform/NUCamera.cpp \
    ../Infrastructure/FieldObjects/Object.cpp \
    ../Infrastructure/FieldObjects/AmbiguousObject.cpp \
    ../Infrastructure/FieldObjects/MobileObject.cpp \
    ../Infrastructure/FieldObjects/StationaryObject.cpp \
    ../Kinematics/Kinematics.cpp \
    ../Kinematics/EndEffector.cpp \
    ../Kinematics/Link.cpp \
