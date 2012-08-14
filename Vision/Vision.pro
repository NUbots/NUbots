QT -= gui
QT -= core

INCLUDEPATH += /usr/local/include/opencv2/
INCLUDEPATH += /usr/include/boost/

LIBS += -lopencv_core -lopencv_highgui

PLATFORM = pc

contains(PLATFORM, "darwin") {
    message("Compiling for Darwin")
    DEFINES += TARGET_IS_DARWIN

    ROBOCUP_DIR = ${HOME}/robocup/            #change to darwin directory later
    #ROBOCUP_DIR = /home/shannon/robocup/
    #ROBOCUP_DIR = /home/david/robocup/

    INCLUDEPATH += $$ROBOCUP_DIR
    INCLUDEPATH += $$ROBOCUP_DIR/Autoconfig/
    
    HEADERS += \
        NUPlatform/Platforms/Darwin/DarwinCamera.h \
        VisionWrapper/datawrapperdarwin.h \
        $$ROBOCUP_DIR/Autoconfig/debug.h \
        $$ROBOCUP_DIR/Autoconfig/nubotdataconfig.h \
        VisionWrapper/visioncontrolwrapperdarwin.h \
    
    SOURCES += \
        $$ROBOCUP_DIR/NUPlatform/Platforms/Darwin/DarwinCamera.cpp \
        VisionWrapper/datawrapperdarwin.cpp \
        VisionWrapper/visioncontrolwrapperdarwin.cpp \
}

contains(PLATFORM, "pc") {
     message("Compiling for PC")
    DEFINES += TARGET_IS_PC

    ROBOCUP_DIR = ${HOME}/robocup/
    #ROBOCUP_DIR = /home/shannon/robocup/
    #ROBOCUP_DIR = /home/david/robocup/

    INCLUDEPATH += $$ROBOCUP_DIR/
    INCLUDEPATH += $$ROBOCUP_DIR/Vision/Debug/
  
    HEADERS += \
        VisionTools/pccamera.h \
        VisionWrapper/datawrapperpc.h \
        VisionWrapper/visioncontrolwrapperpc.h \
        $$ROBOCUP_DIR/Vision/Debug/debugverbosityvision.h \
        $$ROBOCUP_DIR/Vision/Debug/debug.h \
        $$ROBOCUP_DIR/Vision/Debug/nubotdataconfig.h \
    
    SOURCES += \
        VisionTools/pccamera.cpp \
        VisionWrapper/datawrapperpc.cpp \
        VisionWrapper/visioncontrolwrapperpc.cpp\
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
    visionconstants.h
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
    #Threads/SaveImagesThread.cpp

##robocup
HEADERS += \
    $$ROBOCUP_DIR/Tools/FileFormats/LUTTools.h \
    $$ROBOCUP_DIR/Tools/Optimisation/Parameter.h \
    $$ROBOCUP_DIR/Tools/Math/Line.h \
    $$ROBOCUP_DIR/Tools/Math/LSFittedLine.h \
    $$ROBOCUP_DIR/Tools/Math/Matrix.h \
    $$ROBOCUP_DIR/Tools/Math/TransformMatrices.h \
    $$ROBOCUP_DIR/Infrastructure/NUImage/NUImage.h \
    $$ROBOCUP_DIR/NUPlatform/NUCamera/CameraSettings.h \
    $$ROBOCUP_DIR/NUPlatform/NUCamera/NUCameraData.h \
    $$ROBOCUP_DIR/Kinematics/Horizon.h \
    $$ROBOCUP_DIR/NUPlatform/NUCamera.h \
    $$ROBOCUP_DIR/Infrastructure/FieldObjects/Object.h \
    $$ROBOCUP_DIR/Infrastructure/FieldObjects/AmbiguousObject.h \
    $$ROBOCUP_DIR/Infrastructure/FieldObjects/MobileObject.h \
    $$ROBOCUP_DIR/Infrastructure/FieldObjects/StationaryObject.h \
    $$ROBOCUP_DIR/Kinematics/Kinematics.h \
    $$ROBOCUP_DIR/Kinematics/EndEffector.h \
    $$ROBOCUP_DIR/Kinematics/Link.h \

SOURCES += \
    $$ROBOCUP_DIR/Tools/FileFormats/LUTTools.cpp \
    $$ROBOCUP_DIR/Tools/Optimisation/Parameter.cpp \
    $$ROBOCUP_DIR/Tools/Math/Line.cpp \
    $$ROBOCUP_DIR/Tools/Math/LSFittedLine.cpp \
    $$ROBOCUP_DIR/Tools/Math/Matrix.cpp \
    $$ROBOCUP_DIR/Tools/Math/TransformMatrices.cpp \
    $$ROBOCUP_DIR/Infrastructure/NUImage/NUImage.cpp \
    $$ROBOCUP_DIR/NUPlatform/NUCamera/CameraSettings.cpp \
    $$ROBOCUP_DIR/NUPlatform/NUCamera/NUCameraData.cpp \
    $$ROBOCUP_DIR/Kinematics/Horizon.cpp \
    $$ROBOCUP_DIR/NUPlatform/NUCamera.cpp \
    $$ROBOCUP_DIR/Infrastructure/FieldObjects/Object.cpp \
    $$ROBOCUP_DIR/Infrastructure/FieldObjects/AmbiguousObject.cpp \
    $$ROBOCUP_DIR/Infrastructure/FieldObjects/MobileObject.cpp \
    $$ROBOCUP_DIR/Infrastructure/FieldObjects/StationaryObject.cpp \
    $$ROBOCUP_DIR/Kinematics/Kinematics.cpp \
    $$ROBOCUP_DIR/Kinematics/EndEffector.cpp \
    $$ROBOCUP_DIR/Kinematics/Link.cpp \
