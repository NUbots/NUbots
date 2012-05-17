QT -= gui
QT -= core

INCLUDEPATH += /usr/local/include/opencv2/
INCLUDEPATH += /usr/include/boost/

LIBS += -lopencv_core -lopencv_highgui

PLATFORM = pc

contains(PLATFORM, "darwin") {
    message("Compiling for Darwin")

    ROBOCUP_DIR = /home/david/robocup/            #change to darwin directory later
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

    ROBOCUP_DIR = /home/david/robocup/
    INCLUDEPATH += $$ROBOCUP_DIR
    INCLUDEPATH += $$ROBOCUP_DIR/Vision/Debug/
  
    HEADERS += \
        VisionTools/pccamera.h \
        VisionWrapper/datawrapperpc.h \
        VisionWrapper/visioncontrolwrapperpc.h \
        debugverbosityvision.h \
        debug.h \
        nubotdataconfig.h \
    
    SOURCES += \
        VisionTools/pccamera.cpp \
        VisionWrapper/datawrapperpc.cpp \
        VisionWrapper/visioncontrolwrapperpc.cpp\
}


#this
HEADERS += \
    VisionTypes/coloursegment.h \
    VisionTypes/colourtransitionrule.h \
    VisionTypes/colourreplacementrule.h \
    VisionTypes/transition.h \
    VisionTypes/segmentedregion.h \
    VisionTypes/objectcandidate.h \
    VisionTypes/quad.h \
    VisionTypes/VisionFieldObjects/visionfieldobject.h \
    VisionTypes/VisionFieldObjects/ball.h \
    VisionTypes/VisionFieldObjects/goal.h \
    VisionTypes/VisionFieldObjects/beacon.h \
    VisionTypes/VisionFieldObjects/obstacle.h \
    VisionWrapper/datawrappercurrent.h \
    VisionTools/lookuptable.h \
    VisionTools/GTAssert.h \
    VisionTools/classificationcolours.h \
    Modules/greenhorizonch.h \
    Modules/horizoninterpolate.h \
    Modules/objectdetectionch.h \
    Modules/scanlines.h \
    Modules/segmentfilter.h \
    Modules/goaldetection.h \
    basicvisiontypes.h \
    valgorithm.h \
    visionblackboard.h \
    visioncontroller.h \ 
    visionconstants.h

SOURCES += \
    VisionTypes/coloursegment.cpp \
    VisionTypes/colourtransitionrule.cpp \
    VisionTypes/colourreplacementrule.cpp \
    VisionTypes/transition.cpp \
    VisionTypes/segmentedregion.cpp \
    VisionTypes/objectcandidate.cpp \
    VisionTypes/quad.cpp \
    VisionTypes/VisionFieldObjects/visionfieldobject.cpp \
    VisionTypes/VisionFieldObjects/ball.cpp \
    VisionTypes/VisionFieldObjects/goal.cpp \
    VisionTypes/VisionFieldObjects/beacon.cpp \
    VisionTypes/VisionFieldObjects/obstacle.cpp \
    VisionTools/lookuptable.cpp \
    Modules/greenhorizonch.cpp \
    Modules/horizoninterpolate.cpp \
    Modules/objectdetectionch.cpp \
    Modules/scanlines.cpp \
    Modules/segmentfilter.cpp \
    Modules/goaldetection.cpp \
    visionblackboard.cpp \
    visioncontroller.cpp \
    visionconstants.cpp \
    main.cpp \

##robocup
HEADERS += \
    $$ROBOCUP_DIR/Tools/FileFormats/LUTTools.h \
    $$ROBOCUP_DIR/Tools/Optimisation/Parameter.h \
    $$ROBOCUP_DIR/Tools/Math/Line.h \
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
