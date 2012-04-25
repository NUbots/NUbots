QT -= gui
QT -= core

INCLUDEPATH += /usr/local/include/opencv2/
INCLUDEPATH += /usr/include/boost/

LIBS += -lopencv_core -lopencv_highgui

PLATFORM = pc

contains(PLATFORM, "darwin") {
    message("Compiling for Darwin")

    ROBOCUP_DIR = /home/shannon/robocup/            #change to darwin directory later
    INCLUDEPATH += $$ROBOCUP_DIR
    
    HEADERS += \
        NUPlatform/Platforms/Darwin/DarwinCamera.h \
        VisionWrapper/datawrapperdarwin.h \
    
    SOURCES += \
        $$ROBOCUP_DIR\NUPlatform/Platforms/Darwin/DarwinCamera.cpp \
        VisionWrapper/datawrapperdarwin.cpp \
}

contains(PLATFORM, "pc") {
     message("Compiling for PC")

    ROBOCUP_DIR = /home/shannon/robocup/ 
    INCLUDEPATH += $$ROBOCUP_DIR
  
    HEADERS += \
        VisionTools/pccamera.h \
        VisionWrapper/datawrapperpc.h \
    
    SOURCES += \
        VisionTools/pccamera.cpp \
        VisionWrapper/datawrapperpc.cpp \
}

#this
HEADERS += \
    visionblackboard.h \
    visioncontroller.h \
    VisionTools/lookuptable.h \
    VisionWrapper/datawrappercurrent.h \
    VisionTools/GTAssert.h \
    VisionTools/classificationcolours.h \
    Modules/greenhorizonch.h \
    Modules/horizoninterpolate.h \
    Modules/objectdetectionch.h \
    Modules/scanlines.h \
    Modules/segmentfilter.h \
    VisionTypes/coloursegment.h \
    VisionTypes/colourtransitionrule.h \
    VisionTypes/colourreplacementrule.h \
    VisionTypes/transition.h \
    VisionTypes/segmentedregion.h \
    VisionTypes/visionfieldobject.h \
    VisionTypes/objectcandidate.h \
    debugvisionverbosities.h \
    basicvisiontypes.h \
    valgorithm.h \
    Modules/clustercandidates.h \
    VisionWrapper/visioncontrolwrapper.h

SOURCES += \
    visionblackboard.cpp \
    visioncontroller.cpp \
    VisionTools/lookuptable.cpp \
    Modules/greenhorizonch.cpp \
    Modules/horizoninterpolate.cpp \
    Modules/objectdetectionch.cpp \
    Modules/scanlines.cpp \
    Modules/segmentfilter.cpp \
    VisionTypes/coloursegment.cpp \
    VisionTypes/colourtransitionrule.cpp \
    VisionTypes/colourreplacementrule.cpp \
    VisionTypes/transition.cpp \
    VisionTypes/segmentedregion.cpp \
    VisionTypes/visionfieldobject.cpp \
    VisionTypes/objectcandidate.cpp \
    main.cpp \
    Modules/clustercandidates.cpp \
    VisionWrapper/visioncontrolwrapper.cpp

#robocup
HEADERS += \
    $$ROBOCUP_DIR/Tools/FileFormats/LUTTools.h \
    $$ROBOCUP_DIR/Tools/Optimisation/Parameter.h \
    $$ROBOCUP_DIR/Tools/Math/Line.h \
    $$ROBOCUP_DIR/Infrastructure/NUImage/NUImage.h \
    $$ROBOCUP_DIR/NUPlatform/NUCamera/CameraSettings.h \
    debug.h \

SOURCES += \
    $$ROBOCUP_DIR/Tools/FileFormats/LUTTools.cpp \
    $$ROBOCUP_DIR/Tools/Optimisation/Parameter.cpp \
    $$ROBOCUP_DIR/Tools/Math/Line.cpp \
    $$ROBOCUP_DIR/Infrastructure/NUImage/NUImage.cpp \
    $$ROBOCUP_DIR/NUPlatform/NUCamera/CameraSettings.cpp \
