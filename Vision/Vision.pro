
win32{
    INCLUDEPATH += 'C:\Program Files\Boost\boost_1_52_0'
    INCLUDEPATH += 'C:\Qwt\qwt-6.0.2\src'
    DEFINES += TARGET_OS_IS_WINDOWS
}

!macx{
    !win32{
        INCLUDEPATH += /usr/include/boost/
        INCLUDEPATH += /usr/include/qwt
    }
}

CONFIG += qwt

DEFINES += QT_NO_DEBUG_STREAM

INCLUDEPATH += ../

win32 {
    PLATFORM = win
}
!win32 {
    PLATFORM = pc
}

contains(PLATFORM, "darwin") {
    message("Compiling for Darwin")
    DEFINES += TARGET_IS_DARWIN

    INCLUDEPATH += ../Autoconfig/

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

    INCLUDEPATH += ../Vision/Debug/

    HEADERS += \
        #VisionWrapper/datawrapperpc.h \
        VisionWrapper/datawrapperqt.h \
        #VisionWrapper/visioncontrolwrapperpc.h\
        VisionWrapper/visioncontrolwrapperqt.h\
        ../Vision/Debug/debugverbosityvision.h \
        ../Vision/Debug/debug.h \
        ../Vision/Debug/nubotdataconfig.h \

    SOURCES += \
        #VisionWrapper/datawrapperpc.cpp \
        VisionWrapper/datawrapperqt.cpp \
        #VisionWrapper/visioncontrolwrapperpc.cpp\
        VisionWrapper/visioncontrolwrapperqt.cpp\

    HEADERS += \
        VisionTools/pccamera.h
    SOURCES += \
        VisionTools/pccamera.cpp
}

contains(PLATFORM, "win") {
     message("Compiling for Windows")
#    DEFINES += TARGET_IS_PC

    INCLUDEPATH += ../Vision/Debug/

    HEADERS += \
        VisionWrapper/datawrapperqt.h \
        ../Vision/Debug/debugverbosityvision.h \
        ../Vision/Debug/debug.h \
        ../Vision/Debug/nubotdataconfig.h \

    SOURCES += \
        VisionWrapper/datawrapperqt.cpp \

    HEADERS += \
        NUPlatform/Platforms/Generic/Cameras/NUOpenCVCamera.h
    SOURCES += \
        NUPlatform/Platforms/Generic/Cameras/NUOpenCVCamera.cpp
}


contains(PLATFORM, "rpi") {
     message("Compiling for RPi")
    DEFINES += TARGET_IS_RPI

    INCLUDEPATH += ../Vision/Debug/

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
    ../Vision/VisionTypes/RANSACTypes/*.h \
    ../Vision/VisionTypes/VisionFieldObjects/*.h \
    ../Vision/VisionTypes/Interfaces/*.h \
    VisionWrapper/datawrappercurrent.h \
    VisionTools/classificationcolours.h \
    VisionTools/GTAssert.h \
    VisionTools/lookuptable.h \
    VisionTools/transformer.h \
    ../Vision/Modules/*.h \
    ../Vision/Modules/LineDetectionAlgorithms/*.h \
    basicvisiontypes.h \
    valgorithm.h \
    visionblackboard.h \
    visioncontroller.h \
    visionconstants.h \
    #Threads/SaveImagesThread.h
    GenericAlgorithms/ransac.h \
    Modules/GoalDetectionAlgorithms/goaldetectorhistogram.h \
    Modules/cornerdetector.h \
    VisionWrapper/mainwindow.h \
    ../Tools/Math/Circle.h \
    linesegmentscurve.h \
    Modules/GoalDetectionAlgorithms/goaldetectorransaccentres.h \
    Modules/GoalDetectionAlgorithms/goaldetectorransacedges.h

SOURCES += \
    ../Vision/VisionTypes/*.cpp \
    ../Vision/VisionTypes/RANSACTypes/*.cpp \
    ../Vision/VisionTypes/VisionFieldObjects/*.cpp \
    VisionTools/lookuptable.cpp \
    ../Vision/Modules/*.cpp \
    VisionTools/transformer.cpp \
    VisionTools/classificationcolours.cpp \
    ../Vision/Modules/LineDetectionAlgorithms/*.cpp \
    visionblackboard.cpp \
    visioncontroller.cpp \
    visionconstants.cpp \
    main.cpp \
    Modules/GoalDetectionAlgorithms/goaldetectorhistogram.cpp \
    basicvisiontypes.cpp \
    GenericAlgorithms/ransac.template \
    VisionWrapper/mainwindow.cpp \
    #../Tools/Math/Circle.cpp
    #Threads/SaveImagesThread.cpp
    linesegmentscurve.cpp \
    Modules/GoalDetectionAlgorithms/goaldetectorransaccentres.cpp \
    Modules/GoalDetectionAlgorithms/goaldetectorransacedges.cpp

##robocup
HEADERS += \
    ../Tools/FileFormats/LUTTools.h \
    ../Tools/Optimisation/Parameter.h \
    ../Tools/Math/Line.h \
    ../Tools/Math/LSFittedLine.h \
    ../Tools/Math/Matrix.h \
    ../Tools/Math/TransformMatrices.h \
    ../Tools/Math/Vector2.h \
    ../Tools/Math/Vector3.h \
    ../Infrastructure/NUImage/NUImage.h \
    ../Infrastructure/NUImage/ColorModelConversions.h \
    ../Infrastructure/NUSensorsData/NUData.h \
    ../Infrastructure/NUSensorsData/NUSensorsData.h \
    ../Infrastructure/NUSensorsData/NULocalisationSensors.h \
    ../Infrastructure/Sensor.h \
    ../NUPlatform/NUCamera/CameraSettings.h \
    ../NUPlatform/NUCamera/NUCameraData.h \
    ../Kinematics/Horizon.h \
    ../Kinematics/Kinematics.h \
    ../Kinematics/EndEffector.h \
    ../Kinematics/Link.h \
    ../NUPlatform/NUCamera.h \
    ../Infrastructure/FieldObjects/Object.h \
    ../Infrastructure/FieldObjects/AmbiguousObject.h \
    ../Infrastructure/FieldObjects/MobileObject.h \
    ../Infrastructure/FieldObjects/StationaryObject.h \

SOURCES += \
    ../Tools/FileFormats/LUTTools.cpp \
    ../Tools/Optimisation/Parameter.cpp \
    ../Tools/Math/Line.cpp \
    ../Tools/Math/LSFittedLine.cpp \
    ../Tools/Math/Matrix.cpp \
    ../Tools/Math/TransformMatrices.cpp \
    ../Infrastructure/NUImage/NUImage.cpp \
    ../Infrastructure/NUData.cpp \
    ../Infrastructure/NUSensorsData/NUSensorsData.cpp \
    ../Infrastructure/NUSensorsData/NULocalisationSensors.cpp \
    ../Infrastructure/NUSensorsData/Sensor.cpp \
    ../NUPlatform/NUCamera/CameraSettings.cpp \
    ../NUPlatform/NUCamera/NUCameraData.cpp \
    ../Kinematics/Horizon.cpp \
    ../Kinematics/Kinematics.cpp \
    ../Kinematics/EndEffector.cpp \
    ../Kinematics/Link.cpp \
    ../NUPlatform/NUCamera.cpp \
    ../Infrastructure/FieldObjects/Object.cpp \
    ../Infrastructure/FieldObjects/AmbiguousObject.cpp \
    ../Infrastructure/FieldObjects/MobileObject.cpp \
    ../Infrastructure/FieldObjects/StationaryObject.cpp \

FORMS += \
    VisionWrapper/mainwindow.ui
