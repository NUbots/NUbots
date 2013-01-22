
win32{
    INCLUDEPATH += 'C:\Program Files\Boost\boost_1_52_0'
    INCLUDEPATH += 'C:\Program Files\OpenCV\include\opencv2'
    INCLUDEPATH += 'C:\Program Files\OpenCV\modules\core\include'
    INCLUDEPATH += 'C:\Program Files\OpenCV\modules\highgui\include'
    INCLUDEPATH += 'C:\Program Files\OpenCV\modules\imgproc\include'
    DEFINES += TARGET_OS_IS_WINDOWS
}

!macx{
    !win32{
        INCLUDEPATH += /usr/local/include/opencv2/
        INCLUDEPATH += /usr/include/boost/

        LIBS += -lopencv_core -lopencv_highgui -lopencv_imgproc
    }
}

DEFINES += QT_NO_DEBUG_STREAM

INCLUDEPATH += ../
INCLUDEPATH += ../Autoconfig/

PLATFORM = win

contains(PLATFORM, "darwin") {
    message("Compiling for Darwin")
    DEFINES += TARGET_IS_DARWIN
    
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

contains(PLATFORM, "win") {
     message("Compiling for Windows")
    DEFINES += TARGET_IS_PC

    HEADERS += \
        VisionWrapper/datawrapperpc.h \
        VisionWrapper/visioncontrolwrapperpc.h \
        ../Vision/Debug/debugverbosityvision.h \
        ../Vision/Debug/debug.h \
        ../Vision/Debug/nubotdataconfig.h \

    SOURCES += \
        VisionWrapper/datawrapperpc.cpp \
        VisionWrapper/visioncontrolwrapperpc.cpp\
}


contains(PLATFORM, "rpi") {
     message("Compiling for RPi")
    DEFINES += TARGET_IS_RPI

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
    #Threads/SaveImagesThread.h
    GenericAlgorithms/ransac.h \
    Modules/GoalDetectionAlgorithms/goaldetectorhistogram.h \
    Modules/GoalDetectionAlgorithms/goaldetectorransac.h

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
    GenericAlgorithms/ransac.cpp \
    Modules/GoalDetectionAlgorithms/goaldetectorhistogram.cpp \
    Modules/GoalDetectionAlgorithms/goaldetectorransac.cpp
    #Threads/SaveImagesThread.cpp

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
