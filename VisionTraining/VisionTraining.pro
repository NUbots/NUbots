QT -= gui
QT -= core

INCLUDEPATH += /usr/local/include/opencv2/
INCLUDEPATH += /usr/include/boost/

LIBS += -lopencv_core -lopencv_highgui

DEFINES += TARGET_IS_TRAINING

INCLUDEPATH += ${HOME}/robocup/
INCLUDEPATH += ${HOME}/robocup/Vision/Debug/

HEADERS += \
    ../Vision/VisionTools/pccamera.h \
    ../Vision/VisionWrapper/datawrappertraining.h \
    ../Vision/VisionWrapper/visioncontrolwrappertraining.h \
    ../Vision/Debug/debugverbosityvision.h \
    ../Vision/Debug/debugverbositynusensors.h \
    ../Vision/Debug/debug.h \
    ../Vision/Debug/nubotdataconfig.h \
    ../Vision/Debug/debugverbositynusensors.h

SOURCES += \
    ../Vision/VisionTools/pccamera.cpp \
    ../Vision/VisionWrapper/datawrappertraining.cpp \
    ../Vision/VisionWrapper/visioncontrolwrappertraining.cpp\


HEADERS += \
    ../Vision/VisionTypes/*.h \
    ../Vision/VisionTypes/VisionFieldObjects/*.h \
    ../Vision/VisionWrapper/datawrappercurrent.h \
    ../Vision/VisionTools/classificationcolours.h \
    ../Vision/VisionTools/GTAssert.h \
    ../Vision/VisionTools/lookuptable.h \
    ../Vision/Modules/*.h \
    ../Vision/Modules/LineDetectionAlgorithms/*.h \
    ../Vision/basicvisiontypes.h \
    ../Vision/Visionblackboard.h \
    ../Vision/Visioncontroller.h \
    ../Vision/Visionconstants.h \

SOURCES += \
    ../Vision/VisionTypes/*.cpp \
    ../Vision/VisionTypes/VisionFieldObjects/*.cpp \
    ../Vision/VisionTools/lookuptable.cpp \
    ../Vision/Modules/*.cpp \
    ../Vision/Modules/LineDetectionAlgorithms/*.cpp \
    ../Vision/Visionblackboard.cpp \
    ../Vision/Visioncontroller.cpp \
    ../Vision/Visionconstants.cpp \
    main.cpp \

##robocup
HEADERS += \
    ../Tools/FileFormats/LUTTools.h \
    ../Tools/Optimisation/Parameter.h \
    ../Tools/Math/Line.h \
    ../Tools/Math/LSFittedLine.h \
    ../Tools/Math/Matrix.h \
    ../Tools/Math/TransformMatrices.h \
    ../Infrastructure/NUImage/NUImage.h \
    ../NUPlatform/NUCamera/CameraSettings.h \
    ../NUPlatform/NUCamera/NUCameraData.h \
    ../Kinematics/Horizon.h \
    ../NUPlatform/NUCamera.h \
    ../Infrastructure/FieldObjects/Object.h \
    ../Infrastructure/FieldObjects/AmbiguousObject.h \
    ../Infrastructure/FieldObjects/MobileObject.h \
    ../Infrastructure/FieldObjects/StationaryObject.h \
    ../Infrastructure/NUSensorsData/NUSensorsData.h \
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
    ../Infrastructure/NUSensorsData/NUSensorsData.cpp \
    ../Kinematics/Kinematics.cpp \
    ../Kinematics/EndEffector.cpp \
    ../Kinematics/Link.cpp \
