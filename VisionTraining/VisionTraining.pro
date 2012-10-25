INCLUDEPATH += /usr/local/include/opencv2/
INCLUDEPATH += /usr/include/boost/

LIBS += -lopencv_core -lopencv_highgui -lopencv_imgproc

DEFINES += TARGET_IS_TRAINING
DEFINES += QT_NO_DEBUG_STREAM

INCLUDEPATH += ${HOME}/robocup/
INCLUDEPATH += ${HOME}/robocup/Vision/Debug/

HEADERS += \
    mainwindow.h \
    labelgenerator.h \
    ../Vision/VisionTypes/VisionFieldObjects/fieldline.h \
    labeleditor.h \
    visionoptimiser.h \
    visioncomparitor.h \
    optimiserselectwindow.h

SOURCES += \
    mainwindow.cpp \
    labelgenerator.cpp \
    labeleditor.cpp \
    visionoptimiser.cpp \
    visioncomparitor.cpp \
    optimiserselectwindow.cpp

HEADERS += \
    ../Vision/VisionTools/pccamera.h \
    ../Vision/VisionWrapper/datawrappertraining.h \
    ../Vision/VisionWrapper/visioncontrolwrappertraining.h \
    ../Vision/Debug/debugverbosityvision.h \
    ../Vision/Debug/debugverbositynusensors.h \
    ../Vision/Debug/debug.h \
    ../Vision/Debug/nubotdataconfig.h \
    ../Vision/Debug/debugverbositynusensors.h \
    ../Vision/Debug/debugverbositynuactionators.h

SOURCES += \
    ../Vision/VisionTools/pccamera.cpp \
    ../Vision/VisionWrapper/datawrappertraining.cpp \
    ../Vision/VisionWrapper/visioncontrolwrappertraining.cpp \


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
    ../Vision/visionblackboard.h \
    ../Vision/visioncontroller.h \
    ../Vision/visionconstants.h \

SOURCES += \
    ../Vision/VisionTypes/*.cpp \
    ../Vision/VisionTypes/VisionFieldObjects/*.cpp \
    ../Vision/VisionTools/lookuptable.cpp \
    ../Vision/Modules/*.cpp \
    ../Vision/Modules/LineDetectionAlgorithms/*.cpp \
    ../Vision/visionblackboard.cpp \
    ../Vision/visioncontroller.cpp \
    ../Vision/visionconstants.cpp \
    main.cpp \

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
    ../Tools/Optimisation/Optimiser.h \
    ../Tools/Optimisation/EHCLSOptimiser.h \
    ../Tools/Optimisation/PGRLOptimiser.h \
    ../Tools/Optimisation/PSOOptimiser.h \
    ../Tools/Optimisation/PGAOptimiser.h \
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
    ../Infrastructure/NUSensorsData/Sensor.h \
    ../Infrastructure/NUSensorsData/NULocalisationSensors.h \
    ../Infrastructure/NUData.h \
    ../Tools/FileFormats/TimestampedData.h \
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
    ../Tools/Optimisation/Optimiser.cpp \
    ../Tools/Optimisation/EHCLSOptimiser.cpp \
    ../Tools/Optimisation/PGRLOptimiser.cpp \
    ../Tools/Optimisation/PSOOptimiser.cpp \
    ../Tools/Optimisation/PGAOptimiser.cpp \
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
    ../Infrastructure/NUSensorsData/Sensor.cpp \
    ../Infrastructure/NUSensorsData/NULocalisationSensors.cpp \
    ../Infrastructure/NUData.cpp \
    ../Kinematics/Kinematics.cpp \
    ../Kinematics/EndEffector.cpp \
    ../Kinematics/Link.cpp \

FORMS += \
    mainwindow.ui \
    labelgenerator.ui \
    labeleditor.ui \
    visionoptimiser.ui \
    visioncomparitor.ui \
    optimiserselectwindow.ui
