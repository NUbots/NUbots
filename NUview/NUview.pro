QT += network \
    opengl
DEFINES += TARGET_IS_NUVIEW
macx { 
    # Mac Specific Includes
    QMAKE_LFLAGS += -F/System/Library/Frameworks/CoreFoundation.framework/
    LIBS += -framework \
        CoreFoundation \
        -L \
        /usr/lib/libz.dylib
}
win32 { 
    INCLUDEPATH += 'C:/Program Files (x86)/boost/'
    INCLUDEPATH += 'C:/Program Files (x86)/boost/boost_1_42'
    INCLUDEPATH += 'C:/Qt/2010.02.1/qt/src/3rdparty/zlib'
    INCLUDEPATH += 'C:/Program Files/boost/'
    LIBS += -lwsock32
    LIBS += -lpthread
    DEFINES += TARGET_OS_IS_WINDOWS
}
!macx:LIBS += -ldns_sd

# Opencv library
# INCLUDEPATH += "C:\Program Files\OpenCV\cv\include" "C:\Program Files\OpenCV\cvaux\include" "C:\Program Files\OpenCV\cxcore\include" "C:\Program Files\OpenCV\otherlibs\highgui"
# LIBS += -L"C:\Program Files\OpenCV\lib" -lcv -lcvaux -lhighgui -lcxcore
# Input
# HEADERS += "C:\Program Files\OpenCV\cv\include\cv.h" "C:\Program Files\OpenCV\otherlibs\highgui\highgui.h"
INCLUDEPATH += ../
INCLUDEPATH += NUviewconfig/
HEADERS += ui_mainwindow.h \
    mainwindow.h \
    connectionwidget.h \
    ColorModelConversions.h \
    classificationwidget.h \
    ../Vision/ClassificationColours.h \
    ../Tools/FileFormats/NUbotImage.h \
    ../Vision/Vision.h \
    ../Tools/FileFormats/LUTTools.h \
    virtualnubot.h \
    ../Tools/Image/BresenhamLine.h \
    ../Tools/Math/Vector2.h \
    ../Tools/Math/Line.h \
    ../Kinematics/Horizon.h \
    openglmanager.h \
    GLDisplay.h \
    ../Tools/Image/NUimage.h \
    ../Tools/Image/ClassifiedImage.h \
    ../Vision/ClassifiedSection.h \
    ../Vision/ScanLine.h \
    ../Vision/TransitionSegment.h \
    ../Vision/GoalDetection.h \
    LayerSelectionWidget.h \
    WalkParameterWidget.h \
    KickWidget.h \
    locWmGlDisplay.h \
    ../Vision/LineDetection.h \
    ../Tools/Math/LSFittedLine.h \
    ../Tools/Math/Vector3.h \
    ../Vision/FieldObjects/StationaryObject.h \
    ../Vision/FieldObjects/Self.h \
    ../Vision/FieldObjects/Object.h \
    ../Vision/FieldObjects/MobileObject.h \
    ../Vision/FieldObjects/AmbiguousObject.h \
    ../Vision/FieldObjects/FieldObjects.h \
    ../Vision/Threads/SaveImagesThread.h \
    ../Vision/ObjectCandidate.h \
    ../Localisation/WMPoint.h \
    ../Localisation/WMLine.h \
    ../Localisation/sphere.h \
    ../Localisation/cylinder.h \
    ../Localisation/cameramatrix.h \
    ../Tools/Math/matrix.h \
    localisationwidget.h \
    ../Vision/Ball.h \
    ../Vision/CircleFitting.h \
    FileAccess/LogFileFormatReader.h \
    FileAccess/nifVersion1FormatReader.h \
    FileAccess/LogFileReader.h \
    FileAccess/nulVersion1FormatReader.h \
    visionstreamwidget.h \
    camerasettingswidget.h \
    ../NUPlatform/NUCamera/CameraSettings.h \
    ../Tools/FileFormats/Parse.h \
    ../Localisation/LocWM.h \
    ../Localisation/KF.h \
    ../Vision/FieldObjects/WorldModelShareObject.h \
    ../GameController/GameInformation.h \
    ../Tools/Threading/Thread.h \
    ../Tools/Threading/ConditionalThread.h \
    NUviewIO/NUviewIO.h \
    ../Kinematics/Kinematics.h \
    ../Tools/Math/TransformMatrices.h \
    frameInformationWidget.h \
    bonjour/robotSelectDialog.h \
    bonjour/bonjourserviceresolver.h \
    bonjour/bonjourservicebrowser.h \
    bonjour/bonjourrecord.h \
    ../Tools/Math/UKF.h \
    ../Tools/Math/SRUKF.h \
    ../Kinematics/Link.h \
    ../Kinematics/EndEffector.h \
    ../NUPlatform/NUSensors.h \
    ../NUPlatform/NUSensors/NUSensorsData.h \
    ../Tools/Math/General.h \
    ../Vision/CornerPoint.h \
    ../Kinematics/OrientationUKF.h \
    FileAccess/StreamFileReader.h \
    ../Tools/FileFormats/TimestampedData.h \
    FileAccess/NavigableFileReader.h \
    FileAccess/ImageStreamFileReader.h
SOURCES += mainwindow.cpp \
    main.cpp \
    connectionwidget.cpp \
    classificationwidget.cpp \
    ../Tools/FileFormats/NUbotImage.cpp \
    ../Vision/Vision.cpp \
    ../Tools/FileFormats/LUTTools.cpp \
    virtualnubot.cpp \
    ../Tools/Image/BresenhamLine.cpp \
    ../Tools/Math/Line.cpp \
    ../Kinematics/Horizon.cpp \
    openglmanager.cpp \
    GLDisplay.cpp \
    ../Tools/Image/NUimage.cpp \
    ../Tools/Image/ClassifiedImage.cpp \
    ../Vision/ClassifiedSection.cpp \
    ../Vision/ScanLine.cpp \
    ../Vision/TransitionSegment.cpp \
    ../Vision/GoalDetection.cpp \
    LayerSelectionWidget.cpp \
    WalkParameterWidget.cpp \
    KickWidget.cpp \
    ../Motion/Walks/WalkParameters.cpp \
    ../Motion/Tools/MotionFileTools.cpp \
    ../NUPlatform/NUIO.cpp \
    ../NUPlatform/NUIO/*.cpp \
    NUviewIO/NUviewIO.cpp \
    ../NUPlatform/NUSensors.cpp \
    ../NUPlatform/NUSensors/*.cpp \
    ../NUPlatform/NUActionators.cpp \
    ../NUPlatform/NUActionators/*.cpp \
    ../NUPlatform/NUSystem.cpp \
    ../Behaviour/TeamInformation.cpp \
    ../Behaviour/Jobs/*.cpp \
    ../Behaviour/Jobs/CameraJobs/*.cpp \
    ../Behaviour/Jobs/VisionJobs/*.cpp \
    ../Behaviour/Jobs/MotionJobs/*.cpp \
    locWmGlDisplay.cpp \
    ../Vision/ObjectCandidate.cpp \
    ../Vision/LineDetection.cpp \
    ../Tools/Math/LSFittedLine.cpp \
    ../Vision/FieldObjects/StationaryObject.cpp \
    ../Vision/FieldObjects/Self.cpp \
    ../Vision/FieldObjects/Object.cpp \
    ../Vision/FieldObjects/MobileObject.cpp \
    ../Vision/FieldObjects/AmbiguousObject.cpp \
    ../Vision/FieldObjects/FieldObjects.cpp \
    ../Vision/Threads/SaveImagesThread.cpp \
    ../Localisation/WMPoint.cpp \
    ../Localisation/WMLine.cpp \
    ../Localisation/sphere.cpp \
    ../Localisation/cylinder.cpp \
    ../Localisation/cameramatrix.cpp \
    ../Tools/Math/matrix.cpp \
    localisationwidget.cpp \
    ../Vision/Ball.cpp \
    ../Vision/CircleFitting.cpp \
    FileAccess/LogFileFormatReader.cpp \
    FileAccess/nifVersion1FormatReader.cpp \
    FileAccess/LogFileReader.cpp \
    FileAccess/nulVersion1FormatReader.cpp \
    visionstreamwidget.cpp \
    camerasettingswidget.cpp \
    ../NUPlatform/NUCamera/CameraSettings.cpp \
    ../Tools/FileFormats/Parse.cpp \
    ../Localisation/LocWM.cpp \
    ../Localisation/KF.cpp \
    ../Vision/FieldObjects/WorldModelShareObject.cpp \
    ../GameController/GameInformation.cpp \
    ../Tools/Threading/Thread.cpp \
    ../Tools/Threading/ConditionalThread.cpp \
    ../Kinematics/Kinematics.cpp \
    ../Tools/Math/TransformMatrices.cpp \
    frameInformationWidget.cpp \
    bonjour/robotSelectDialog.cpp \
    bonjour/bonjourserviceresolver.cpp \
    bonjour/bonjourservicebrowser.cpp \
    ../Tools/Math/UKF.cpp \
    ../Tools/Math/SRUKF.cpp \
    ../Kinematics/Link.cpp \
    ../Kinematics/EndEffector.cpp \
    ../Kinematics/OrientationUKF.cpp
RESOURCES = textures.qrc
RESOURCES += icons.qrc
