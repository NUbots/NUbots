QT += network \
    opengl
macx { 
    # Mac Specific Includes
    QMAKE_LFLAGS += -F/System/Library/Frameworks/CoreFoundation.framework/
    LIBS += -framework \
        CoreFoundation \
        -lz
}
win32 { 
    INCLUDEPATH += 'C:/Program Files (x86)/boost/'
    INCLUDEPATH += 'C:/Program Files (x86)/boost/boost_1_42'
    INCLUDEPATH += 'C:/Qt/2010.02.1/qt/src/3rdparty/zlib'
    INCLUDEPATH += 'C:/Program Files (x86)/boost/boost_1_44'
    INCLUDEPATH += 'C:/Qt/2010.05/qt/src/3rdparty/zlib'
    INCLUDEPATH += 'C:/Program Files/boost/boost_1_44'
    INCLUDEPATH += 'C:/Program Files/boost/'
    INCLUDEPATH += 'C:/Program Files/Bonjour SDK/Include/'
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
    ../Infrastructure/NUImage/BresenhamLine.h \
    ../Tools/Math/Vector2.h \
    ../Tools/Math/Line.h \
    ../Kinematics/Horizon.h \
    openglmanager.h \
    GLDisplay.h \
    ../Infrastructure/NUImage/NUImage.h \
    ../Infrastructure/NUImage/ClassifiedImage.h \
    ../Vision/ClassifiedSection.h \
    ../Vision/ScanLine.h \
    ../Vision/TransitionSegment.h \
    ../Vision/GoalDetection.h \
    LayerSelectionWidget.h \
    WalkParameterWidget.h \
    ../Motion/Walks/WalkParameters.h \
    ../Tools/Optimisation/Parameter.h \
    KickWidget.h \
    locWmGlDisplay.h \
    ../Vision/LineDetection.h \
    ../Tools/Math/LSFittedLine.h \
    ../Tools/Math/Vector3.h \
    ../Infrastructure/FieldObjects/StationaryObject.h \
    ../Infrastructure/FieldObjects/Self.h \
    ../Infrastructure/FieldObjects/Object.h \
    ../Infrastructure/FieldObjects/MobileObject.h \
    ../Infrastructure/FieldObjects/AmbiguousObject.h \
    ../Infrastructure/FieldObjects/FieldObjects.h \
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
    ../Localisation/KF.h \
    ../Localisation/Localisation.h \
    ../Infrastructure/FieldObjects/WorldModelShareObject.h \
    ../Infrastructure/GameInformation/GameInformation.h \
    ../Tools/Threading/Thread.h \
    ../Tools/Threading/ConditionalThread.h \
    ../Tools/Threading/PeriodicThread.h \
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
    ../Infrastructure/NUSensorsData/NUSensorsData.h \
    ../Infrastructure/NUData.h \
    ../Infrastructure/NUBlackboard.h \
    ../NUPlatform/NUPlatform.h \
    ../Tools/Math/General.h \
    ../Vision/CornerPoint.h \
    ../Kinematics/OrientationUKF.h \
    FileAccess/StreamFileReader.h \
    ../Tools/FileFormats/TimestampedData.h \
    FileAccess/ImageStreamFileReader.h \
    ../Motion/Tools/MotionScript.h \
    ../Motion/Tools/MotionCurves.h \
    ../Vision/EllipseFit.h \
    ../Vision/EllipseFitting/tnt_version.h \
    ../Vision/EllipseFitting/tnt_vec.h \
    ../Vision/EllipseFitting/tnt_subscript.h \
    ../Vision/EllipseFitting/tnt_stopwatch.h \
    ../Vision/EllipseFitting/tnt_sparse_matrix_csr.h \
    ../Vision/EllipseFitting/tnt_math_utils.h \
    ../Vision/EllipseFitting/tnt_i_refvec.h \
    ../Vision/EllipseFitting/tnt_fortran_array3d_utils.h \
    ../Vision/EllipseFitting/tnt_fortran_array3d.h \
    ../Vision/EllipseFitting/tnt_fortran_array2d_utils.h \
    ../Vision/EllipseFitting/tnt_fortran_array2d.h \
    ../Vision/EllipseFitting/tnt_fortran_array1d_utils.h \
    ../Vision/EllipseFitting/tnt_fortran_array1d.h \
    ../Vision/EllipseFitting/tnt_cmat.h \
    ../Vision/EllipseFitting/tnt_array3d_utils.h \
    ../Vision/EllipseFitting/tnt_array3d.h \
    ../Vision/EllipseFitting/tnt_array2d_utils.h \
    ../Vision/EllipseFitting/tnt_array2d.h \
    ../Vision/EllipseFitting/tnt_array1d_utils.h \
    ../Vision/EllipseFitting/tnt_array1d.h \
    ../Vision/EllipseFitting/tnt.h \
    ../Vision/EllipseFitting/jama_svd.h \
    ../Vision/EllipseFitting/jama_qr.h \
    ../Vision/EllipseFitting/jama_lu.h \
    ../Vision/EllipseFitting/jama_eig.h \
    ../Vision/EllipseFitting/jama_cholesky.h \
    ../Localisation/odometryMotionModel.h \
    ../Localisation/probabilityUtils.h \
    FileAccess/SplitStreamFileFormatReader.h \
    SensorDisplayWidget.h \
    locwmstreamwidget.h \
    ../Vision/EllipseFitting/FittingCalculations.h \
    ../Tools/Math/Rectangle.h \
    ../NUPlatform/NUCamera.h \
    ../Vision/fitellipsethroughcircle.h \
    ../Localisation/LocWmFrame.h \
    FileAccess/IndexedFileReader.h \
    LUTGlDisplay.h \
    ../Vision/SplitAndMerge/SAM.h \
    ../NUPlatform/NUSensors/EndEffectorTouch.h
SOURCES += mainwindow.cpp \
    main.cpp \
    connectionwidget.cpp \
    classificationwidget.cpp \
    ../Tools/FileFormats/NUbotImage.cpp \
    ../Vision/Vision.cpp \
    ../Tools/FileFormats/LUTTools.cpp \
    virtualnubot.cpp \
    ../Infrastructure/NUImage/BresenhamLine.cpp \
    ../Tools/Math/Line.cpp \
    ../Kinematics/Horizon.cpp \
    openglmanager.cpp \
    GLDisplay.cpp \
    ../Infrastructure/NUImage/NUImage.cpp \
    ../Infrastructure/NUImage/ClassifiedImage.cpp \
    ../Vision/ClassifiedSection.cpp \
    ../Vision/ScanLine.cpp \
    ../Vision/TransitionSegment.cpp \
    ../Vision/GoalDetection.cpp \
    LayerSelectionWidget.cpp \
    WalkParameterWidget.cpp \
    KickWidget.cpp \
    ../Motion/Walks/WalkParameters.cpp \
    ../Tools/Optimisation/Parameter.cpp \
    ../Motion/Tools/MotionFileTools.cpp \
    ../NUPlatform/NUIO.cpp \
    ../NUPlatform/NUIO/*.cpp \
    NUviewIO/NUviewIO.cpp \
    ../Infrastructure/NUData.cpp \
    ../Infrastructure/NUBlackboard.cpp \
    ../NUPlatform/NUPlatform.cpp \
    ../NUPlatform/NUSensors.cpp \
    ../Infrastructure/NUSensorsData/*.cpp \
    ../NUPlatform/NUActionators.cpp \
    ../NUPlatform/NUActionators/*.cpp \
    ../Infrastructure/NUActionatorsData/*.cpp \
    ../Infrastructure/TeamInformation/TeamInformation.cpp \
    ../Infrastructure/Jobs/*.cpp \
    ../Infrastructure/Jobs/CameraJobs/*.cpp \
    ../Infrastructure/Jobs/VisionJobs/*.cpp \
    ../Infrastructure/Jobs/MotionJobs/*.cpp \
    locWmGlDisplay.cpp \
    ../Vision/ObjectCandidate.cpp \
    ../Vision/LineDetection.cpp \
    ../Tools/Math/LSFittedLine.cpp \
    ../Infrastructure/FieldObjects/StationaryObject.cpp \
    ../Infrastructure/FieldObjects/Self.cpp \
    ../Infrastructure/FieldObjects/Object.cpp \
    ../Infrastructure/FieldObjects/MobileObject.cpp \
    ../Infrastructure/FieldObjects/AmbiguousObject.cpp \
    ../Infrastructure/FieldObjects/FieldObjects.cpp \
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
    ../Localisation/KF.cpp \
    ../Localisation/Localisation.cpp \
    ../Infrastructure/FieldObjects/WorldModelShareObject.cpp \
    ../Infrastructure/GameInformation/GameInformation.cpp \
    ../Tools/Threading/Thread.cpp \
    ../Tools/Threading/ConditionalThread.cpp \
    ../Tools/Threading/PeriodicThread.cpp \
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
    ../Kinematics/OrientationUKF.cpp \
    ../Motion/Tools/MotionScript.cpp \
    ../Motion/Tools/MotionCurves.cpp \
    ../Vision/EllipseFit.cpp \
    ../Localisation/odometryMotionModel.cpp \
    ../Localisation/probabilityUtils.cpp \
    FileAccess/SplitStreamFileFormatReader.cpp \
    SensorDisplayWidget.cpp \
    locwmstreamwidget.cpp \
    ../Vision/EllipseFitting/FittingCalculations.cpp \
    ../Tools/Math/Rectangle.cpp \
    ../NUPlatform/NUCamera.cpp \
    ../Vision/fitellipsethroughcircle.cpp \
    ../Localisation/LocWmFrame.cpp \
    FileAccess/IndexedFileReader.cpp \
    LUTGlDisplay.cpp \
    ../Vision/SplitAndMerge/SAM.cpp \
    ../NUPlatform/NUSensors/EndEffectorTouch.cpp
RESOURCES = textures.qrc
RESOURCES += icons.qrc
