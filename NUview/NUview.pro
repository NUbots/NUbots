QT += network \
    opengl
macx { 
    # Mac Specific Includes
    QMAKE_LFLAGS += -F/System/Library/Frameworks/CoreFoundation.framework/
    LIBS += -framework \
        CoreFoundation \
        -L \
        /usr/lib/libz.dylib
}
win32 {
INCLUDEPATH += 'C:/Program Files/boost/'
}
INCLUDEPATH += ../
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
    LayerSelectionWidget.h \
    locWmGlDisplay.h \
    ../Vision/RobotCandidate.h \
    ../Vision/ObjectCandidate.h
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
    LayerSelectionWidget.cpp \
    locWmGlDisplay.cpp \
    ../Vision/RobotCandidate.cpp \
    ../Vision/ObjectCandidate.cpp
RESOURCES = textures.qrc
