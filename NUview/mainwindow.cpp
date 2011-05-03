#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "LayerSelectionWidget.h"
#include "WalkParameterWidget.h"
#include "KickWidget.h"
#include "camerasettingswidget.h"
#include <QtGui>
#include <QMdiArea>
#include <QStatusBar>
#include <stdio.h>
#include <QDebug>
#include <QWidget>
#include <iostream>
#include <QTabWidget>
#include <QImage>
#include <typeinfo>

#include "NUPlatform/NUPlatform.h"
#include "Infrastructure/NUBlackboard.h"
#include "Infrastructure/NUSensorsData/NUSensorsData.h"
#include "Infrastructure/NUActionatorsData/NUActionatorsData.h"
#include "Infrastructure/FieldObjects/FieldObjects.h"
#include "Infrastructure/Jobs/JobList.h"
#include "Infrastructure/GameInformation/GameInformation.h"
#include "Infrastructure/TeamInformation/TeamInformation.h"
#include "NUviewIO/NUviewIO.h"

#include "frameInformationWidget.h"
#include "bonjour/robotSelectDialog.h"
#include "bonjour/bonjourserviceresolver.h"

using namespace std;
ofstream debug;
ofstream errorlog;

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), bonjourResolver(0)
{
    qDebug() << "NUview is starting in: MainWindow.cpp";
    debug.open("debug.log");
    errorlog.open("error.log");

    m_platform = new NUPlatform();                      // you could make the arguement that NUView should have its own platform, for now we just use a 'blank' one
    m_blackboard = new NUBlackboard();
    m_blackboard->add(new NUSensorsData());
    m_blackboard->add(new NUActionatorsData());
    m_blackboard->add(new FieldObjects());
    m_blackboard->add(new JobList());
    m_blackboard->add(new GameInformation(0, 0));
    m_blackboard->add(new TeamInformation(0, 0));
    
    m_nuview_io = new NUviewIO();

    // create mdi workspace
    mdiArea = new QMdiArea(this);
    mdiArea->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    mdiArea->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);

    createActions();
    createMenus();
    createContextMenu();
    createToolBars();
    createStatusBar();

    sensorDisplay = new SensorDisplayWidget(this);
    QDockWidget* sensorDock = new QDockWidget("Sensor Values");
    sensorDock->setObjectName("Sensor Values");
    sensorDock->setWidget(sensorDisplay);
    sensorDock->setShown(false);
    addDockWidget(Qt::RightDockWidgetArea,sensorDock);

    // Add localisation widget
    localisation = new LocalisationWidget(this);
    addDockWidget(Qt::BottomDockWidgetArea,localisation);
    localisation->setVisible(false);

    // Add Vision Widgets to Tab then Dock them on Screen
    visionTabs = new QTabWidget(this);
    layerSelection = new LayerSelectionWidget(mdiArea,this);
    visionTabs->addTab(layerSelection,layerSelection->objectName());
    classification = new ClassificationWidget(this);
    visionTabs->addTab(classification,classification->objectName());
    visionTabDock = new QDockWidget("Vision");
    visionTabDock->setWidget(visionTabs);
    visionTabDock->setObjectName(tr("visionTab"));
    addDockWidget(Qt::RightDockWidgetArea, visionTabDock);
    
    // Add Network widgets to Tabs then dock them on Screen
    networkTabs = new QTabWidget(this);
    connection = new ConnectionWidget(this);
    networkTabs->addTab(connection, connection->objectName());
    walkParameter = new WalkParameterWidget(mdiArea, this);
    kick = new KickWidget(mdiArea, this);
    networkTabs->addTab(walkParameter, walkParameter->objectName());
    networkTabs->addTab(kick, kick->objectName());
    VisionStreamer = new visionStreamWidget(mdiArea, this);
    LocWmStreamer = new locwmStreamWidget(mdiArea, this);
    networkTabs->addTab(VisionStreamer, VisionStreamer->objectName());
    networkTabs->addTab(LocWmStreamer, LocWmStreamer->objectName());
    cameraSetting = new cameraSettingsWidget(mdiArea, this);
    networkTabs->addTab(cameraSetting, cameraSetting->objectName());

    //networkTabs->addTab(kick, kick->objectName());
    networkTabDock = new QDockWidget("Network");
    networkTabDock->setWidget(networkTabs);
    networkTabDock->setObjectName(tr("networkTab"));
    addDockWidget(Qt::RightDockWidgetArea, networkTabDock);

    frameInfo = new frameInformationWidget(this);
    QDockWidget* temp = new QDockWidget(this);
    temp->setWidget(frameInfo);
    temp->setObjectName("Frame Information Dock");
    temp->setWindowTitle(frameInfo->windowTitle());
    addDockWidget(Qt::RightDockWidgetArea,temp);

    createConnections();
    setCentralWidget(mdiArea);
    qDebug() << "Main Window Starting";
    setWindowTitle(QString("NUview"));
    glManager.clearAllDisplays();
    qDebug() << "Display Cleared";
    readSettings();
    qDebug() << "Main Window Started";

    //glManager.writeWMBallToDisplay(100,100,30,GLDisplay::CalGrid);
    glManager.writeCalGridToDisplay(GLDisplay::CalGrid);
    //
    //glManager.writeCalGridToDisplay(GLDisplay::CalGrid);
    //
}

MainWindow::~MainWindow()
{
// Delete widgets and displays
    delete statusBar;
    delete classification;
    delete connection;
    delete localisation;
    delete layerSelection;
    delete walkParameter;
    delete kick;
    delete mdiArea;
    delete visionTabs;
    delete networkTabs;
    delete frameInfo;

// Delete Actions
    delete openAction;
    delete copyAction;
    delete undoAction;
    delete exitAction;
    delete firstFrameAction;
    delete previousFrameAction;
    delete selectFrameAction;
    delete nextFrameAction;
    delete lastFrameAction;
    delete cascadeAction;
    delete tileAction;
    delete nativeAspectAction;
    delete newVisionDisplayAction;
    delete newLocWMDisplayAction;
    delete doBonjourTestAction;
    
    delete m_nuview_io;
    delete m_blackboard;
    delete m_platform;
    return;
}

void MainWindow::createActions()
{
    // Open Action
    openAction = new QAction(QIcon(":/icons/open.png"),tr("&Open..."), this);
    openAction->setShortcut(QKeySequence::Open);
    openAction->setStatusTip(tr("Open a new file"));
    connect(openAction, SIGNAL(triggered()), this, SLOT(openLog()));

    // Copy Action
    copyAction = new QAction(QIcon(":/icons/copy.png"),tr("&Copy"), this);
    copyAction->setShortcut(QKeySequence::Copy);
    copyAction->setStatusTip(tr("Copy"));
    connect(copyAction, SIGNAL(triggered()), this, SLOT(copy()));

    // Undo action
    undoAction = new QAction(QIcon(":/icons/undo.png"),tr("&Undo"), this);
    undoAction->setShortcut(QKeySequence::Undo);
    undoAction->setStatusTip(tr("Undo"));
    connect(undoAction, SIGNAL(triggered()), &virtualRobot, SLOT(UndoLUT()));

    // LUT Action
    LUT_Action = new QAction(QIcon(":/icons/open.png"),tr("&Open LUT..."), this);
    LUT_Action->setShortcut(tr("Ctrl+L"));
    LUT_Action->setStatusTip(tr("Open a LUT file"));
    connect(LUT_Action, SIGNAL(triggered()), this, SLOT(openLUT()));

    // Exit Action
    exitAction = new QAction(tr("E&xit"), this);
    exitAction->setShortcut(tr("Ctrl+Q"));
    exitAction->setStatusTip(tr("Exit the application"));
    exitAction->setIcon(this->style()->standardIcon(QStyle::SP_DialogCloseButton));
    connect(exitAction, SIGNAL(triggered()), qApp, SLOT(closeAllWindows()));

    // First Frame
    firstFrameAction = new QAction(tr("&First Frame"), this);
    firstFrameAction->setShortcut(QKeySequence::MoveToStartOfLine);
    firstFrameAction->setStatusTip(tr("Go to the first frame of the replay"));
    firstFrameAction->setIcon(QIcon(QString(":/icons/first.png")));
    firstFrameAction->setEnabled(false);
    connect(firstFrameAction, SIGNAL(triggered()), &LogReader, SLOT(firstFrame()));

    // Previous Frame
    previousFrameAction = new QAction(tr("&Previous Frame"), this);
    previousFrameAction->setShortcut(QKeySequence::MoveToPreviousChar);
    previousFrameAction->setStatusTip(tr("Select the previous frame"));
    previousFrameAction->setIcon(QIcon(QString(":/icons/previous.png")));
    connect(previousFrameAction, SIGNAL(triggered()), &LogReader, SLOT(previousFrame()));
    previousFrameAction->setEnabled(false);

    // Select Frame
    selectFrameAction = new QAction(tr("&Select Frame..."), this);
    selectFrameAction->setShortcut(tr("Ctrl+G"));
    selectFrameAction->setStatusTip(tr("Select frame number to go to"));
    selectFrameAction->setIcon(QIcon(QString(":/icons/select.png")));
    connect(selectFrameAction, SIGNAL(triggered()), this, SLOT(selectFrame()));
    selectFrameAction->setEnabled(false);

    // Next Frame
    nextFrameAction = new QAction(tr("&Next Frame"), this);
    nextFrameAction->setShortcut(QKeySequence::MoveToNextChar);
    nextFrameAction->setStatusTip(tr("Select next frame"));
    nextFrameAction->setIcon(QIcon(QString(":/icons/next.png")));
    connect(nextFrameAction, SIGNAL(triggered()), &LogReader, SLOT(nextFrame()));
    nextFrameAction->setEnabled(false);

    // Last Frame
    lastFrameAction = new QAction(tr("&Last Frame"), this);
    lastFrameAction->setShortcut(QKeySequence::MoveToEndOfLine);
    lastFrameAction->setStatusTip(tr("Select last frame"));
    lastFrameAction->setIcon(QIcon(QString(":/icons/last.png")));
    connect(lastFrameAction, SIGNAL(triggered()), &LogReader, SLOT(lastFrame()));
    lastFrameAction->setEnabled(false);

    // Cascade windows
    cascadeAction = new QAction(tr("&Cascade Window"), this);
    cascadeAction->setStatusTip(tr("Cascade windows in Main Area"));
    connect(cascadeAction, SIGNAL(triggered()), mdiArea, SLOT(cascadeSubWindows()));
    // Tile windows
    tileAction = new QAction(tr("&Tile Window"), this);
    tileAction->setStatusTip(tr("Tiles windows in Main Area"));
    connect(tileAction, SIGNAL(triggered()), mdiArea, SLOT(tileSubWindows()));

    nativeAspectAction = new QAction(tr("&Native Aspect"), this);
    nativeAspectAction->setStatusTip(tr("Resize display to its native aspect ratio."));
    connect(nativeAspectAction, SIGNAL(triggered()), this, SLOT(shrinkToNativeAspectRatio()));

    // New vision display window
    newVisionDisplayAction = new QAction(tr("&New display"), this);
    newVisionDisplayAction->setStatusTip(tr("Create a new vision display window."));
    connect(newVisionDisplayAction, SIGNAL(triggered()), this, SLOT(createGLDisplay()));

    // New LocWM display window
    newLocWMDisplayAction = new QAction(tr("&New display"), this);
    newLocWMDisplayAction->setStatusTip(tr("Create a new Localisation and World Model display window."));
    connect(newLocWMDisplayAction, SIGNAL(triggered()), this, SLOT(createLocWmGlDisplay()));

    // New LUT Display Window
    newLUTDisplayAction = new QAction(tr("&New display"), this);
    newLUTDisplayAction->setStatusTip(tr("Create a new Look up table display window."));
    connect(newLUTDisplayAction, SIGNAL(triggered()), this, SLOT(createLUTGlDisplay()));

    doBonjourTestAction = new QAction(tr("&Bonjour Test..."), this);
    doBonjourTestAction->setStatusTip(tr("Test something."));
    connect(doBonjourTestAction, SIGNAL(triggered()), this, SLOT(BonjourTest()));
}

void MainWindow::createMenus()
{
    // File Menu
    fileMenu = menuBar()->addMenu(tr("&File"));
    fileMenu->addAction(openAction);
    fileMenu->addAction(LUT_Action);
    fileMenu->addSeparator();
    fileMenu->addAction(exitAction);

    // Edit Menu
    editMenu = menuBar()->addMenu(tr("&Edit"));
    editMenu->addAction(undoAction);
    editMenu->addSeparator();
    editMenu->addAction(copyAction);

    // Navigation Menu
    navigationMenu = menuBar()->addMenu(tr("&Navigation"));
    navigationMenu->addAction(firstFrameAction);
    navigationMenu->addAction(previousFrameAction);
    navigationMenu->addAction(selectFrameAction);
    navigationMenu->addAction(nextFrameAction);
    navigationMenu->addAction(lastFrameAction);

    testMenu = menuBar()->addMenu(tr("&Testing"));
    testMenu->addAction(doBonjourTestAction);

    // Window Menu
    windowMenu = menuBar()->addMenu(tr("&Window"));

    visionWindowMenu = windowMenu->addMenu(tr("&Vision"));
    visionWindowMenu->addAction(newVisionDisplayAction);

    localisationWindowMenu = windowMenu->addMenu(tr("&Localisation"));
    localisationWindowMenu->addAction(newLocWMDisplayAction);

    LUTWindowMenu = windowMenu->addMenu(tr("&Look Up Table"));
    LUTWindowMenu->addAction(newLUTDisplayAction);

    networkWindowMenu = windowMenu->addMenu(tr("&Network"));
    windowMenu->addSeparator();
    windowMenu->addAction(cascadeAction);
    windowMenu->addAction(tileAction);
    windowMenu->addAction(nativeAspectAction);
}

void MainWindow::createContextMenu()
{
}

void MainWindow::createToolBars()
{
    // File toolbar
    fileToolBar = addToolBar(tr("&File"));
    fileToolBar->addAction(openAction);
    fileToolBar->addAction(exitAction);
    fileToolBar->setObjectName(tr("fileToolbar"));

    // Navigation Toolbar
    navigationToolbar = addToolBar(tr("N&avigation"));
    navigationToolbar->addAction(firstFrameAction);
    navigationToolbar->addAction(previousFrameAction);
    navigationToolbar->addAction(selectFrameAction);
    navigationToolbar->addAction(nextFrameAction);
    navigationToolbar->addAction(lastFrameAction);
    navigationToolbar->setObjectName(tr("navigationToolbar"));
    //windowDisplayToolbar = addToolBar(tr("&Display"));
}

void MainWindow::createStatusBar()
{
        statusBar = new QStatusBar;
        this->setStatusBar(statusBar);
        this->statusBar->showMessage("Welcome to NUview!",10000);
}

void MainWindow::createConnections()
{
    qDebug() <<"Start Connecting Widgets";
    // Connect to log file reader
    connect(&LogReader,SIGNAL(sensorDataChanged(NUSensorsData*)),sensorDisplay, SLOT(SetSensorData(NUSensorsData*)));
    connect(&LogReader,SIGNAL(sensorDataChanged(NUSensorsData*)),&virtualRobot, SLOT(setSensorData(NUSensorsData*)));
    connect(&LogReader,SIGNAL(frameChanged(int,int)),this, SLOT(imageFrameChanged(int,int)));

    connect(&LogReader,SIGNAL(rawImageChanged(const NUImage*)),&glManager, SLOT(setRawImage(const NUImage*)));
    connect(&LogReader,SIGNAL(rawImageChanged(const NUImage*)), frameInfo, SLOT(setRawImage(const NUImage*)));

    connect(&LogReader,SIGNAL(fileOpened(QString)),this, SLOT(filenameChanged(QString)));
    connect(&LogReader,SIGNAL(fileOpened(QString)),frameInfo, SLOT(setFrameSource(QString)));
    connect(&LogReader,SIGNAL(fileClosed()),this, SLOT(fileClosed()));

    connect(&LogReader,SIGNAL(cameraChanged(int)),&virtualRobot, SLOT(setCamera(int)));
    connect(&LogReader,SIGNAL(rawImageChanged(const NUImage*)),&virtualRobot, SLOT(setRawImage(const NUImage*)));
    connect(&LogReader,SIGNAL(sensorDataChanged(const float*, const float*, const float*)),&virtualRobot, SLOT(setSensorData(const float*, const float*, const float*)));
    connect(&LogReader,SIGNAL(frameChanged(int,int)),&virtualRobot, SLOT(processVisionFrame()));

    connect(&LogReader,SIGNAL(rawImageChanged(const NUImage*)), this, SLOT(updateSelection()));

    connect(VisionStreamer,SIGNAL(rawImageChanged(const NUImage*)),&glManager, SLOT(setRawImage(const NUImage*)));
    connect(VisionStreamer,SIGNAL(rawImageChanged(const NUImage*)), this, SLOT(updateSelection()));
    connect(VisionStreamer,SIGNAL(rawImageChanged(const NUImage*)),&virtualRobot, SLOT(setRawImage(const NUImage*)));
    connect(VisionStreamer,SIGNAL(rawImageChanged(const NUImage*)),&virtualRobot, SLOT(processVisionFrame()));
    connect(VisionStreamer,SIGNAL(sensorsDataChanged(NUSensorsData*)),&virtualRobot, SLOT(setSensorData(NUSensorsData*)));
    connect(VisionStreamer,SIGNAL(sensorsDataChanged(NUSensorsData*)),sensorDisplay, SLOT(SetSensorData(NUSensorsData*)));
    // Setup navigation control enabling/disabling
    connect(&LogReader,SIGNAL(firstFrameAvailable(bool)),firstFrameAction, SLOT(setEnabled(bool)));
    connect(&LogReader,SIGNAL(nextFrameAvailable(bool)),nextFrameAction, SLOT(setEnabled(bool)));
    connect(&LogReader,SIGNAL(previousFrameAvailable(bool)),previousFrameAction, SLOT(setEnabled(bool)));
    connect(&LogReader,SIGNAL(lastFrameAvailable(bool)),lastFrameAction, SLOT(setEnabled(bool)));
    connect(&LogReader,SIGNAL(setFrameAvailable(bool)),selectFrameAction, SLOT(setEnabled(bool)));

    // Connect the virtual robot to the opengl manager.
    connect(&virtualRobot,SIGNAL(imageDisplayChanged(const NUImage*,GLDisplay::display)),&glManager, SLOT(writeNUImageToDisplay(const NUImage*,GLDisplay::display)));
    connect(&virtualRobot,SIGNAL(lineDisplayChanged(Line*, GLDisplay::display)),&glManager, SLOT(writeLineToDisplay(Line*, GLDisplay::display)));
    connect(&virtualRobot,SIGNAL(classifiedDisplayChanged(ClassifiedImage*, GLDisplay::display)),&glManager, SLOT(writeClassImageToDisplay(ClassifiedImage*, GLDisplay::display)));
    connect(&virtualRobot,SIGNAL(pointsDisplayChanged(std::vector< Vector2<int> >, GLDisplay::display)),&glManager, SLOT(writePointsToDisplay(std::vector< Vector2<int> >, GLDisplay::display)));
    connect(&virtualRobot,SIGNAL(transitionSegmentsDisplayChanged(std::vector< TransitionSegment >, GLDisplay::display)),&glManager, SLOT(writeTransitionSegmentsToDisplay(std::vector< TransitionSegment >, GLDisplay::display)));
    //connect(&virtualRobot,SIGNAL(robotCandidatesDisplayChanged(std::vector< RobotCandidate >, GLDisplay::display)),&glManager, SLOT(writeRobotCandidatesToDisplay(std::vector< RobotCandidate >, GLDisplay::display)));
    connect(&virtualRobot,SIGNAL(lineDetectionDisplayChanged(std::vector< LSFittedLine >, GLDisplay::display)),&glManager, SLOT(writeFieldLinesToDisplay(std::vector< LSFittedLine >, GLDisplay::display)));
    connect(&virtualRobot,SIGNAL(candidatesDisplayChanged(std::vector< ObjectCandidate >, GLDisplay::display)),&glManager, SLOT(writeCandidatesToDisplay(std::vector< ObjectCandidate >, GLDisplay::display)));
    connect(&virtualRobot,SIGNAL(fieldObjectsDisplayChanged(FieldObjects*,GLDisplay::display)),&glManager,SLOT(writeFieldObjectsToDisplay(FieldObjects*,GLDisplay::display)));
    connect(&virtualRobot,SIGNAL(linePointsDisplayChanged(std::vector< LinePoint >,GLDisplay::display)),&glManager,SLOT(writeLinesPointsToDisplay(std::vector< LinePoint >,GLDisplay::display)));
    connect(&virtualRobot,SIGNAL(cornerPointsDisplayChanged(std::vector< CornerPoint >,GLDisplay::display)),&glManager,SLOT(writeCornersToDisplay(std::vector< CornerPoint >,GLDisplay::display)));
    connect(&virtualRobot,SIGNAL(edgeFilterChanged(QImage, GLDisplay::display)),&glManager,SLOT(stub(QImage, GLDisplay::display)));
    connect(&virtualRobot,SIGNAL(fftChanged(QImage, GLDisplay::display)),&glManager,SLOT(stub(QImage, GLDisplay::display)));

    // Connect Statistics for Classification
    connect(&virtualRobot,SIGNAL(updateStatistics(float *)),classification,SLOT(updateStatistics(float *)));

    // Connect the virtual robot to the incoming packets.
    connect(connection, SIGNAL(PacketReady(QByteArray*)), &virtualRobot, SLOT(ProcessPacket(QByteArray*)));
    connect(classification,SIGNAL(selectionChanged()), this, SLOT(updateSelection()));
    connect(classification,SIGNAL(openLookupTableFile(QString)), &virtualRobot, SLOT(loadLookupTableFile(QString)));
    connect(classification,SIGNAL(saveLookupTableFile(QString)), &virtualRobot, SLOT(saveLookupTableFile(QString)));
    connect(classification,SIGNAL(displayStatusBarMessage(QString,int)), statusBar, SLOT(showMessage(QString,int)));

    connect(classification,SIGNAL(autoSoftColourChanged(bool)),&virtualRobot, SLOT(setAutoSoftColour(bool)));

    // Connect the virtual robot to the localisation widget and the localisation widget to the opengl manager
    //connect(&virtualRobot,SIGNAL(imageDisplayChanged(const double*,bool,const double*)),localisation, SLOT(frameChange(const double*,bool,const double*)));
    connect(&LogReader,SIGNAL(cameraChanged(int)),localisation, SLOT(setCamera(int)));
    connect(&LogReader,SIGNAL(sensorDataChanged(const float*, const float*, const float*)),localisation, SLOT(setSensorData(const float*, const float*, const float*)));
    connect(localisation,SIGNAL(updateLocalisationLine(WMLine*,int,GLDisplay::display)),&glManager,SLOT(writeWMLineToDisplay(WMLine*,int,GLDisplay::display)));
    connect(localisation,SIGNAL(updateLocalisationBall(float, float, float,GLDisplay::display)),&glManager,SLOT(writeWMBallToDisplay(float, float, float,GLDisplay::display)));
    connect(localisation,SIGNAL(removeLocalisationLine(GLDisplay::display)),&glManager,SLOT(clearDisplay(GLDisplay::display)));
    qDebug() <<"Finnished Connecting Widgets";
}

void MainWindow::openLog()
{

    QString fileName = QFileDialog::getOpenFileName(this,
                            tr("Open Replay File"), ".",
                            tr("All NUbot Image Files(*.nul;*.nif;*.nurf;*.strm);;NUbot Log Files (*.nul);;NUbot Image Files (*.nif);;NUbot Replay Files (*.nurf);;Stream File(*.strm);;All Files(*.*)"));
    openLog(fileName);

}

void MainWindow::openLog(const QString& fileName)
{
    if (!fileName.isEmpty()){
        LogReader.openFile(fileName);
        LogReader.firstFrame();
    }
    /*
    if (!fileName.isEmpty()){
        this->fileName = fileName;
        setWindowTitle(QString("NUview - ") + fileName);
        glManager.clearAllDisplays();
        totalFrameNumber = virtualRobot.openFile(fileName);
        QString message = "Opening File: ";
        message.append(fileName);
        qDebug() << message;
        this->statusBar->showMessage(message,10000);
        firstFrame();
        if(virtualRobot.fileType == QString("nul"))
        {
            firstFrameAction->setEnabled(true);
            previousFrameAction->setEnabled(false);
            selectFrameAction->setEnabled(false);
            nextFrameAction->setEnabled(true);
            lastFrameAction->setEnabled(false);
        }
        else if(virtualRobot.fileType == QString("nif"))
        {
            firstFrameAction->setEnabled(true);
            previousFrameAction->setEnabled(true);
            selectFrameAction->setEnabled(true);
            nextFrameAction->setEnabled(true);
            lastFrameAction->setEnabled(true);
        }
        else
        {
            firstFrameAction->setEnabled(false);
            previousFrameAction->setEnabled(false);
            selectFrameAction->setEnabled(false);
            nextFrameAction->setEnabled(false);
            lastFrameAction->setEnabled(false);
        }
    }
    */
}

void MainWindow::copy()
{
    if(QMdiSubWindow *activeSubWindow = mdiArea->activeSubWindow())
    {
        QWidget* widget = activeSubWindow->widget();
        if(typeid(*widget) == typeid(GLDisplay))
        {
            GLDisplay* currWindow = qobject_cast<GLDisplay *>(widget);
            currWindow->snapshotToClipboard();
        }
        else if(typeid(*widget) == typeid(locWmGlDisplay))
        {
            locWmGlDisplay* currWindow = qobject_cast<locWmGlDisplay *>(widget);
            currWindow->snapshotToClipboard();
        }
    }
}

void MainWindow::shrinkToNativeAspectRatio()
{
    if(QMdiSubWindow *activeSubWindow = mdiArea->activeSubWindow())
    {
        QSize windowSize = activeSubWindow->size();
        QSize sourceSize;
        bool validWidget = false;
        QWidget* widget = activeSubWindow->widget();
        if(typeid(*widget) == typeid(GLDisplay))
        {
            GLDisplay* currWindow = qobject_cast<GLDisplay *>(widget);
            validWidget = true;
            sourceSize = currWindow->imageSize();
        }

        if(validWidget)
        {
            sourceSize.scale(windowSize,Qt::KeepAspectRatio);
            activeSubWindow->resize(sourceSize);
        }
    }
}

void MainWindow::BonjourTest()
{
    robotSelectDialog test(this, "_nuview._tcp");
    if(test.exec())
    {
        BonjourRecord bonjourHost = test.getBonjourHost();
        if (!bonjourResolver)
        {
                bonjourResolver = new BonjourServiceResolver(this);
                connect(bonjourResolver, SIGNAL(bonjourRecordResolved(const QHostInfo &, int)),
                        this, SLOT(PrintConnectionInfo(const QHostInfo &, int)));
        }
        bonjourResolver->resolveBonjourRecord(bonjourHost);
    }
    else
    {
        qDebug() << "Cancelled" << endl;
    }
}

void MainWindow::PrintConnectionInfo(const QHostInfo &hostInfo, int port)
{
    const QList<QHostAddress> &addresses = hostInfo.addresses();

    if (hostInfo.error() != QHostInfo::NoError) {
        qWarning("Lookup failed: %s", hostInfo.errorString().toAscii().constData());
        return;
    }

    if (!addresses.isEmpty())
    {
        QHostAddress address = addresses.first();
        qDebug() << "Connect: " << address.toString() << " Port: " << port << endl;
    }
}

void MainWindow::closeEvent(QCloseEvent *event)
{
    //mdiArea->closeAllSubWindows();
    //if (mdiArea->currentSubWindow()) {
    //    event->ignore();
    //} else {
        writeSettings();
        event->accept();
    //}
}


void MainWindow::readSettings()
{
    QSettings settings("NUbots", "NUview");
    qDebug() <<"Start Reading Settings";
    // Restore the main window.
    settings.beginGroup("mainWindow");
    restoreGeometry(settings.value("geometry").toByteArray());  // Set previous position/size
    restoreState(settings.value("state").toByteArray());    // restore the other main widgets layout.
//    openFile(settings.value("logFile").toString());
    settings.endGroup();

    int numWindows = settings.beginReadArray("midWindows"); // Get midi windows settings
    for (int i = 0; i < numWindows; ++i) {
        settings.setArrayIndex(i);      // set to element
        QString windowType = settings.value("type").toString(); // Get the window type
        if(windowType == "unknown")
        {
            continue;
        }
        else if(windowType == "GLDisplay")
        {
            QMdiSubWindow* tempGl = createGLDisplay();
            tempGl->restoreGeometry(settings.value("geometry").toByteArray());
            GLDisplay *GLDisp = qobject_cast<GLDisplay *>(tempGl->widget());
            GLDisp->restoreState(settings.value("state").toByteArray());
        }
        else if(windowType == "locWmGlDisplay")
        {
            QMdiSubWindow* tempLocwm = createLocWmGlDisplay();
            tempLocwm->restoreGeometry(settings.value("geometry").toByteArray());
            locWmGlDisplay *lwmDisp = qobject_cast<locWmGlDisplay *>(tempLocwm->widget());
            lwmDisp->restoreState(settings.value("state").toByteArray());
        }
        else if(windowType == "LUTGlDisplay")
        {
            QMdiSubWindow* tempLUTDisp = createLUTGlDisplay();
            tempLUTDisp->restoreGeometry(settings.value("geometry").toByteArray());
            LUTGlDisplay *lwmDisp = qobject_cast<LUTGlDisplay *>(tempLUTDisp->widget());
            lwmDisp->restoreState(settings.value("state").toByteArray());
        }
    }
}

void MainWindow::writeSettings()
{
    QSettings settings("NUbots", "NUview");

    // Save the main window setting
    settings.beginGroup("mainWindow");
    settings.setValue("geometry", saveGeometry()); // Save the position and size.
    settings.setValue("state", saveState()); // Save the main widget layouts.
//    settings.setValue("logFile",fileName);
    settings.endGroup();

    // Save the individual mdi views
    QList<QMdiSubWindow *> mdiWindows = mdiArea->subWindowList(); // Get the windows.
    int numWindows = mdiWindows.count();    // Find how many there are.
    settings.beginWriteArray("midWindows"); // begin array entry
    for (int i = 0; i < numWindows; ++i) {
        settings.setArrayIndex(i);      // set to element
        QString windowType = getMdiWindowType(mdiWindows[i]->widget()); // Get the window type
        settings.setValue("type", windowType); // Save display type
        settings.setValue("geometry", mdiWindows[i]->saveGeometry()); // Save size/position
        if(windowType == "GLDisplay")
        {
            GLDisplay* glDisp = qobject_cast<GLDisplay *> (mdiWindows[i]->widget());
            settings.setValue("state", glDisp->saveState()); // Save size/position
        }
        else if(windowType == "locWmGlDisplay")
        {
            locWmGlDisplay* lwmDisp = qobject_cast<locWmGlDisplay *> (mdiWindows[i]->widget());
            settings.setValue("state", lwmDisp->saveState()); // Save size/position
        }
        else if(windowType == "LUTGlDisplay")
        {
            LUTGlDisplay* lutDisp = qobject_cast<LUTGlDisplay *> (mdiWindows[i]->widget());
            settings.setValue("state", lutDisp->saveState()); // Save size/position
        }
    }
    settings.endArray();   // end array entry
}

QString MainWindow::getMdiWindowType(QWidget* theWidget)
{
    QString windowType;
    if(typeid(*theWidget) == typeid(GLDisplay))
    {
        windowType = QString("GLDisplay");
    }
    else if(typeid(*theWidget) == typeid(locWmGlDisplay))
    {
        windowType = QString("locWmGlDisplay");
    }
    else if(typeid(*theWidget) == typeid(LUTGlDisplay))
    {
        windowType = QString("LUTGlDisplay");
    }
    else
    {
        windowType = QString("unknown");
    }
    return windowType;
}

void MainWindow::openLUT()
{
    classification->doOpen();
}

void  MainWindow::filenameChanged(QString filename)
{
    if(!filename.isEmpty()){
        setWindowTitle(QString("NUview - ") + filename);
    }
    else
    {
        setWindowTitle(QString("NUview"));
    }
}

void  MainWindow::fileClosed()
{
    filenameChanged(QString());
}

void MainWindow::imageFrameChanged(int currFrame, int totalFrames)
{
    QString message = "Frame Loaded:  Number ";
    message.append(QString::number(currFrame));
    message.append("/");
    message.append(QString::number(totalFrames));
    this->statusBar->showMessage(message, 10000);
}

void MainWindow::selectFrame()
{


    bool ok;
    int selectedFrameNumber = QInputDialog::getInteger(this, tr("Select Frame"), tr("Enter frame to jump to:"), LogReader.currentFrame(), 1, LogReader.numFrames(), 1, &ok);
    if(ok)
    {
        LogReader.setFrame(selectedFrameNumber);
    }
    return;
}

int MainWindow::getNumMdiWindowType(const QString& windowType)
{
    QList<QMdiSubWindow *> mdiWindows = mdiArea->subWindowList(); // Get the windows.
    int count = 0;
    for (int i = 0; i < mdiWindows.count(); i++)
    {
        if(windowType == getMdiWindowType(mdiWindows[i]->widget())) // Get the window type
        {
            count++;
        }
    }
    return count;
}

QMdiSubWindow* MainWindow::createGLDisplay()
{
    GLDisplay* temp = new GLDisplay(this, &glManager);
    QMdiSubWindow* window = mdiArea->addSubWindow(temp);
    temp->show();
    // Required because openGL drawing command do not seem to work when there is no associated display.
    if(getNumMdiWindowType("GLDisplay") <= 1)
    {
        LogReader.setFrame(LogReader.currentFrame());
    }    
    return window;
}

QMdiSubWindow* MainWindow::createLocWmGlDisplay()
{
    locWmGlDisplay* temp = new locWmGlDisplay(this);
    connect(&LogReader,SIGNAL(LocalisationDataChanged(const Localisation*)),temp, SLOT(SetLocalisation(const Localisation*)));
    connect(LocWmStreamer, SIGNAL(locwmDataChanged(const Localisation*)),temp, SLOT(SetLocalisation(const Localisation*)));
    connect(LocWmStreamer, SIGNAL(fieldObjectDataChanged(const FieldObjects*)),temp, SLOT(setFieldObjects(const FieldObjects*)));
    QMdiSubWindow* window = mdiArea->addSubWindow(temp);
    temp->show();
    return window;
}

QMdiSubWindow* MainWindow::createLUTGlDisplay()
{
    LUTGlDisplay* temp = new LUTGlDisplay(this, &glManager);
    connect(&virtualRobot,SIGNAL(LUTChanged(unsigned char*)),temp,SLOT(SetLUT(unsigned char*)));
    QMdiSubWindow* window = mdiArea->addSubWindow(temp);
    temp->show();
    return window;
}

void MainWindow::updateSelection()
{
    virtualRobot.updateSelection(classification->getColourLabel(),classification->getSelectedColours());
}

void MainWindow::SelectColourAtPixel(int x, int y)
{
    if(virtualRobot.imageAvailable())
    {
        Pixel tempPixel = virtualRobot.selectRawPixel(x,y);

        QString message = "(";
        message.append(QString::number(x));
        message.append(",");
        message.append(QString::number(y));
        message.append(")");
        this->statusBar->showMessage(message, 10000);
        classification->setColour(tempPixel);
    }
}

void MainWindow::ClassifySelectedColour()
{
    virtualRobot.UpdateLUT(classification->getColourLabel(),classification->getSelectedColours());
}

void MainWindow::SelectAndClassifySelectedPixel(int x, int y)
{
    if(virtualRobot.imageAvailable())
    {
        SelectColourAtPixel(x,y);
        ClassifySelectedColour();
    }
}
