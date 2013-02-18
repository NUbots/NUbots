#include "mainwindow.h"
#include "ui_mainwindow.h"

#ifndef WIN32
    #include "ConnectionManager/ConnectionManager.h"
#endif

#include "LayerSelectionWidget.h"
#include "camerasettingswidget.h"
#include "MotionWidgets/WalkParameterWidget.h"
#include "MotionWidgets/KickWidget.h"
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
#include <QFileInfo>

//for plotting
#include "plotdisplay.h"
#include <qwt/qwt_plot_curve.h>

#include "NUPlatform/NUPlatform.h"
#include "Infrastructure/NUBlackboard.h"
#include "Infrastructure/NUSensorsData/NUSensorsData.h"
#include "Infrastructure/NUActionatorsData/NUActionatorsData.h"
#include "Infrastructure/FieldObjects/FieldObjects.h"
#include "Infrastructure/Jobs/JobList.h"
#include "Infrastructure/GameInformation/GameInformation.h"
#include "Infrastructure/TeamInformation/TeamInformation.h"
#include "NUViewIO/NUViewIO.h"

#include "frameInformationWidget.h"

#include "offlinelocalisationdialog.h"

#include "Kinematics/Kinematics.h"

#include "debug.h"

using namespace std;
ofstream debug("debug.log");
ofstream errorlog("error.log");

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent)
{
    m_previous_log_path = "";
    qDebug() << "NUView is starting in: MainWindow.cpp";
    //debug.open;
    //errorlog.open;

    //initialise a static int to count image saves
    GLDisplay::imageCount = 0;
    setColourTheme(DefaultColours);

    m_platform = new NUPlatform();                      // you could make the arguement that NUView should have its own platform, for now we just use a 'blank' one
    m_blackboard = new NUBlackboard();
    m_blackboard->add(new NUSensorsData());
    m_blackboard->add(new NUActionatorsData());
    m_blackboard->add(new FieldObjects());
    m_blackboard->add(new JobList());
    m_blackboard->add(new GameInformation(0, 0));
    m_blackboard->add(new TeamInformation(0, 0));

    LogReader = new LogFileReader();
    
    m_nuview_io = new NUViewIO();

    virtualRobot = new virtualNUbot();

#ifndef WIN32
    m_connection_manager = new ConnectionManager(this);
#endif

    // create mdi workspace
    mdiArea = new QMdiArea(this);
    mdiArea->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    mdiArea->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);

    // Sensor Widget
    sensorDisplay = new SensorDisplayWidget(this);
    addAsDockable(sensorDisplay, "Sensor Values");

    // Object Widget
    objectDisplayVision = new ObjectDisplayWidget(this);
    addAsDockable(objectDisplayVision, "Field Objects (Vision)");

    // Object Widget
    objectDisplayLog = new ObjectDisplayWidget(this);
    addAsDockable(objectDisplayLog, "Field Objects (Log)");

    // Game Info Widget
    gameInfoDisplay = new GameInformationDisplayWidget(this);
    addAsDockable(gameInfoDisplay, "Game Information");

    // Team Info Widget
    teamInfoDisplay = new TeamInformationDisplayWidget(this);
    addAsDockable(teamInfoDisplay, "Team Information");

    // Localisation info display
    locInfoDisplay = new QTextBrowser(this);
    addAsDockable(locInfoDisplay, "Localisation Information");

    // New self localisation display
    selflocInfoDisplay = new QTextBrowser(this);
    addAsDockable(selflocInfoDisplay, "Self Localisation Information");

    // Add localisation widget
    localisation = new LocalisationWidget(this);
    addDockWidget(Qt::BottomDockWidgetArea,localisation);
    localisation->setVisible(false);

    // Add Vision Widgets to Tab then Dock them on Screen
    QTabWidget* visionTabs = new QTabWidget(this);
    layerSelection = new LayerSelectionWidget(mdiArea,this);
    visionTabs->addTab(layerSelection,layerSelection->objectName());
    classification = new ClassificationWidget(this);
    visionTabs->addTab(classification,classification->objectName());
    addAsDockable(visionTabs, "Vision tools");

    // Add Plots Widget then Dock it on Screen
    plotSelection = new PlotSelectionWidget(mdiArea,this);
    addAsDockable(plotSelection, "Plot tools");
    
    // Add Network widgets to Tabs then dock them on Screen
    QTabWidget* networkTabs = new QTabWidget(this);
    connection = new ConnectionWidget(this);
    networkTabs->addTab(connection, connection->objectName());
    walkParameter = new WalkParameterWidget(this);
    kick = new KickWidget(this);
    networkTabs->addTab(walkParameter, walkParameter->objectName());
    networkTabs->addTab(kick, kick->objectName());
    VisionStreamer = new visionStreamWidget(this);
    LocWmStreamer = new locwmStreamWidget(this);
    networkTabs->addTab(VisionStreamer, VisionStreamer->objectName());
    networkTabs->addTab(LocWmStreamer, LocWmStreamer->objectName());
    //networkTabs->addTab(kick, kick->objectName());
    addAsDockable(networkTabs, "Network");


    // Add the camera settings as a seperate window, since it is so large compared to the other
    // Network windows.
    cameraSetting = new cameraSettingsWidget(this);
    addAsDockable(cameraSetting, "Camera Settings");

    frameInfo = new frameInformationWidget(this);
    addAsDockable(frameInfo, "Frame Information");

    offlinelocDialog = new OfflineLocalisationDialog(LogReader,this);
    offlinelocDialog->hide();

    createActions();
    createMenus();
    createContextMenu();
    createToolBars();
    createConnections();
    setCentralWidget(mdiArea);

    // Initialise status bar
    statusBar()->showMessage("Welcome to NUView!",10000);

    setWindowTitle(QString("NUView"));
    glManager.clearAllDisplays();
    readSettings();

    glManager.writeCalGridToDisplay(GLDisplay::CalGrid);
}


MainWindow::~MainWindow()
{
    // NOTE: QObjects with parents do not need to be deleted.
    // They are delete when their parents are.
    delete m_nuview_io;
    delete m_blackboard;
    delete m_platform;
    return;
}

void MainWindow::addAsDockable(QWidget* widget, const QString& name)
{
    QDockWidget* dockable = new QDockWidget(name);
    dockable->setObjectName(name);
    dockable->setWidget(widget);
    dockable->setShown(false);
    addDockWidget(Qt::RightDockWidgetArea,dockable);
    m_dockable_windows.push_back(dockable);
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

    // Save Image Action
    saveAction = new QAction(QIcon(":/icons/copy.png"),tr("&Save Image"), this);
    saveAction->setShortcut(QKeySequence::Save);
    saveAction->setStatusTip(tr("Save Image"));
    connect(saveAction, SIGNAL(triggered()), this, SLOT(saveViewImage()));

    // Undo action
    undoAction = new QAction(QIcon(":/icons/undo.png"),tr("&Undo"), this);
    undoAction->setShortcut(QKeySequence::Undo);
    undoAction->setStatusTip(tr("Undo"));
    connect(undoAction, SIGNAL(triggered()), virtualRobot, SLOT(UndoLUT()));

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
    connect(firstFrameAction, SIGNAL(triggered()), LogReader, SLOT(firstFrame()));

    // Previous Frame
    previousFrameAction = new QAction(tr("&Previous Frame"), this);
    previousFrameAction->setShortcut(QKeySequence::MoveToPreviousChar);
    previousFrameAction->setStatusTip(tr("Select the previous frame"));
    previousFrameAction->setIcon(QIcon(QString(":/icons/previous.png")));
    connect(previousFrameAction, SIGNAL(triggered()), LogReader, SLOT(previousFrame()));
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
    connect(nextFrameAction, SIGNAL(triggered()), LogReader, SLOT(nextFrame()));
    nextFrameAction->setEnabled(false);

    // Last Frame
    lastFrameAction = new QAction(tr("&Last Frame"), this);
    lastFrameAction->setShortcut(QKeySequence::MoveToEndOfLine);
    lastFrameAction->setStatusTip(tr("Select last frame"));
    lastFrameAction->setIcon(QIcon(QString(":/icons/last.png")));
    connect(lastFrameAction, SIGNAL(triggered()), LogReader, SLOT(lastFrame()));
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

    // New plot display window
    newPlotDisplayAction = new QAction(tr("&New display"), this);
    newPlotDisplayAction->setStatusTip(tr("Create a new plot display window."));
    connect(newPlotDisplayAction, SIGNAL(triggered()), this, SLOT(createPlotDisplay()));

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

    runOfflineLocalisatonAction = new QAction(tr("&Offline Localisation..."), this);
    runOfflineLocalisatonAction->setStatusTip(tr("Run offline localisation simulation."));
    connect(runOfflineLocalisatonAction, SIGNAL(triggered()), this, SLOT(RunOfflineLocalisation()));
}

void MainWindow::createMenus()
{
    // File Menu
    QMenu* fileMenu = menuBar()->addMenu(tr("&File"));
    fileMenu->addAction(openAction);
    fileMenu->addAction(LUT_Action);
    fileMenu->addSeparator();
    fileMenu->addAction(exitAction);

    // Edit Menu
    QMenu* editMenu = menuBar()->addMenu(tr("&Edit"));
    editMenu->addAction(undoAction);
    editMenu->addSeparator();
    editMenu->addAction(copyAction);
    editMenu->addSeparator();
    editMenu->addAction(saveAction);

    // Navigation Menu
    QMenu* navigationMenu = menuBar()->addMenu(tr("&Navigation"));
    navigationMenu->addAction(firstFrameAction);
    navigationMenu->addAction(previousFrameAction);
    navigationMenu->addAction(selectFrameAction);
    navigationMenu->addAction(nextFrameAction);
    navigationMenu->addAction(lastFrameAction);

    // Test Menu
    QMenu* testMenu = menuBar()->addMenu(tr("&Testing"));
    testMenu->addAction(doBonjourTestAction);

    // Tools Menu
    QMenu* toolsMenu = menuBar()->addMenu(tr("T&ools"));
    toolsMenu->addAction(runOfflineLocalisatonAction);

    // Window Menu
    QMenu* windowMenu = menuBar()->addMenu(tr("&Window"));

    QMenu* visionWindowMenu = windowMenu->addMenu(tr("&Vision"));
    visionWindowMenu->addAction(newVisionDisplayAction);

    QMenu* plotWindowMenu = windowMenu->addMenu(tr("&Plot"));
    plotWindowMenu->addAction(newPlotDisplayAction);

    QMenu* localisationWindowMenu = windowMenu->addMenu(tr("&Localisation"));
    localisationWindowMenu->addAction(newLocWMDisplayAction);

    QMenu* LUTWindowMenu = windowMenu->addMenu(tr("&Look Up Table"));
    LUTWindowMenu->addAction(newLUTDisplayAction);

    QMenu* networkWindowMenu = windowMenu->addMenu(tr("&Network"));

    windowMenu->addSeparator();
    // Add the actions for the dockable windows
    QList<QAction*> dockableActions;
    for(QList<QDockWidget*>::iterator dockable = m_dockable_windows.begin(); dockable != m_dockable_windows.end(); ++dockable)
    {
        dockableActions.push_back((*dockable)->toggleViewAction());
    }
    windowMenu->addActions(dockableActions);

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
    navigationToolbar = addToolBar(tr("&Navigation"));
    navigationToolbar->addAction(firstFrameAction);
    navigationToolbar->addAction(previousFrameAction);
    navigationToolbar->addAction(selectFrameAction);
    navigationToolbar->addAction(nextFrameAction);
    navigationToolbar->addAction(lastFrameAction);
    navigationToolbar->setObjectName(tr("navigationToolbar"));

    #ifndef WIN32
    // Connection Toolbar
    connectionToolBar = addToolBar("Connection");
    connectionToolBar->addWidget(m_connection_manager);
    connectionToolBar->setObjectName("connectionToolbar");
    #endif
}

void MainWindow::createConnections()
{
    // Connect to log file reader
    connect(LogReader,SIGNAL(sensorDataChanged(NUSensorsData*)),sensorDisplay, SLOT(SetSensorData(NUSensorsData*)));
    connect(LogReader,SIGNAL(sensorDataChanged(NUSensorsData*)),virtualRobot, SLOT(setSensorData(NUSensorsData*)));
    connect(LogReader,SIGNAL(frameChanged(int,int)),this, SLOT(imageFrameChanged(int,int)));
    connect(LogReader,SIGNAL(frameChanged(int,int)),offlinelocDialog,SLOT(SetFrame(int,int)));
    connect(LogReader,SIGNAL(ObjectDataChanged(const FieldObjects*)),objectDisplayLog, SLOT(setObjectData(const FieldObjects*)));
    connect(LogReader,SIGNAL(GameInfoChanged(const GameInformation*)),gameInfoDisplay, SLOT(setGameInfo(const GameInformation*)));
    connect(LogReader,SIGNAL(TeamInfoChanged(const TeamInformation*)),teamInfoDisplay, SLOT(setTeamInfo(const TeamInformation*)));

    connect(LogReader,SIGNAL(rawImageChanged(const NUImage*)),&glManager, SLOT(setRawImage(const NUImage*)));
    connect(LogReader,SIGNAL(rawImageChanged(const NUImage*)), frameInfo, SLOT(setRawImage(const NUImage*)));

    connect(LogReader,SIGNAL(fileOpened(QString)),this, SLOT(filenameChanged(QString)));
    connect(LogReader,SIGNAL(fileOpened(QString)),frameInfo, SLOT(setFrameSource(QString)));
    connect(LogReader,SIGNAL(fileClosed()),this, SLOT(fileClosed()));

    connect(LogReader,SIGNAL(cameraChanged(int)),virtualRobot, SLOT(setCamera(int)));
    connect(LogReader,SIGNAL(rawImageChanged(const NUImage*)),virtualRobot, SLOT(setRawImage(const NUImage*)));
    //connect(LogReader,SIGNAL(sensorDataChanged(const float*, const float*, const float*)),virtualRobot, SLOT(setSensorData(const float*, const float*, const float*)));
    connect(LogReader,SIGNAL(frameChanged(int,int)),virtualRobot, SLOT(processVisionFrame()));

    connect(LogReader,SIGNAL(rawImageChanged(const NUImage*)), this, SLOT(updateSelection()));

    connect(VisionStreamer,SIGNAL(rawImageChanged(const NUImage*)),&glManager, SLOT(setRawImage(const NUImage*)));
    connect(VisionStreamer,SIGNAL(rawImageChanged(const NUImage*)), this, SLOT(updateSelection()));
    connect(VisionStreamer,SIGNAL(rawImageChanged(const NUImage*)),virtualRobot, SLOT(setRawImage(const NUImage*)));
    connect(VisionStreamer,SIGNAL(rawImageChanged(const NUImage*)),virtualRobot, SLOT(processVisionFrame()));
    connect(VisionStreamer,SIGNAL(sensorsDataChanged(NUSensorsData*)),virtualRobot, SLOT(setSensorData(NUSensorsData*)));
    connect(VisionStreamer,SIGNAL(sensorsDataChanged(NUSensorsData*)),sensorDisplay, SLOT(SetSensorData(NUSensorsData*)));
    // Setup navigation control enabling/disabling
    connect(LogReader,SIGNAL(firstFrameAvailable(bool)),firstFrameAction, SLOT(setEnabled(bool)));
    connect(LogReader,SIGNAL(nextFrameAvailable(bool)),nextFrameAction, SLOT(setEnabled(bool)));
    connect(LogReader,SIGNAL(previousFrameAvailable(bool)),previousFrameAction, SLOT(setEnabled(bool)));
    connect(LogReader,SIGNAL(lastFrameAvailable(bool)),lastFrameAction, SLOT(setEnabled(bool)));
    connect(LogReader,SIGNAL(setFrameAvailable(bool)),selectFrameAction, SLOT(setEnabled(bool)));

    // Connect the virtual robot to the opengl manager.
    connect(virtualRobot,SIGNAL(imageDisplayChanged(const NUImage*,GLDisplay::display)),&glManager, SLOT(writeNUImageToDisplay(const NUImage*,GLDisplay::display)));
    connect(virtualRobot,SIGNAL(lineDisplayChanged(Line*, GLDisplay::display)),&glManager, SLOT(writeLineToDisplay(Line*, GLDisplay::display)));
    connect(virtualRobot,SIGNAL(classifiedDisplayChanged(ClassifiedImage*, GLDisplay::display)),&glManager, SLOT(writeClassImageToDisplay(ClassifiedImage*, GLDisplay::display)));
    connect(virtualRobot,SIGNAL(pointsDisplayChanged(std::vector<Point>, GLDisplay::display)),&glManager, SLOT(writePointsToDisplay(std::vector<Point>, GLDisplay::display)));
    connect(virtualRobot, SIGNAL(segmentsDisplayChanged(std::vector<std::vector<ColourSegment> >,GLDisplay::display)), &glManager, SLOT(writeSegmentsToDisplay(std::vector<std::vector<ColourSegment> >,GLDisplay::display)));
    //connect(virtualRobot,SIGNAL(transitionSegmentsDisplayChanged(std::vector< TransitionSegment >, GLDisplay::display)),&glManager, SLOT(writeTransitionSegmentsToDisplay(std::vector< TransitionSegment >, GLDisplay::display)));
    //connect(virtualRobot,SIGNAL(robotCandidatesDisplayChanged(std::vector< RobotCandidate >, GLDisplay::display)),&glManager, SLOT(writeRobotCandidatesToDisplay(std::vector< RobotCandidate >, GLDisplay::display)));
    connect(virtualRobot,SIGNAL(fittedLineDisplayChanged(std::vector< LSFittedLine >, GLDisplay::display)),&glManager, SLOT(writeLinesToDisplay(std::vector< LSFittedLine >, GLDisplay::display)));
    //connect(virtualRobot,SIGNAL(candidatesDisplayChanged(std::vector< ObjectCandidate >, GLDisplay::display)),&glManager, SLOT(writeCandidatesToDisplay(std::vector< ObjectCandidate >, GLDisplay::display)));
    connect(virtualRobot,SIGNAL(fieldObjectsDisplayChanged(FieldObjects*,GLDisplay::display)),&glManager,SLOT(writeFieldObjectsToDisplay(FieldObjects*,GLDisplay::display)));
    connect(virtualRobot,SIGNAL(linePointsDisplayChanged(std::vector< Point >,GLDisplay::display)),&glManager,SLOT(writeLinesPointsToDisplay(std::vector< Point >,GLDisplay::display)));
    //connect(virtualRobot,SIGNAL(cornerPointsDisplayChanged(std::vector< CornerPoint >,GLDisplay::display)),&glManager,SLOT(writeCornersToDisplay(std::vector< CornerPoint >,GLDisplay::display)));
    connect(virtualRobot,SIGNAL(edgeFilterChanged(QImage, GLDisplay::display)),&glManager,SLOT(stub(QImage, GLDisplay::display)));
    connect(virtualRobot,SIGNAL(fftChanged(QImage, GLDisplay::display)),&glManager,SLOT(stub(QImage, GLDisplay::display)));
    connect(virtualRobot,SIGNAL(fieldObjectsChanged(const FieldObjects*)),objectDisplayVision, SLOT(setObjectData(const FieldObjects*)));

    // Connect Statistics for Classification
    connect(virtualRobot,SIGNAL(updateStatistics(float *)),classification,SLOT(updateStatistics(float *)));

    // Connect the virtual robot to the incoming packets.
    connect(connection, SIGNAL(PacketReady(QByteArray*)), virtualRobot, SLOT(ProcessPacket(QByteArray*)));
    connect(classification,SIGNAL(selectionChanged()), this, SLOT(updateSelection()));
    connect(classification,SIGNAL(openLookupTableFile(QString)), virtualRobot, SLOT(loadLookupTableFile(QString)));
    connect(classification,SIGNAL(saveLookupTableFile(QString)), virtualRobot, SLOT(saveLookupTableFile(QString)));
    connect(classification,SIGNAL(displayStatusBarMessage(QString,int)), statusBar(), SLOT(showMessage(QString,int)));

    connect(classification,SIGNAL(autoSoftColourChanged(bool)),virtualRobot, SLOT(setAutoSoftColour(bool)));

    // Connect the virtual robot to the localisation widget and the localisation widget to the opengl manager
    //connect(virtualRobot,SIGNAL(imageDisplayChanged(const double*,bool,const double*)),localisation, SLOT(frameChange(const double*,bool,const double*)));
    connect(LogReader,SIGNAL(cameraChanged(int)),localisation, SLOT(setCamera(int)));
    //connect(LogReader,SIGNAL(sensorDataChanged(const float*, const float*, const float*)),localisation, SLOT(setSensorData(const float*, const float*, const float*)));
    connect(localisation,SIGNAL(updateLocalisationLine(WMLine*,int,GLDisplay::display)),&glManager,SLOT(writeWMLineToDisplay(WMLine*,int,GLDisplay::display)));
    connect(localisation,SIGNAL(updateLocalisationBall(float, float, float,GLDisplay::display)),&glManager,SLOT(writeWMBallToDisplay(float, float, float,GLDisplay::display)));
    connect(localisation,SIGNAL(removeLocalisationLine(GLDisplay::display)),&glManager,SLOT(clearDisplay(GLDisplay::display)));

    connect(offlinelocDialog,SIGNAL(LocalisationInfoChanged(QString)),locInfoDisplay, SLOT(setText(QString)));
    connect(offlinelocDialog,SIGNAL(SelfLocalisationInfoChanged(QString)),selflocInfoDisplay, SLOT(setText(QString)));
    connect(LocWmStreamer, SIGNAL(fieldObjectDataChanged(const FieldObjects*)),objectDisplayLog, SLOT(setObjectData(const FieldObjects*)));

    connect(virtualRobot, SIGNAL(clearPlots()), this, SLOT(clearPlots()));
}

void MainWindow::setColourTheme(ColourScheme newColors)
{
    QString style;
    currentColourScheme = newColors;
    switch(newColors)
    {
        case StevenColours:
            style = "QTextBrowser {background: #3F3F3F; color: #dcdccc}";
            break;
        default:
            currentColourScheme = DefaultColours;
            style = "QTextBrowser {background: white; color: black}";
            break;
    }
    this->setStyleSheet(style);
    return;
}

void MainWindow::RunOfflineLocalisation()
{
    offlinelocDialog->show();   // Make visible.
    offlinelocDialog->raise();  // Bring to foreground.
    return;
}

void MainWindow::openLog()
{
    QString intial_directory = ".";
    if(!m_previous_log_path.isEmpty())
    {
        intial_directory = m_previous_log_path;
    }
    QString fileName = QFileDialog::getOpenFileName(this,
                            tr("Open Replay File"), intial_directory,
                            tr("All NUbot Image Files(*.nul *.nif *.nurf *.strm);;NUbot Log Files (*.nul);;NUbot Image Files (*.nif);;NUbot Replay Files (*.nurf);;Stream File(*.strm);;All Files(*.*)"));
    if(!fileName.isEmpty())
    {
        QFileInfo file_info(fileName);
        if(file_info.exists())
        {
            m_previous_log_path = file_info.absolutePath();
            openLog(fileName);
        }
    }
    return;
}

void MainWindow::openLog(const QString& fileName)
{
    if (!fileName.isEmpty()){
        LogReader->openFile(fileName);
        LogReader->firstFrame();
    }
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

void MainWindow::saveViewImage()
{
    if(QMdiSubWindow *activeSubWindow = mdiArea->activeSubWindow())
    {
        QWidget* widget = activeSubWindow->widget();
        if(typeid(*widget) == typeid(GLDisplay))
        {
            GLDisplay* currWindow = qobject_cast<GLDisplay *>(widget);
            currWindow->snapshotToFile();
        }
        else if(typeid(*widget) == typeid(locWmGlDisplay))
        {
            //locWmGlDisplay* currWindow = qobject_cast<locWmGlDisplay *>(widget);
            //currWindow->snapshotToFile();
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
    QSettings settings("NUbots", "NUView");
    // Restore the main window.
    settings.beginGroup("mainWindow");
    restoreGeometry(settings.value("geometry").toByteArray());  // Set previous position/size
    restoreState(settings.value("state").toByteArray());    // restore the other main widgets layout.
    setColourTheme(ColourScheme(settings.value("ColorScheme").toUInt()));
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
    QSettings settings("NUbots", "NUView");

    // Save the main window setting
    settings.beginGroup("mainWindow");
    settings.setValue("geometry", saveGeometry()); // Save the position and size.
    settings.setValue("state", saveState()); // Save the main widget layouts.
//    settings.setValue("logFile",fileName);
    settings.setValue("ColorScheme", currentColourScheme);
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
        setWindowTitle(QString("NUView - ") + filename);
    }
    else
    {
        setWindowTitle(QString("NUView"));
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
    statusBar()->showMessage(message, 10000);
}

void MainWindow::selectFrame()
{


    bool ok;
    int selectedFrameNumber = QInputDialog::getInteger(this, tr("Select Frame"), tr("Enter frame to jump to:"), LogReader->currentFrame(), 1, LogReader->numFrames(), 1, &ok);
    if(ok)
    {
        LogReader->setFrame(selectedFrameNumber);
        offlinelocDialog->SetFrame(selectedFrameNumber);
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
        LogReader->setFrame(LogReader->currentFrame());
    }    
    return window;
}

QMdiSubWindow* MainWindow::createPlotDisplay()
{
    PlotDisplay* temp = new PlotDisplay(this);
    //connect signals
    connect(virtualRobot, SIGNAL(curveChanged(QVector<QPointF>, QString)), plotSelection, SLOT(curveNamesUpdated()));
    connect(virtualRobot, SIGNAL(curveChanged(QVector<QPointF>, QString)), temp, SLOT(updateCurve(QVector<QPointF>, QString)));
    connect(virtualRobot, SIGNAL(clearPlots()), temp, SLOT(clearMap()));
    QMdiSubWindow* window = mdiArea->addSubWindow(temp);
    temp->resize(320, 240);
    temp->show();
    return window;
}

QMdiSubWindow* MainWindow::createLocWmGlDisplay()
{
    locWmGlDisplay* temp = new locWmGlDisplay(this);
    connect(LogReader,SIGNAL(LocalisationDataChanged(const Localisation*)),temp, SLOT(SetLocalisation(const Localisation*)));
    connect(LogReader,SIGNAL(SelfLocalisationDataChanged(const SelfLocalisation*)),temp, SLOT(setSelfLocalisation(const SelfLocalisation*)));
    connect(LogReader,SIGNAL(sensorDataChanged(NUSensorsData*)),temp, SLOT(setSensorData(NUSensorsData*)));
    connect(LogReader, SIGNAL(ObjectDataChanged(const FieldObjects*)),temp, SLOT(setFieldObjects(const FieldObjects*)));
    connect(LocWmStreamer, SIGNAL(locwmDataChanged(const Localisation*)),temp, SLOT(SetLocalisation(const Localisation*)));
    connect(LocWmStreamer, SIGNAL(selfLocwmDataChanged(const SelfLocalisation*)),temp, SLOT(setSelfLocalisation(const SelfLocalisation*)));
    connect(LocWmStreamer, SIGNAL(fieldObjectDataChanged(const FieldObjects*)),temp, SLOT(setFieldObjects(const FieldObjects*)));
    connect(offlinelocDialog, SIGNAL(LocalisationChanged(const Localisation*)),temp, SLOT(SetLocalLocalisation(const Localisation*)));
    connect(offlinelocDialog, SIGNAL(SelfLocalisationChanged(const SelfLocalisation*)),temp, SLOT(setSelfLocalisation(const SelfLocalisation*)));
    QMdiSubWindow* window = mdiArea->addSubWindow(temp);
    temp->show();
    return window;
}

QMdiSubWindow* MainWindow::createLUTGlDisplay()
{
    LUTGlDisplay* temp = new LUTGlDisplay(this, &glManager);
    unsigned char* lut = virtualRobot->getLUT();
    temp->SetLUT(lut);
    connect(virtualRobot,SIGNAL(LUTChanged(unsigned char*)),temp,SLOT(SetLUT(unsigned char*)));
    QMdiSubWindow* window = mdiArea->addSubWindow(temp);
    temp->show();
    return window;
}

void MainWindow::updateSelection()
{
    virtualRobot->updateSelection(classification->getColourLabel(),classification->getSelectedColours());
}

void MainWindow::SelectColourAtPixel(int x, int y)
{
    if(virtualRobot->imageAvailable())
    {
        Pixel tempPixel = virtualRobot->selectRawPixel(x,y);

        QString message = "(";
        message.append(QString::number(x));
        message.append(",");
        message.append(QString::number(y));
        message.append(")");
        statusBar()->showMessage(message, 10000);
        classification->setColour(tempPixel);
    }
}

void MainWindow::ClassifySelectedColour()
{
    virtualRobot->UpdateLUT(classification->getColourLabel(),classification->getSelectedColours());
}

void MainWindow::SelectAndClassifySelectedPixel(int x, int y)
{
    if(virtualRobot->imageAvailable())
    {
        SelectColourAtPixel(x,y);
        ClassifySelectedColour();
    }
}
