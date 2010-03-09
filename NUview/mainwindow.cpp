#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "LayerSelectionWidget.h"
#include "WalkParameterWidget.h"
#include <QtGui>
#include <QMdiArea>
#include <QStatusBar>
#include <stdio.h>
#include <QDebug>
#include <QWidget>
#include <iostream>
#include <QTabWidget>
#include <typeinfo>
using namespace std;
ofstream debug;
ofstream errorlog;

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{
    qDebug() << "NUview is starting in: MainWindow.cpp";
    createActions();
    createMenus();
    createContextMenu();
    createToolBars();
    createStatusBar();

  // create mdi workspace
    mdiArea = new QMdiArea(this);
    mdiArea->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    mdiArea->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);


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
    walkParameter = new WalkParameterWidget(mdiArea,this);
    networkTabs->addTab(walkParameter, walkParameter->objectName());
    networkTabDock = new QDockWidget("Network");
    networkTabDock->setWidget(networkTabs);
    networkTabDock->setObjectName(tr("networkTab"));
    addDockWidget(Qt::RightDockWidgetArea, networkTabDock);

    createConnections();
    setCentralWidget(mdiArea);
    currentFrameNumber = -1;


    setWindowTitle(QString("NUview"));
    glManager.clearAllDisplays();
    readSettings();
    qDebug() << "Main Window Started";
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
    delete mdiArea;
    delete visionTabs;
    delete networkTabs;

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
    return;
}

void MainWindow::createActions()
{
    // Open Action
    openAction = new QAction(QIcon(":/icons/open.png"),tr("&Open..."), this);
    openAction->setShortcut(QKeySequence::Open);
    openAction->setStatusTip(tr("Open a new file"));
    connect(openAction, SIGNAL(triggered()), this, SLOT(open()));

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

    connect(firstFrameAction, SIGNAL(triggered()), this, SLOT(firstFrame()));

    // Previous Frame
    previousFrameAction = new QAction(tr("&Previous Frame"), this);
    previousFrameAction->setShortcut(QKeySequence::MoveToPreviousChar);
    previousFrameAction->setStatusTip(tr("Select the previous frame"));
    previousFrameAction->setIcon(QIcon(QString(":/icons/previous.png")));
    connect(previousFrameAction, SIGNAL(triggered()), this, SLOT(previousFrame()));

    // Select Frame
    selectFrameAction = new QAction(tr("&Select Frame..."), this);
    selectFrameAction->setShortcut(tr("Ctrl+G"));
    selectFrameAction->setStatusTip(tr("Select frame number to go to"));
    selectFrameAction->setIcon(QIcon(QString(":/icons/select.png")));
    connect(selectFrameAction, SIGNAL(triggered()), this, SLOT(selectFrame()));

    // Next Frame
    nextFrameAction = new QAction(tr("&Next Frame"), this);
    nextFrameAction->setShortcut(QKeySequence::MoveToNextChar);
    nextFrameAction->setStatusTip(tr("Select next frame"));
    nextFrameAction->setIcon(QIcon(QString(":/icons/next.png")));
    connect(nextFrameAction, SIGNAL(triggered()), this, SLOT(nextFrame()));

    // Last Frame
    lastFrameAction = new QAction(tr("&Last Frame"), this);
    lastFrameAction->setShortcut(QKeySequence::MoveToEndOfLine);
    lastFrameAction->setStatusTip(tr("Select last frame"));
    lastFrameAction->setIcon(QIcon(QString(":/icons/last.png")));
    connect(lastFrameAction, SIGNAL(triggered()), this, SLOT(lastFrame()));


    // Cascade windows
    cascadeAction = new QAction(tr("&Cascade Window"), this);
    cascadeAction->setStatusTip(tr("Cascade windows in Main Area"));
    connect(cascadeAction, SIGNAL(triggered()), this, SLOT(cascade()));

    // Tile windows
    tileAction = new QAction(tr("&Tile Window"), this);
    tileAction->setStatusTip(tr("Tiles windows in Main Area"));
    connect(tileAction, SIGNAL(triggered()), this, SLOT(tile()));

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

    // Window Menu
    windowMenu = menuBar()->addMenu(tr("&Window"));

    visionWindowMenu = windowMenu->addMenu(tr("&Vision"));
    visionWindowMenu->addAction(newVisionDisplayAction);

    localisationWindowMenu = windowMenu->addMenu(tr("&Localisation"));
    localisationWindowMenu->addAction(newLocWMDisplayAction);

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
        this->statusBar->showMessage("NUViewer Loaded",10000);
}

void MainWindow::createConnections()
{
    // Connect the virtual robot to the opengl manager.
    connect(&virtualRobot,SIGNAL(imageDisplayChanged(NUimage*,GLDisplay::display)),&glManager, SLOT(writeNUimageToDisplay(NUimage*,GLDisplay::display)));
    connect(&virtualRobot,SIGNAL(lineDisplayChanged(Line*, GLDisplay::display)),&glManager, SLOT(writeLineToDisplay(Line*, GLDisplay::display)));
    connect(&virtualRobot,SIGNAL(classifiedDisplayChanged(ClassifiedImage*, GLDisplay::display)),&glManager, SLOT(writeClassImageToDisplay(ClassifiedImage*, GLDisplay::display)));
    connect(&virtualRobot,SIGNAL(pointsDisplayChanged(std::vector< Vector2<int> >, GLDisplay::display)),&glManager, SLOT(writePointsToDisplay(std::vector< Vector2<int> >, GLDisplay::display)));
    connect(&virtualRobot,SIGNAL(transitionSegmentsDisplayChanged(std::vector< TransitionSegment >, GLDisplay::display)),&glManager, SLOT(writeTransitionSegmentsToDisplay(std::vector< TransitionSegment >, GLDisplay::display)));
    //connect(&virtualRobot,SIGNAL(robotCandidatesDisplayChanged(std::vector< RobotCandidate >, GLDisplay::display)),&glManager, SLOT(writeRobotCandidatesToDisplay(std::vector< RobotCandidate >, GLDisplay::display)));
    connect(&virtualRobot,SIGNAL(lineDetectionDisplayChanged(std::vector< LSFittedLine >, GLDisplay::display)),&glManager, SLOT(writeFieldLinesToDisplay(std::vector< LSFittedLine >, GLDisplay::display)));
    connect(&virtualRobot,SIGNAL(candidatesDisplayChanged(std::vector< ObjectCandidate >, GLDisplay::display)),&glManager, SLOT(writeCandidatesToDisplay(std::vector< ObjectCandidate >, GLDisplay::display)));
    connect(&virtualRobot,SIGNAL(drawFO_Ball(float, float, float,GLDisplay::display)),&glManager,SLOT(writeWMBallToDisplay(float, float, float,GLDisplay::display) ));
    // Connect the virtual robot to the incoming packets.
    connect(connection, SIGNAL(PacketReady(QByteArray*)), &virtualRobot, SLOT(ProcessPacket(QByteArray*)));
    connect(classification,SIGNAL(newSelection()), this, SLOT(updateSelection()));
    connect(classification,SIGNAL(openLookupTableFile(QString)), &virtualRobot, SLOT(loadLookupTableFile(QString)));
    connect(classification,SIGNAL(saveLookupTableFile(QString)), &virtualRobot, SLOT(saveLookupTableFile(QString)));
    connect(classification,SIGNAL(displayStatusBarMessage(QString,int)), statusBar, SLOT(showMessage(QString,int)));

    // Connect the virtual robot to the localisation widget and the localisation widget to the opengl manager
    connect(&virtualRobot,SIGNAL(imageDisplayChanged(double*,bool,double*)),localisation, SLOT(frameChange(double*,bool,double*)));
    connect(localisation,SIGNAL(updateLocalisationLine(WMLine*,int,GLDisplay::display)),&glManager,SLOT(writeWMLineToDisplay(WMLine*,int,GLDisplay::display)));
    connect(localisation,SIGNAL(updateLocalisationBall(float, float, float,GLDisplay::display)),&glManager,SLOT(writeWMBallToDisplay(float, float, float,GLDisplay::display)));
    connect(localisation,SIGNAL(removeLocalisationLine(GLDisplay::display)),&glManager,SLOT(clearDisplay(GLDisplay::display)));
}

void MainWindow::open()
{
    fileName = QFileDialog::getOpenFileName(this,
                            tr("Open Replay File"), ".",
                            tr("All NUbot Image Files(*.nul;*.nif;*.nurf);;NUbot Log Files (*.nul);;NUbot Image Files (*.nif);;NUbot Replay Files (*.nurf);;All Files(*.*)"));
    openFile(fileName);
}

void MainWindow::openFile(const QString& fileName)
{
    if (!fileName.isEmpty()){
        this->fileName = fileName;
        setWindowTitle(QString("NUview - ") + fileName);
        glManager.clearAllDisplays();
        totalFrameNumber = virtualRobot.loadFile(fileName);
        QString message = "Opening File: ";
        message.append(fileName);
        this->statusBar->showMessage(message,10000);
        qDebug() << "Number of Frames in File: " << totalFrameNumber;
        firstFrame();
        if(virtualRobot.fileType == QString("nul"))
        {
            previousFrameAction->setEnabled(false);
            selectFrameAction->setEnabled(false);
            lastFrameAction->setEnabled(false);
        }
        else
        {
            previousFrameAction->setEnabled(true);
            selectFrameAction->setEnabled(true);
            lastFrameAction->setEnabled(true);
        }
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


void MainWindow::firstFrame()
{
    currentFrameNumber = 1;
    LoadFrame(currentFrameNumber);
    return;
}

void MainWindow::previousFrame()
{
    if (!fileName.isEmpty() && currentFrameNumber > 1){
        currentFrameNumber = currentFrameNumber -1;
        LoadFrame(currentFrameNumber);
    }
    return;
}

QMdiSubWindow* MainWindow::createGLDisplay()
{
    GLDisplay* temp = new GLDisplay(this, &glManager);
    QMdiSubWindow* window = mdiArea->addSubWindow(temp);
    temp->show();
    return window;
}

QMdiSubWindow* MainWindow::createLocWmGlDisplay()
{
    locWmGlDisplay* temp = new locWmGlDisplay(this);
    QMdiSubWindow* window = mdiArea->addSubWindow(temp);
    temp->show();
    return window;
}

void MainWindow::selectFrame()
{
    int selectedFrameNumber;
    bool ok = true;

    selectedFrameNumber = QInputDialog::getInteger(this, tr("Select Frame"), tr("Enter frame to jump to:"), currentFrameNumber, 1, totalFrameNumber, 1, &ok);

    if (ok && !fileName.isEmpty() && selectedFrameNumber <= totalFrameNumber && selectedFrameNumber >= 1){
        currentFrameNumber = selectedFrameNumber;
        LoadFrame(currentFrameNumber);
    }
    return;
}

void MainWindow::nextFrame()
{
    if (!fileName.isEmpty() && currentFrameNumber < totalFrameNumber){
        currentFrameNumber = currentFrameNumber +1;
        LoadFrame(currentFrameNumber);
    }
    return;
}

void MainWindow::lastFrame()
{
    if (!fileName.isEmpty()){
        currentFrameNumber = totalFrameNumber;
        LoadFrame(currentFrameNumber);
    }
    return;
}

void MainWindow::LoadFrame(int frameNumber)
{
    //glManager.clearAllDisplays(); // Turn this on if we have trouble with old data being displayed.
    virtualRobot.loadFrame(frameNumber);
    updateSelection();
    QString message = "Frame Loaded:  Number ";
    message.append(QString::number(frameNumber));
    message.append("/");
    message.append(QString::number(totalFrameNumber));
    this->statusBar->showMessage(message, 10000);
    return;
}

//WINDOW MENU
void MainWindow::cascade()
{
    mdiArea->cascadeSubWindows();
    return;
}

void MainWindow::tile()
{
    mdiArea->tileSubWindows();
    return;
}

void MainWindow::updateSelection()
{
    virtualRobot.updateSelection(classification->getColourLabel(),classification->getSelectedColours(ClassificationWidget::YCbCr));
}

void MainWindow::SelectColourAtPixel(int x, int y)
{
    if(virtualRobot.imageAvailable())
    {
        pixels::Pixel tempPixel = virtualRobot.selectRawPixel(x,y);

        QString message = "(";
        message.append(QString::number(x));
        message.append(",");
        message.append(QString::number(y));
        message.append(")");
        this->statusBar->showMessage(message, 10000);
        classification->setColour(tempPixel,ClassificationWidget::YCbCr);
    }
}

void MainWindow::ClassifySelectedColour()
{
    virtualRobot.UpdateLUT(classification->getColourLabel(),classification->getSelectedColours(ClassificationWidget::YCbCr));
}

void MainWindow::SelectAndClassifySelectedPixel(int x, int y)
{
    if(virtualRobot.imageAvailable())
    {
        SelectColourAtPixel(x,y);
        ClassifySelectedColour();
    }
}
