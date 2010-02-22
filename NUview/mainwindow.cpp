#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "LayerSelectionWidget.h"
#include "WalkParameterWidget.h"
#include <QtGui>
#include <QMdiArea>
#include <QStatusBar>
#include <stdio.h>
#include <QWidget>
#include <iostream>
using namespace std;
ofstream debug;
ofstream errorlog;

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
{

    createActions();
    createMenus();
    createContextMenu();
    createToolBars();
    createStatusBar();

  // create mdi workspace
    mdiArea = new QMdiArea(this);
    mdiArea->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
    mdiArea->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);

    connection = new ConnectionWidget(this);
    addDockWidget(Qt::RightDockWidgetArea, connection);

    localisation = new LocalisationWidget(this);
    addDockWidget(Qt::BottomDockWidgetArea,localisation);

    classification = new ClassificationWidget(this);
    addDockWidget(Qt::RightDockWidgetArea, classification);

    miscDisplay = new GLDisplay(this,&glManager);
    mdiArea->addSubWindow(miscDisplay);

    horizonDisplay = new GLDisplay(this,&glManager);
    mdiArea->addSubWindow(horizonDisplay);

    classDisplay = new GLDisplay(this,&glManager);
    mdiArea->addSubWindow(classDisplay);

    imageDisplay = new GLDisplay(this,&glManager);
    mdiArea->addSubWindow(imageDisplay);

    // Disabled
    wmDisplay = NULL;
    //wmDisplay = new locWmGlDisplay(this);
    //mdiArea->addSubWindow(wmDisplay);

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

    mdiArea->tileSubWindows();
    setCentralWidget(mdiArea);
    currentFrameNumber = -1;

    // Add layer selection dock widget
    layerSelection = new LayerSelectionWidget(mdiArea,this);
    layerSelectionDock = new QDockWidget(layerSelection->windowTitle());
    layerSelectionDock->setWidget(layerSelection);
    addDockWidget(Qt::RightDockWidgetArea, layerSelectionDock);

    // Add walk parameter dock widget
    walkParameter = new WalkParameterWidget(mdiArea,this);
    walkParameterDock = new QDockWidget(walkParameter->windowTitle());
    walkParameterDock->setWidget(walkParameter);
    addDockWidget(Qt::RightDockWidgetArea, walkParameterDock);

    imageDisplay->setPrimaryDisplay(GLDisplay::rawImage);
    imageDisplay->setOverlayDrawing(GLDisplay::horizonLine,true,0.5);
    //imageDisplay->setOverlayDrawing(classifiedImage,true, 0.5);
    //imageDisplay->setOverlayDrawing(GLDisplay::classificationSelection,true);
    //imageDisplay->setOverlayDrawing(GLDisplay::greenHorizonScanPoints,true, QColor(255,0,0));
    //imageDisplay->setOverlayDrawing(GLDisplay::greenHorizonPoints,true, QColor(0,255,127));
    //imageDisplay->setOverlayDrawing(GLDisplay::horizontalScanPath,true, QColor(255,0,0));
    //imageDisplay->setOverlayDrawing(GLDisplay::verticalScanPath,true, QColor(0,255,127));
//    imageDisplay->setOverlayDrawing(GLDisplay::ObjectCandidates,true);

    //imageDisplay->setOverlayDrawing(GLDisplay::ObjectCandidates,true);
    //imageDisplay->setOverlayDrawing(GLDisplay::wmLeftLeg,true);
    //imageDisplay->setOverlayDrawing(GLDisplay::wmRightLeg,true);
    //imageDisplay->setOverlayDrawing(GLDisplay::wmBall,true);

    classDisplay->setPrimaryDisplay(GLDisplay::classifiedImage);
    classDisplay->setOverlayDrawing(GLDisplay::horizonLine,true,0.5);
    classDisplay->setOverlayDrawing(GLDisplay::classificationSelection,true);
//    classDisplay->setOverlayDrawing(GLDisplay::greenHorizonPoints,true, QColor(255,0,0));


    horizonDisplay->setPrimaryDisplay(GLDisplay::horizonLine);
//    horizonDisplay->setOverlayDrawing(GLDisplay::TransitionSegments,true);
//    horizonDisplay->setOverlayDrawing(GLDisplay::ObjectCandidates,true);

    miscDisplay->setPrimaryDisplay(GLDisplay::classificationSelection);

    miscDisplay->setOverlayDrawing(GLDisplay::FieldLines,true, QColor(255,0,0));
    miscDisplay->setPrimaryDisplay(GLDisplay::classificationSelection);
    setWindowTitle(QString("NUview"));
    qDebug() << "Main Window Started";
}

MainWindow::~MainWindow()
{
// Delete widgets and displays
    delete statusBar;
    delete miscDisplay;
    delete horizonDisplay;
    delete classDisplay;
    delete imageDisplay;
    delete classification;
    delete connection;
    delete localisation;
    delete layerSelection;
    delete layerSelectionDock;
    delete walkParameter;
    delete walkParameterDock;
    delete mdiArea;

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
    firstFrameAction->setIcon(QIcon(QString("../diagona/icon/16/138.png")));

    connect(firstFrameAction, SIGNAL(triggered()), this, SLOT(firstFrame()));

    // Previous Frame
    previousFrameAction = new QAction(tr("&Previous Frame"), this);
    previousFrameAction->setShortcut(QKeySequence::MoveToPreviousChar);
    previousFrameAction->setStatusTip(tr("Select the previous frame"));
    previousFrameAction->setIcon(QIcon(QString("../diagona/icon/16/132.png")));
    connect(previousFrameAction, SIGNAL(triggered()), this, SLOT(previousFrame()));

    // Select Frame
    selectFrameAction = new QAction(tr("&Select Frame..."), this);
    selectFrameAction->setShortcut(tr("Ctrl+G"));
    selectFrameAction->setStatusTip(tr("Select frame number to go to"));
    selectFrameAction->setIcon(QIcon(QString("../diagona/icon/16/134.png")));
    connect(selectFrameAction, SIGNAL(triggered()), this, SLOT(selectFrame()));

    // Next Frame
    nextFrameAction = new QAction(tr("&Next Frame"), this);
    nextFrameAction->setShortcut(QKeySequence::MoveToNextChar);
    nextFrameAction->setStatusTip(tr("Select next frame"));
    nextFrameAction->setIcon(QIcon(QString("../diagona/icon/16/131.png")));
    connect(nextFrameAction, SIGNAL(triggered()), this, SLOT(nextFrame()));

    // Last Frame
    lastFrameAction = new QAction(tr("&Last Frame"), this);
    lastFrameAction->setShortcut(QKeySequence::MoveToEndOfLine);
    lastFrameAction->setStatusTip(tr("Select last frame"));
    lastFrameAction->setIcon(QIcon(QString("../diagona/icon/16/137.png")));
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

    // Navigation Toolbar
    navigationToolbar = addToolBar(tr("N&avigation"));
    navigationToolbar->addAction(firstFrameAction);
    navigationToolbar->addAction(previousFrameAction);
    navigationToolbar->addAction(selectFrameAction);
    navigationToolbar->addAction(nextFrameAction);
    navigationToolbar->addAction(lastFrameAction);
    //windowDisplayToolbar = addToolBar(tr("&Display"));
}

void MainWindow::createStatusBar()
{
        statusBar = new QStatusBar;
        this->setStatusBar(statusBar);
        this->statusBar->showMessage("NUViewer Loaded",10000);
}

void MainWindow::open()
{
    fileName = QFileDialog::getOpenFileName(this,
                            tr("Open Replay File"), ".",
                            tr("NUbot Image Files (*.nif);;NUbot Replay Files (*.nurf);;NUbot Log Files (*.nul)"));

    setWindowTitle(QString("NUview - ") + fileName);
    if (!fileName.isEmpty()){
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
        /*if(typeid(*widget) == typeid(GLDisplay))
        {
            GLDisplay* currWindow = qobject_cast<GLDisplay *>(widget);
            currWindow->snapshotToClipboard();
        }*/
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
        /*if(typeid(*widget) == typeid(GLDisplay))
        {
            GLDisplay* currWindow = qobject_cast<GLDisplay *>(widget);
            validWidget = true;
            sourceSize = currWindow->imageSize();
        }*/

        if(validWidget)
        {
            sourceSize.scale(windowSize,Qt::KeepAspectRatio);
            activeSubWindow->resize(sourceSize);
        }
    }
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



void MainWindow::selectFrame()
{

    int selectedFrameNumber;
    bool ok;

    //selectedFrameNumber = QInputDialog::getInt(this, tr("Select Frame"), tr("Enter frame to jump to:"), currentFrameNumber, 1, totalFrameNumber, 1, &ok);

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
