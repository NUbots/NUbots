#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "LayerSelectionWidget.h"
#include <QtGui>
#include <QMdiArea>
#include <QStatusBar>
#include <stdio.h>
#include <QDebug>



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

    // Connect the virtual robot to the opengl manager.
    connect(&virtualRobot,SIGNAL(imageDisplayChanged(NUimage*,GLDisplay::display)),&glManager, SLOT(writeNUimageToDisplay(NUimage*,GLDisplay::display)));
    connect(&virtualRobot,SIGNAL(lineDisplayChanged(Line*, GLDisplay::display)),&glManager, SLOT(writeLineToDisplay(Line*, GLDisplay::display)));
    connect(&virtualRobot,SIGNAL(classifiedDisplayChanged(ClassifiedImage*, GLDisplay::display)),&glManager, SLOT(writeClassImageToDisplay(ClassifiedImage*, GLDisplay::display)));
    connect(&virtualRobot,SIGNAL(pointsDisplayChanged(std::vector< Vector2<int> >, GLDisplay::display)),&glManager, SLOT(writePointsToDisplay(std::vector< Vector2<int> >, GLDisplay::display)));

    // Connect the virtual robot to the incoming packets.
    connect(connection, SIGNAL(PacketReady(QByteArray*)), &virtualRobot, SLOT(ProcessPacket(QByteArray*)));
    connect(classification,SIGNAL(newSelection()), this, SLOT(updateSelection()));
    connect(classification,SIGNAL(openLookupTableFile(QString)), &virtualRobot, SLOT(loadLookupTableFile(QString)));
    connect(classification,SIGNAL(saveLookupTableFile(QString)), &virtualRobot, SLOT(saveLookupTableFile(QString)));
    connect(classification,SIGNAL(displayStatusBarMessage(QString,int)), statusBar, SLOT(showMessage(QString,int)));

    mdiArea->tileSubWindows();
    setCentralWidget(mdiArea);
    currentFrameNumber = -1;

    // Add layer selection dock widget
    layerSelection = new LayerSelectionWidget(mdiArea,this);
    layerSelectionDock = new QDockWidget(layerSelection->windowTitle());
    layerSelectionDock->setWidget(layerSelection);
    addDockWidget(Qt::RightDockWidgetArea, layerSelectionDock);

    imageDisplay->setPrimaryDisplay(GLDisplay::rawImage);
    //imageDisplay->setOverlayDrawing(GLDisplay::horizonLine,true,0.5);
    //imageDisplay->setOverlayDrawing(classifiedImage,true, 0.5);
    imageDisplay->setOverlayDrawing(GLDisplay::classificationSelection,true);
    //imageDisplay->setOverlayDrawing(GLDisplay::greenHorizonScanPoints,true, QColor(255,0,0));
    //imageDisplay->setOverlayDrawing(GLDisplay::greenHorizonPoints,true, QColor(0,255,127));
    imageDisplay->setOverlayDrawing(GLDisplay::horizontalScanPath,true, QColor(255,0,0));
    imageDisplay->setOverlayDrawing(GLDisplay::verticalScanPath,true, QColor(0,255,127));


    classDisplay->setPrimaryDisplay(GLDisplay::classifiedImage);
    classDisplay->setOverlayDrawing(GLDisplay::horizonLine,true,0.5);
    classDisplay->setOverlayDrawing(GLDisplay::classificationSelection,true);
    classDisplay->setOverlayDrawing(GLDisplay::greenHorizonPoints,true, QColor(255,0,0));


    horizonDisplay->setPrimaryDisplay(GLDisplay::horizonLine);

    miscDisplay->setPrimaryDisplay(GLDisplay::classificationSelection);

    setWindowTitle(QString("NUview"));
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
    delete layerSelection;
    delete layerSelectionDock;
    delete mdiArea;

// Delete Actions
    delete openAction;
    delete exitAction;
    delete firstFrameAction;
    delete previousFrameAction;
    delete selectFrameAction;
    delete nextFrameAction;
    delete lastFrameAction;
    delete cascadeAction;
    delete tileAction;
    return;

}

void MainWindow::createActions()
{
    // Open Action
    openAction = new QAction(tr("&Open..."), this);
    openAction->setShortcut(tr("Ctrl+O"));
    openAction->setStatusTip(tr("Open a new file"));
    openAction->setIcon(this->style()->standardIcon(QStyle::SP_DialogOpenButton));
    connect(openAction, SIGNAL(triggered()), this, SLOT(open()));

    // Exit Action
    exitAction = new QAction(tr("E&xit"), this);
    exitAction->setShortcut(tr("Ctrl+Q"));
    exitAction->setStatusTip(tr("Exit the application"));
    exitAction->setIcon(this->style()->standardIcon(QStyle::SP_DialogCloseButton));
    connect(exitAction, SIGNAL(triggered()), qApp, SLOT(closeAllWindows()));

    // First Frame
    firstFrameAction = new QAction(tr("&First Frame"), this);
    //firstFrameAction->setShortcut(tr("Ctrl+O"));
    firstFrameAction->setStatusTip(tr("Go to the first frame of the replay"));
    firstFrameAction->setIcon(QIcon(QString("../diagona/icon/16/138.png")));

    connect(firstFrameAction, SIGNAL(triggered()), this, SLOT(firstFrame()));

    // Previous Frame
    previousFrameAction = new QAction(tr("&Previous Frame"), this);
    previousFrameAction->setShortcut(tr("Left"));
    previousFrameAction->setStatusTip(tr("Select the previous frame"));
    previousFrameAction->setIcon(QIcon(QString("../diagona/icon/16/132.png")));
    connect(previousFrameAction, SIGNAL(triggered()), this, SLOT(previousFrame()));

    // Select Frame
    selectFrameAction = new QAction(tr("&Select Frame..."), this);
    //selectFrameAction->setShortcut(tr("Ctrl+O"));
    selectFrameAction->setStatusTip(tr("Select frame number to go to"));
    selectFrameAction->setIcon(QIcon(QString("../diagona/icon/16/134.png")));
    connect(selectFrameAction, SIGNAL(triggered()), this, SLOT(selectFrame()));

    // Next Frame
    nextFrameAction = new QAction(tr("&Next Frame"), this);
    nextFrameAction->setShortcut(tr("Right"));
    nextFrameAction->setStatusTip(tr("Select next frame"));
    nextFrameAction->setIcon(QIcon(QString("../diagona/icon/16/131.png")));
    connect(nextFrameAction, SIGNAL(triggered()), this, SLOT(nextFrame()));

    // Last Frame
    lastFrameAction = new QAction(tr("&Last Frame"), this);
    //lastFrameAction->setShortcut(tr("Ctrl+O"));
    lastFrameAction->setStatusTip(tr("Select last frame"));
    lastFrameAction->setIcon(QIcon(QString("../diagona/icon/16/137.png")));
    connect(lastFrameAction, SIGNAL(triggered()), this, SLOT(lastFrame()));


    // Cascade windows
    cascadeAction = new QAction(tr("&Cascade Window"), this);
    //lastFrameAction->setShortcut(tr("Ctrl+O"));
    cascadeAction->setStatusTip(tr("Cascade windows in Main Area"));
    connect(cascadeAction, SIGNAL(triggered()), this, SLOT(cascade()));

    // Tile windows
    tileAction = new QAction(tr("&Tile Window"), this);
    //lastFrameAction->setShortcut(tr("Ctrl+O"));
    tileAction->setStatusTip(tr("Tiles windows in Main Area"));
    connect(tileAction, SIGNAL(triggered()), this, SLOT(tile()));
}

void MainWindow::createMenus()
{
    // File Menu
    fileMenu = menuBar()->addMenu(tr("&File"));
    fileMenu->addAction(openAction);
    fileMenu->addSeparator();
    fileMenu->addAction(exitAction);

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
    windowMenu->addAction(cascadeAction);
    windowMenu->addAction(tileAction);
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
                            tr("NUbot Image Files (*.nif);;NUbot Replay Files (*.nurf)"));

    setWindowTitle(QString("NUview - ") + fileName);
    if (!fileName.isEmpty()){
        //loadFile(fileName);
        const char* filestr = fileName.toAscii();

        totalFrameNumber = virtualRobot.loadFile(filestr);
        QString message = "Opening File: ";
        message.append(fileName);
        this->statusBar->showMessage(message,10000);
        qDebug() << "Number of Frames in File: " << totalFrameNumber;
        firstFrame();
    }
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

void MainWindow::keyPressEvent ( QKeyEvent * event )
{
    if(event->key() == Qt::Key_Z && (event->modifiers() & Qt::ControlModifier))
    {
        if(event->isAutoRepeat() == false)
        {
            virtualRobot.UndoLUT();
        }
    }
}
