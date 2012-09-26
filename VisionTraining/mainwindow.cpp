#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <string>
#include <QFileDialog>

using namespace std;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    
    QObject::connect(ui->autoPB, SIGNAL(clicked()), this, SLOT(generateLabels()));
    QObject::connect(ui->modPB, SIGNAL(clicked()), this, SLOT(modifyLabels()));
    QObject::connect(ui->viewPB, SIGNAL(clicked()), this, SLOT(viewStream()));
    QObject::connect(ui->optPB, SIGNAL(clicked()), this, SLOT(runOptimiser()));
    QObject::connect(ui->exitPB, SIGNAL(clicked()), this, SLOT(close()));
    QObject::connect(ui->dirEdit, SIGNAL(clicked()), this, SLOT(getDirectory()));
}

MainWindow::~MainWindow()
{
    delete ui;
}


void MainWindow::getDirectory()
{
    QString dir = QFileDialog::getExistingDirectory(this, "Select Directory", (string(getenv("HOME")) + string("/Images/FYP/Final100/")).c_str());
    if(!dir.isNull())
        ui->dirEdit->setText(dir);        
}

void MainWindow::generateLabels()
{
    
}

void MainWindow::modifyLabels()
{
    
}

void MainWindow::viewStream()
{
    
}

void MainWindow::runOptimiser()
{
    
}
