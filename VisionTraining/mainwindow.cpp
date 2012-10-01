#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <string>
#include <QFileDialog>
#include <QInputDialog>
#include "labeleditor.h"
#include "labelgenerator.h"
#include "visionoptimiser.h"
#include "visioncomparitor.h"

using namespace std;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    
    QObject::connect(ui->autoPB, SIGNAL(clicked()), this, SLOT(generateLabels()));
    QObject::connect(ui->modPB, SIGNAL(clicked()), this, SLOT(modifyLabels()));
    QObject::connect(ui->compPB, SIGNAL(clicked()), this, SLOT(compareParams()));
    QObject::connect(ui->optPB, SIGNAL(clicked()), this, SLOT(runOptimiser()));
    QObject::connect(ui->exitPB, SIGNAL(clicked()), this, SLOT(close()));
    QObject::connect(ui->browsePB, SIGNAL(clicked()), this, SLOT(getDirectory()));

    ui->dirEdit->setText(QString(getenv("HOME")) + QString("/Images/FYP/Final100/"));
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
    LabelGenerator lg(this);
    lg.show();
    lg.run(ui->dirEdit->text().toStdString());    
}

void MainWindow::modifyLabels()
{
    LabelEditor le(this);
    le.show();
    le.run(ui->dirEdit->text().toStdString());
}

void MainWindow::compareParams()
{
    QString config0 = QFileDialog::getOpenFileName(this, "Select First Config", (string(getenv("HOME")) + string("/Images/FYP/Final100/")).c_str());
    if(!config0.isNull()) {
        QString config1 = QFileDialog::getOpenFileName(this, "Select First Config", (string(getenv("HOME")) + string("/Images/FYP/Final100/")).c_str());
        if(!config1.isNull()) {
            string image = ui->dirEdit->text().toStdString() + string("image.strm");
            string lut = ui->dirEdit->text().toStdString() + string("default.lut");

            VisionComparitor comp;
            comp.show();
            comp.run(image, lut, config0.toStdString(), config1.toStdString());
        }
    }
}

void MainWindow::runOptimiser()
{
    bool ok;
    QStringList l;
    l.append("PSO");
    l.append("PGRL");
    l.append("EHCLS");
    QString s = QInputDialog::getItem(this, "Select Optimiser", "Select the preferred optimiser", l, 0, false, &ok);
    if(ok) {
        int iterations = QInputDialog::getInt(this, "Iterations", "Select the number of optimiser iterations.", 100, 1, 1000000, 1, &ok);
        if(ok) {
            VisionOptimiser opt(this, VisionOptimiser::getChoiceFromQString(s));
            opt.show();

            opt.run(ui->dirEdit->text().toStdString(), iterations);
        }
    }
}
