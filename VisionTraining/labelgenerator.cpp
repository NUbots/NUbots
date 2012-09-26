#include "labelgenerator.h"
#include "ui_labelgenerator.h"
#include "Vision/VisionWrapper/visioncontrolwrappertraining.h"
#include <QApplication>

LabelGenerator::LabelGenerator(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::LabelGenerator)
{
    ui->setupUi(this);
    QObject::connect(ui->cancelPB, SIGNAL(clicked()), this, SLOT(setCancelled()));
}

LabelGenerator::~LabelGenerator()
{
    delete ui;
}

bool LabelGenerator::run(const string &dir)
{
    //find number of images
    int num_images=0,
        cur_image=0;
    NUImage temp;
    ifstream infile((dir + string("/image.strm")).c_str());
    ofstream outfile((dir + string("/labels.strm")).c_str());
    cancelled = false;
    while(!cancelled && infile.good()) {
        infile >> temp;
        infile.peek();
        num_images++;
    }
    infile.close();
    
    //exit if input failed
    if(num_images == 0)
        return false;
            
    //setup vision system
    VisionControlWrapper* vision = VisionControlWrapper::getInstance();
    vision->setImageStream(dir + string("/image.strm"));
    vision->setSensorStream(dir + string("/sensor.strm"));
    vision->setLUT(dir + string("/default.lut"));
    
    ui->progressBar->setMaximum(num_images);
    
    while(!cancelled && vision->runFrame()) {
        vision->printLabels(outfile);
        ui->progressBar->setValue(cur_image);
        QApplication::processEvents();
        cur_image++;
    }
    
    return cancelled;
}
