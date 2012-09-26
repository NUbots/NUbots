#include "labelgenerator.h"
#include "ui_labelgenerator.h"
#include "Vision/VisionWrapper/visioncontrolwrappertraining.h"

LabelGenerator::LabelGenerator(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::LabelGenerator)
{
    ui->setupUi(this);
}

LabelGenerator::~LabelGenerator()
{
    delete ui;
}

bool LabelGenerator::run(const string &dir)
{
    VisionControlWrapper* vision = VisionControlWrapper::getInstance();
    
    
    vision->setImageStream(dir + string("/image.strm"));
    vision->setSensorStream(dir + string("/sensor.strm"));
    vision->setLUT(dir + string("/default.lut"));
}
