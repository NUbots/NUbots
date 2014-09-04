#include "startoptionsdialog.h"
#include "ui_startoptionsdialog.h"

StartOptionsDialog::StartOptionsDialog(QWidget *parent) :
    QDialog(parent),
    valid(false),
    camera(false),
    sensorstream(true),
    automatic(true),
    folder(false),
    ui(new Ui::StartOptionsDialog)
{
    ui->setupUi(this);

    connect(ui->buttonBox, SIGNAL(accepted()), this, SLOT(setValid()));
    connect(ui->buttonBox, SIGNAL(rejected()), this, SLOT(setInValid()));

    connect(ui->radioButton_SrcCam, SIGNAL(toggled(bool)), this, SLOT(setInputCamera(bool)));
    connect(ui->radioButton_SrcCam, SIGNAL(toggled(bool)), ui->radioButton_SenOn, SLOT(setDisabled(bool)));
    connect(ui->radioButton_SrcCam, SIGNAL(toggled(bool)), ui->radioButton_SenOff, SLOT(setDisabled(bool)));
    connect(ui->radioButton_SenOn, SIGNAL(toggled(bool)), this, SLOT(setSensorStream(bool)));
    connect(ui->radioButton_FileAuto, SIGNAL(toggled(bool)), this, SLOT(setAutomatic(bool)));
    connect(ui->radioButton_FileFolder, SIGNAL(toggled(bool)), this, SLOT(setFolderBased(bool)));
}

StartOptionsDialog::~StartOptionsDialog()
{
    delete ui;
}
