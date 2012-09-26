#include "labeleditor.h"
#include "ui_labeleditor.h"

LabelEditor::LabelEditor(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::LabelEditor)
{
    ui->setupUi(this);
    halted = false;
    
    //generate interface
    int y=40;
    for(int i=0; i<4; i++) {
        QLabel label(ui->groupBox);
        QSpinBox ispin(ui->groupBox);
        QDoubleSpinBox dspin(ui->groupBox);
        QSlider slider(ui->groupBox);
        label.setGeometry(10, y, 50, 20);
        slider.setGeometry(80, y, 160, 20);
        dspin.setGeometry(80, y, 60, 20);
        ispin.setGeometry(250, y, 50, 20);        
        y+=40;
    }
    
    QObject::connect(ui->exitPB, SIGNAL(clicked()), this, SLOT(exit()));
}

LabelEditor::~LabelEditor()
{
    delete ui;
}

int LabelEditor::run(string dir)
{
    while(!halted);
    close();
}
