#include "createqwtsymboldialog.h"
#include "ui_createqwtsymboldialog.h"
#include <QColorDialog>
#include <QPen>
#include <QBrush>
#include <QPainter>

CreateQwtSymbolDialog::CreateQwtSymbolDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::CreateQwtSymbolDialog),
    fillColour(Qt::gray),
    borderColour(Qt::black),
    style(QwtSymbol::NoSymbol),
    size(3)
{
    ui->setupUi(this);

    for(int i=0; i<15; i++) {
        ui->styleCombo->addItem(getSymbolName(getSymbolFromInt(i)));
    }
    ui->styleCombo->setCurrentIndex(ui->styleCombo->findText(getSymbolName(style)));
    ui->sizeSpin->setValue(size);
    updateButtonColour(ui->fillPB, fillColour);
    updateButtonColour(ui->borderPB, borderColour);

    connect(ui->fillPB, SIGNAL(clicked()), this, SLOT(getNewFillColour()));
    connect(ui->borderPB, SIGNAL(clicked()), this, SLOT(getNewBorderColour()));
    connect(ui->sizeSpin, SIGNAL(valueChanged(int)), this, SLOT(setSymbolSize(int)));
    connect(ui->styleCombo, SIGNAL(activated(int)), this, SLOT(setSymbolStyle(int)));
    connect(ui->cancelPB, SIGNAL(clicked()), this, SLOT(hide()));
    connect(ui->acceptPB, SIGNAL(clicked()), this, SLOT(hide()));
    connect(ui->acceptPB, SIGNAL(clicked()), this, SLOT(createNewSymbol()));
}

CreateQwtSymbolDialog::~CreateQwtSymbolDialog()
{
    delete ui;
}

void CreateQwtSymbolDialog::getNewFillColour()
{
    fillColour = QColorDialog::getColor(fillColour,this);
    updateButtonColour(ui->fillPB, fillColour);
}

void CreateQwtSymbolDialog::getNewBorderColour()
{
    borderColour = QColorDialog::getColor(borderColour,this);
    updateButtonColour(ui->borderPB, borderColour);
}

void CreateQwtSymbolDialog::createNewSymbol()
{
    emit newSymbolCreated(QwtSymbol(style, QBrush(fillColour), QPen(borderColour), QSize(size, size)));
}

void CreateQwtSymbolDialog::updateButtonColour(QToolButton *button, QColor colour)
{
    QPixmap tempPixmap(22,22);
    QPainter painter(&tempPixmap);
    tempPixmap.fill(colour);
    painter.drawRect(0,0,tempPixmap.width(),tempPixmap.height());
    button->setIcon(QIcon(tempPixmap));
    button->setAutoRaise(true);
}

QwtSymbol::Style CreateQwtSymbolDialog::getSymbolFromInt(int i)
{
    switch(i) {
    case 1: return QwtSymbol::Ellipse;
    case 2: return QwtSymbol::Rect;
    case 3: return QwtSymbol::Diamond;
    case 4: return QwtSymbol::UTriangle;
    case 5: return QwtSymbol::DTriangle;
    case 6: return QwtSymbol::LTriangle;
    case 7: return QwtSymbol::RTriangle;
    case 8: return QwtSymbol::Cross;
    case 9: return QwtSymbol::XCross;
    case 10: return QwtSymbol::HLine;
    case 11: return QwtSymbol::VLine;
    case 12: return QwtSymbol::Star1;
    case 13: return QwtSymbol::Star2;
    case 14: return QwtSymbol::Hexagon;
    default: return QwtSymbol::NoSymbol;
    }
}

QString CreateQwtSymbolDialog::getSymbolName(QwtSymbol::Style id)
{
    switch(id) {
    case QwtSymbol::NoSymbol: return "No Symbol";
    case QwtSymbol::Ellipse: return "Ellipse";
    case QwtSymbol::Rect: return "Rectangle";
    case QwtSymbol::Diamond: return "Diamond";
    case QwtSymbol::UTriangle: return "Triangle (up)";
    case QwtSymbol::DTriangle: return "Triangle (down)";
    case QwtSymbol::LTriangle: return "Triangle (left)";
    case QwtSymbol::RTriangle: return "Triangle (right)";
    case QwtSymbol::Cross: return "Cross (+)";
    case QwtSymbol::XCross: return "Cross (X)";
    case QwtSymbol::HLine: return "Horizontal Bar";
    case QwtSymbol::VLine: return "Vertical Bar";
    case QwtSymbol::Star1: return "Asterisk";
    case QwtSymbol::Star2: return "Star";
    case QwtSymbol::Hexagon: return "Hexagon";
    default: return "Invalid";
    }
}
