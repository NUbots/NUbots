#include "ObjectDisplayWidget.h"
#include "Infrastructure/FieldObjects/FieldObjects.h"
#include <sstream>

ObjectDisplayWidget::ObjectDisplayWidget(QWidget *parent) :
    QTextBrowser(parent)
{
//    QPalette p =  this->palette();
//    p.setColor(QPalette::Base, QColor(0x3f, 0x3f, 0x3f));
//    p.setColor(QPalette::Text, Qt::white);
//    this->setPalette(p);
}

void ObjectDisplayWidget::setObjectData(const FieldObjects* newObjectData)
{
    QString displayText(newObjectData->toString(true).c_str());
    this->setText(displayText);
}
