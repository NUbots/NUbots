#include "ObjectDisplayWidget.h"
#include "Infrastructure/FieldObjects/FieldObjects.h"
#include <sstream>

ObjectDisplayWidget::ObjectDisplayWidget(QWidget *parent) :
    QTextBrowser(parent)
{
}

void ObjectDisplayWidget::setObjectData(const FieldObjects* newObjectData)
{
    QString displayText(newObjectData->toString(true).c_str());
    this->setText(displayText);
}
