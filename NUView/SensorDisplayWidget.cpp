#include "SensorDisplayWidget.h"
#include "Infrastructure/NUSensorsData/NUSensorsData.h"
#include <sstream>
#include <QDebug>
SensorDisplayWidget::SensorDisplayWidget(QWidget *parent) :
    QTextBrowser(parent)
{
}

void SensorDisplayWidget::SetSensorData(NUSensorsData* newSensorData)
{
    qDebug("Sensor Update...");
    qDebug() << "Size: " << newSensorData->size();
    std::stringstream data;
    newSensorData->summaryTo(data);
    QString displayText(data.str().c_str());
    this->setText(displayText);
}
