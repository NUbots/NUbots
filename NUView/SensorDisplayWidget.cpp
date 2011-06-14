#include "SensorDisplayWidget.h"
#include "Infrastructure/NUSensorsData/NUSensorsData.h"
#include <sstream>

SensorDisplayWidget::SensorDisplayWidget(QWidget *parent) :
    QTextBrowser(parent)
{
}

void SensorDisplayWidget::SetSensorData(NUSensorsData* newSensorData)
{
    std::stringstream data;
    newSensorData->summaryTo(data);
    QString displayText(data.str().c_str());
    this->setText(displayText);
}
