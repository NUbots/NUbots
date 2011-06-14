#ifndef SENSORDISPLAYWIDGET_H
#define SENSORDISPLAYWIDGET_H

#include <QWidget>
#include <QTextBrowser>
class NUSensorsData;

class SensorDisplayWidget : public QTextBrowser
{
Q_OBJECT
public:
    explicit SensorDisplayWidget(QWidget *parent = 0);

signals:

public slots:
    void SetSensorData(NUSensorsData* newSensorData);
};

#endif // SENSORDISPLAYWIDGET_H
