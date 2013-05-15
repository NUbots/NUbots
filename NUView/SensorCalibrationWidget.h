#ifndef SENSORCALIBRATIONWIDGET_H
#define SENSORCALIBRATIONWIDGET_H

#include <QWidget>
class NUSensorsData;
namespace Ui {
class SensorCalibrationWidget;
}

class SensorCalibration
{
public:
    // Location
    float location_x;
    float location_y;
    float location_orientation;

    // Body offsets.
    float body_roll_offset;
    float body_pitch_offset;

    // Camera Offsets
    float camera_roll_offset;
    float camera_pitch_offset;
    float camera_yaw_offset;
    SensorCalibration(): location_x(0.f), location_y(0.f), location_orientation(0.f), body_roll_offset(0.f), body_pitch_offset(0.f),
        camera_roll_offset(0.f), camera_pitch_offset(0.f), camera_yaw_offset(0.f){}
};

class SensorCalibrationWidget : public QWidget
{
    Q_OBJECT
    
public:
    explicit SensorCalibrationWidget(QWidget *parent = 0);
    SensorCalibration* Calibration();
    void setCalibration(SensorCalibration& calibration){m_calibration = calibration;}
    ~SensorCalibrationWidget();

signals:
    void CalibrationChanged(SensorCalibration* newCalibration);
public slots:
    void setSensorData(NUSensorsData* new_sensordata);
    void select_pixel(int x, int y);
protected slots:
    void ValueChanged();
private:
    Ui::SensorCalibrationWidget *ui;
    SensorCalibration m_calibration;
    NUSensorsData* m_sensors;
    int selected_x;
    int selected_y;

};

#endif // SENSORCALIBRATIONWIDGET_H
