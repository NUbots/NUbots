#ifndef SENSORCALIBRATIONWIDGET_H
#define SENSORCALIBRATIONWIDGET_H

#include <QWidget>
class NUSensorsData;
namespace Ui {
class SensorCalibrationWidget;
}

#include "Infrastructure/SensorCalibration.h"
class Transformer;
class SensorCalibrationSettings
{
public:
    // Location
    float location_x;               //! X position on the field.
    float location_y;               //! Y position on the field.
    float location_orientation;     //! Orientation on the field.

    Vector2<double> m_fov;               //! Field of view of the current camera (hard coded options at the moment.)
    SensorCalibration m_calibration;    //! Sensor calibration

    SensorCalibrationSettings(): location_x(0.f), location_y(0.f), location_orientation(0.f){}
};

class SensorCalibrationWidget : public QWidget
{
    Q_OBJECT
    
public:
    explicit SensorCalibrationWidget(QWidget *parent = 0);
    ~SensorCalibrationWidget();

    /*!
    * @brief Fetches the current calibration setting.
    * @return The current calibration setting of the widget.
    */
    SensorCalibrationSettings* Calibration();

    /*!
    * @brief Sets the current calibration setting.
    * @param calibration The new calibration setting of the widget.
    */
    void setCalibration(SensorCalibrationSettings& calibration){m_calibration = calibration;}

signals:
    /*!
    * @brief Signal indicating that the current calibration settings have changed.
    * @param calibration The new calibration setting of the widget.
    */
    void CalibrationChanged(SensorCalibrationSettings* newCalibration);
public slots:
    /*!
    * @brief Slot used to update the NUSensorData used by the widget.
    * @param new_sensordata The new sensor data.
    */
    void setSensorData(NUSensorsData* new_sensordata);
    /*!
    * @brief Slot used to select a pixel.
    * @param x pixel x-position.
    * @param y pixel y-position.
    */
    void select_pixel(int x, int y);
protected slots:
    /*!
    * @brief Slot used when one of the contols has been changed.
    * The calibration is updated and the new calibration settings emitted.
    */
    void ValueChanged();
private slots:
    /*!
    * @brief Slot used when the load button is clicked.
    * Opens a file select dialog and loads the chosen configuration file.
    */
    void on_loadButton_clicked();

    /*!
    * @brief Slot used when the save button is clicked.
    * Opens a file select dialog and saves the current configuration to the selected file.
    */
    void on_saveButton_clicked();

private:
    Ui::SensorCalibrationWidget *ui;
    SensorCalibrationSettings m_calibration;
    NUSensorsData* m_sensors;
    int selected_x;
    int selected_y;
    Transformer* m_transformer;
    bool m_updating;

};

#endif // SENSORCALIBRATIONWIDGET_H
