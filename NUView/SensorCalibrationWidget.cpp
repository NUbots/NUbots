#include "SensorCalibrationWidget.h"
#include "ui_SensorCalibrationWidget.h"
#include "Infrastructure/NUSensorsData/NUSensorsData.h"
#include "Tools/Math/Matrix.h"
#include "Tools/Math/General.h"
#include <QDebug>
#include <iostream>
#include "Vision/VisionTools/transformer.h"
#include "Kinematics/Kinematics.h"
#include "Tools/Math/TransformMatrices.h"
#include <QFileDialog>
#include <QDir>

SensorCalibrationWidget::SensorCalibrationWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::SensorCalibrationWidget)
{
    ui->setupUi(this);
    connect(ui->doubleSpinBox_field_x,SIGNAL(valueChanged(double)), this, SLOT(ValueChanged()));
    connect(ui->doubleSpinBox_field_y,SIGNAL(valueChanged(double)), this, SLOT(ValueChanged()));
    connect(ui->doubleSpinBox_field_ori,SIGNAL(valueChanged(double)), this, SLOT(ValueChanged()));

    connect(ui->doubleSpinBox_body_roll,SIGNAL(valueChanged(double)), this, SLOT(ValueChanged()));
    connect(ui->doubleSpinBox_body_pitch,SIGNAL(valueChanged(double)), this, SLOT(ValueChanged()));

    connect(ui->doubleSpinBox_camera_roll,SIGNAL(valueChanged(double)), this, SLOT(ValueChanged()));
    connect(ui->doubleSpinBox_camera_pitch,SIGNAL(valueChanged(double)), this, SLOT(ValueChanged()));
    connect(ui->doubleSpinBox_camera_yaw,SIGNAL(valueChanged(double)), this, SLOT(ValueChanged()));
    connect(ui->comboBox_camera_select,SIGNAL(currentIndexChanged(int)), this, SLOT(ValueChanged()));
    m_sensors = NULL;
    selected_x = -1;
    selected_y = -1;

    m_transformer = new Transformer();
    m_transformer->setCamParams(Vector2<double>(640,480), Vector2<double>(1.0472, 0.7854));
    m_transformer->setCalibration(m_calibration.m_calibration);

    m_calibration.m_calibration.m_neck_position_offset = Vector3<double>(0.0, 0.0, 5.05);

    m_updating = false;
}

SensorCalibrationWidget::~SensorCalibrationWidget()
{
    if(m_transformer) delete m_transformer;
    delete ui;
}

SensorCalibrationSettings* SensorCalibrationWidget::Calibration()
{
    return &m_calibration;
}

void SensorCalibrationWidget::ValueChanged()
{
    if(m_updating) return;

    // Location
    m_calibration.location_x = ui->doubleSpinBox_field_x->value();
    m_calibration.location_y = ui->doubleSpinBox_field_y->value();
    m_calibration.location_orientation = ui->doubleSpinBox_field_ori->value();

    // Body offsets.
    m_calibration.m_calibration.m_body_angle_offset.x = ui->doubleSpinBox_body_roll->value();
    m_calibration.m_calibration.m_body_angle_offset.y = ui->doubleSpinBox_body_pitch->value();

    // Camera offsets.
    m_calibration.m_calibration.m_camera_angle_offset.x = ui->doubleSpinBox_camera_roll->value();
    m_calibration.m_calibration.m_camera_angle_offset.y = ui->doubleSpinBox_camera_pitch->value();
    m_calibration.m_calibration.m_camera_angle_offset.z = ui->doubleSpinBox_camera_yaw->value();

    if(ui->comboBox_camera_select->currentText().toLower() == "darwin")
    {
        // From CameraSpecs.cfg file.
        m_calibration.m_fov.x = 62.8f;
        m_calibration.m_fov.y = 48.8f;
        Vector2<double> rad_fov(mathGeneral::deg2rad(m_calibration.m_fov.x), mathGeneral::deg2rad(m_calibration.m_fov.y));
        m_transformer->setCamParams(Vector2<double>(640,480), rad_fov);
    }
    else if(ui->comboBox_camera_select->currentText().toLower() == "darwinwebots")
    {
        // From webots model
        m_calibration.m_fov.x = 60.f;
        m_calibration.m_fov.y = 45.f;
        Vector2<double> rad_fov(mathGeneral::deg2rad(m_calibration.m_fov.x), mathGeneral::deg2rad(m_calibration.m_fov.y));
        m_transformer->setCamParams(Vector2<double>(640,480), rad_fov);
    }
    else
    {
        // default
        m_calibration.m_fov.x = 60.f;
        m_calibration.m_fov.y = 45.f;
        Vector2<double> rad_fov(mathGeneral::deg2rad(m_calibration.m_fov.x), mathGeneral::deg2rad(m_calibration.m_fov.y));
        m_transformer->setCamParams(Vector2<double>(640,480), rad_fov);
    }

    emit CalibrationChanged(&m_calibration);

    if(selected_x >= 0 and selected_y >= 0)
        select_pixel(selected_x, selected_y);

    m_transformer->setCalibration(m_calibration.m_calibration);
}

void SensorCalibrationWidget::setSensorData(NUSensorsData* new_sensordata)
{
    m_sensors = new_sensordata;

    if(m_sensors)
    {
        float head_pitch(0.f), head_yaw(0.f);
        Vector3<double> neck_position;
        vector<float> orientation(3,0.f), left, right;
        m_sensors->getPosition(NUSensorsData::HeadPitch, head_pitch);
        m_sensors->getPosition(NUSensorsData::HeadYaw, head_yaw);
        m_sensors->getOrientation(orientation);

        if(m_sensors->get(NUSensorsData::LLegTransform, left) and m_sensors->get(NUSensorsData::RLegTransform, right))
        {
            neck_position = Kinematics::CalculateNeckPosition(Matrix4x4fromVector(left), Matrix4x4fromVector(right), m_calibration.m_calibration.m_neck_position_offset);
        }
        else
        {
            // Default in case kinemtaics not available. Base height of darwin.
            neck_position = Vector3<double>(0.0, 0.0, 39.22);
        }
        m_transformer->setSensors(head_pitch, head_yaw, orientation.at(0), orientation.at(1), neck_position);
    }
}

void SensorCalibrationWidget::select_pixel(int x, int y)
{
    Vector3<double> result = m_transformer->distanceToPoint(Vector2<double>(x, y));

    double flatDist = result.x * cos(result.z);
    double x_pos = flatDist * cos(result.y);
    double y_pos = flatDist * sin(result.y);
    double z_pos = result.x * sin(result.z);

    QString message = "Pixel: (%1, %2)\nSpherical Position: (%3, %4, %5)\nCart Position: (%6, %7, %8)";
    ui->textOutput->setText(message.arg(x).arg(y).arg(result.x,0,'g',3).arg(result.y,0,'g',3).arg(result.z,0,'g',3).arg(x_pos,0,'g',3).arg(y_pos,0,'g',3).arg(z_pos,0,'g',3));
}


void SensorCalibrationWidget::on_loadButton_clicked()
{
    QString init_path = QDir::homePath() + QDir::separator() + "*";
    QString filename = QFileDialog::getOpenFileName(this, "Open configuration file", init_path, "Configuration files (*.cfg);;All Files(*.*)");

    if(!filename.isEmpty())
    {
        m_updating = true;  // Flag to stop reading of changed values.

        SensorCalibration calibration(filename.toStdString());

        // Body offsets.
        ui->doubleSpinBox_body_roll->setValue(calibration.m_body_angle_offset.x);
        ui->doubleSpinBox_body_pitch->setValue(calibration.m_body_angle_offset.y);

        // Camera offsets.
        ui->doubleSpinBox_camera_roll->setValue(calibration.m_camera_angle_offset.x);
        ui->doubleSpinBox_camera_pitch->setValue(calibration.m_camera_angle_offset.y);
        ui->doubleSpinBox_camera_yaw->setValue(calibration.m_camera_angle_offset.z);

        m_calibration.m_calibration = calibration;

        m_updating = false;
        update();
    }
}

void SensorCalibrationWidget::on_saveButton_clicked()
{
    QString init_path = QDir::homePath() + QDir::separator() + "SensorCalibration.cfg";
    QString filename = QFileDialog::getSaveFileName(this, "Open configuration file", init_path, "Configuration files (*.cfg);;All Files(*.*)");
    if(!filename.isEmpty())
    {
        m_calibration.m_calibration.WriteSettings(filename.toStdString());
    }
}
