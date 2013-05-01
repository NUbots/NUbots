#include "SensorCalibrationWidget.h"
#include "ui_SensorCalibrationWidget.h"

#include <QDebug>

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
}

SensorCalibrationWidget::~SensorCalibrationWidget()
{
    delete ui;
}

SensorCalibration* SensorCalibrationWidget::Calibration()
{
    return &m_calibration;
}

void SensorCalibrationWidget::ValueChanged()
{
    // Location
    m_calibration.location_x = ui->doubleSpinBox_field_x->value();
    m_calibration.location_y = ui->doubleSpinBox_field_y->value();
    m_calibration.location_orientation = ui->doubleSpinBox_field_ori->value();

    // Body offsets.
    m_calibration.body_roll_offset = ui->doubleSpinBox_body_roll->value();
    m_calibration.body_pitch_offset = ui->doubleSpinBox_body_pitch->value();

    // Camera offsets.
    m_calibration.camera_roll_offset = ui->doubleSpinBox_camera_roll->value();
    m_calibration.camera_pitch_offset = ui->doubleSpinBox_camera_pitch->value();
    m_calibration.camera_yaw_offset = ui->doubleSpinBox_camera_yaw->value();

    emit CalibrationChanged(&m_calibration);
}
