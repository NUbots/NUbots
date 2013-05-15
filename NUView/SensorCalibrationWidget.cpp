#include "SensorCalibrationWidget.h"
#include "ui_SensorCalibrationWidget.h"
#include "Infrastructure/NUSensorsData/NUSensorsData.h"
#include "Tools/Math/Matrix.h"
#include "Tools/Math/General.h"
#include <QDebug>
#include <iostream>

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
    m_sensors = NULL;
    selected_x = -1;
    selected_y = -1;
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

    if(selected_x >= 0 and selected_y >= 0)
        select_pixel(selected_x, selected_y);
}

void SensorCalibrationWidget::setSensorData(NUSensorsData* new_sensordata)
{
    m_sensors = new_sensordata;
}

void SensorCalibrationWidget::select_pixel(int x, int y)
{
    if(m_sensors)
    {
        const unsigned int image_width = 640;
        const unsigned int image_height = 480;
        const unsigned int diagnal_dist = sqrt(image_width*image_width + image_height*image_height);

//        const float fov_diag = 1.0472;
//        const float f = 0.5f * diagnal_dist / tan(0.5f * fov_diag);
//        const float fov_horizontal = 2 * atan(0.5f * image_width / f);
//        const float fov_vertical = 2 * atan(0.5f * image_height / f);

        const float fov_horizontal = 1.0472;
        const float fov_vertical = fov_horizontal * image_height / image_width;

        const float f = 0.5f * image_width / tan(0.5f * fov_horizontal);

        std::cout << "f: " << f << std::endl;
        std::cout << "fov_horizontal: " << fov_horizontal << std::endl;
        std::cout << "fov_vertical: " << fov_vertical << std::endl;

        // Varibles for orientations.
        float camera_roll = m_calibration.camera_roll_offset;
        float camera_pitch = m_calibration.camera_pitch_offset;
        float camera_yaw = m_calibration.camera_yaw_offset;

        float head_roll = 0.f;
        float head_pitch = 0.f;
        float head_yaw = 0.f;

        float body_roll = m_calibration.body_roll_offset;
        float body_pitch = m_calibration.body_pitch_offset;
        float body_yaw = 0.f;

        // Camera offset values. (These should not be hard-coded since they differ per robot type)
        const float camera_x_offset = 3.32f;
        const float camera_y_offset = 0.f;
        const float camera_z_offset = 3.44f;

        // Body offset values. (These should not be hard-coded since they differ per robot type)
        const float body_x_offset = 0.f;
        const float body_y_offset = 0.f;
        const float body_z_offset = 39.22f;

        // Get the sensor information.
        float temp = 0.f;   // temp variable for fetching sensor values.
        std::vector<float> temp_vec;
        if(m_sensors)  // Check if sensor data is available
        {
            // Head pitch
            if(m_sensors->getPosition(NUSensorsData::HeadPitch, temp))
            {
                head_pitch += temp;
            }
            // Head yaw
            if(m_sensors->getPosition(NUSensorsData::HeadYaw, temp))
            {
                head_yaw += temp;
            }
            // Head roll - Our robots do not have thus at the moment.
            if(m_sensors->getPosition(NUSensorsData::HeadRoll, temp))
            {
                head_roll += temp;
            }
    //        if(m_sensors->getOrientation(temp_vec))
    //        {
    //            body_roll += temp_vec[0];
    //            body_pitch += temp_vec[1];
    //            body_yaw += temp_vec[2];

    //            std::cout << body_roll << ", " << body_pitch << ", " << body_yaw << std::endl;
    //        }
            // Camera Height
    //        if(SensorData->getCameraHeight(temp))
    //        {
    //            std::cout << "Camera Height: " << temp << std::endl;
    //        }
        }

        ///
        // Testing area
        ///
        Matrix camOffsetVec(3,1,false);
        camOffsetVec[0][0] = camera_x_offset;
        camOffsetVec[1][0] = camera_y_offset;
        camOffsetVec[2][0] = camera_z_offset;

        std::cout << "camOffsetVec" << std::endl << camOffsetVec << std::endl;

        Matrix neckV(3,1,false);
        neckV[0][0] = body_x_offset;
        neckV[1][0] = body_y_offset;
        neckV[2][0] = body_z_offset;

        std::cout << "neckV" << std::endl << neckV << std::endl;

        double sinA;
        double cosA;

        Matrix body_roll_rot = Matrix(3,3,true);
        sinA = sin(body_roll);
        cosA = cos(body_roll);
        body_roll_rot[1][1] = cosA;
        body_roll_rot[1][2] = -sinA;
        body_roll_rot[2][1] = sinA;
        body_roll_rot[2][2] = cosA;

        Matrix camera_roll_rot = Matrix(3,3,true);
        sinA = sin(camera_roll);
        cosA = cos(camera_roll);
        camera_roll_rot[1][1] = cosA;
        camera_roll_rot[1][2] = -sinA;
        camera_roll_rot[2][1] = sinA;
        camera_roll_rot[2][2] = cosA;

        Matrix body_pitch_rot = Matrix(3,3,true);
        sinA = sin(body_pitch);
        cosA = cos(body_pitch);
        body_pitch_rot[0][0] = cosA;
        body_pitch_rot[0][2] = sinA;
        body_pitch_rot[2][0] = -sinA;
        body_pitch_rot[2][2] = cosA;

        Matrix camera_pitch_rot = Matrix(3,3,true);
        sinA = sin(camera_pitch);
        cosA = cos(camera_pitch);
        camera_pitch_rot[0][0] = cosA;
        camera_pitch_rot[0][2] = sinA;
        camera_pitch_rot[2][0] = -sinA;
        camera_pitch_rot[2][2] = cosA;

        Matrix head_pitch_rot = Matrix(3,3,true);
        sinA = sin(head_pitch);
        cosA = cos(head_pitch);
        head_pitch_rot[0][0] = cosA;
        head_pitch_rot[0][2] = sinA;
        head_pitch_rot[2][0] = -sinA;
        head_pitch_rot[2][2] = cosA;

        Matrix head_yaw_rot = Matrix(3,3,true);
        sinA = sin(head_yaw);
        cosA = cos(head_yaw);
        head_yaw_rot[0][0] = cosA;
        head_yaw_rot[0][1] = -sinA;
        head_yaw_rot[1][0] = sinA;
        head_yaw_rot[1][1] = cosA;

        Matrix camera_yaw_rot = Matrix(3,3,true);
        sinA = sin(camera_yaw);
        cosA = cos(camera_yaw);
        camera_yaw_rot[0][0] = cosA;
        camera_yaw_rot[0][1] = -sinA;
        camera_yaw_rot[1][0] = sinA;
        camera_yaw_rot[1][1] = cosA;

        Matrix headV2RobotRotation = body_roll_rot * body_pitch_rot * head_yaw_rot * head_pitch_rot;
        std::cout << "headV2RobotRotation" << std::endl << headV2RobotRotation << std::endl;

        Matrix camVector = (headV2RobotRotation * camOffsetVec) + neckV;
        std::cout << "camVector" << std::endl << camVector << std::endl;
        Matrix camV2RobotRotation = headV2RobotRotation * camera_pitch_rot * camera_roll_rot * camera_yaw_rot;
        std::cout << "camV2RobotRotation" << std::endl << camV2RobotRotation << std::endl;

        int cx = x;
        int cy = y;
        float objectHeight = 0;

        Matrix vcam(3,1,false);
        vcam[0][0] = f;
        vcam[1][0] = image_width / 2.f - cx;
        vcam[2][0] = image_height / 2.f - cy;
        std::cout << "vcam" << std::endl << vcam << std::endl;

        Matrix roboVdir = camV2RobotRotation * vcam;
        std::cout << "roboVdir" << std::endl << roboVdir << std::endl;
        double alpha = (objectHeight - camVector[2][0]) / roboVdir[2][0];
        std::cout << "alpha" << std::endl << alpha << std::endl;

        Matrix v2FieldPoint = alpha * roboVdir + camVector;

        std::cout << "v2FieldPoint" << std::endl << v2FieldPoint << std::endl;

        double distance = sqrt(pow(v2FieldPoint[0][0], 2) + pow(v2FieldPoint[1][0], 2));
        double bearing = atan2((double)v2FieldPoint[1][0], (double)v2FieldPoint[0][0]);
        double elevation = atan2(objectHeight - (double)camVector[2][0], (double)distance);

        std::cout << "Pixel: (" << x << ", " << y << ")" << std::endl;

        std::cout << "Distance: " << distance << std::endl;
        std::cout << "Bearing: " << bearing << std::endl;
        std::cout << "Elevation: " << elevation << std::endl;
    }

}

