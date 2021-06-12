
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <array>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <numeric>
#include <vector>

#include "build.hpp"
#include "kalman.hpp"

int main() {
    std::vector<Eigen::Vector3d> gyro_readings{};
    std::vector<Eigen::Vector3d> acc_readings{};
    std::vector<Eigen::Quaternion<double>> quaternions{};

    char comma = 0;
    std::ifstream ifs("../gyroscope.csv");
    while (ifs.good()) {
        Eigen::Vector3d gyro;
        ifs >> gyro[0] >> comma >> gyro[1] >> comma >> gyro[2];
        if (ifs.good()) {
            gyro_readings.emplace_back(gyro);
        }
    }
    ifs.close();
    ifs.open("../accelerometer.csv");
    while (ifs.good()) {
        Eigen::Vector3d acc;
        ifs >> acc[0] >> comma >> acc[1] >> comma >> acc[2];
        if (ifs.good()) {
            acc_readings.emplace_back(acc);
        }
    }
    ifs.close();
    ifs.open("../quaternion.csv");
    while (ifs.good()) {
        Eigen::Quaternion<double> quat{};
        ifs >> quat.w() >> comma >> quat.x() >> comma >> quat.y() >> comma >> quat.z();
        if (ifs.good()) {
            quaternions.emplace_back(quat);
        }
    }
    ifs.close();


    std::cout << "Found " << gyro_readings.size() << " gyroscope readings" << std::endl;
    std::cout << "Found " << acc_readings.size() << " accelerometer readings" << std::endl;
    std::cout << "Found " << quaternions.size() << " ground-truth quaternions" << std::endl;

    struct filter::tasks::SV_6DOF_GY_KALMAN filter {};
    // TODO clarify coord system - WIN8 has bad perf. NED/ANDROID good perf??
    static constexpr int COORDINATE_SYSTEM = NED;
    static constexpr int SAMPLE_RATE       = 200;
    static constexpr int DECIMATION_FACTOR = 1;

    // Initialise the filter
    filter::kalman::fInit_6DOF_GY_KALMAN(filter);

    // The output quaternions
    std::vector<Eigen::Quaternion<double>> orientations{};

    for (int t = 0; t < int(quaternions.size()); ++t) {
        auto acc_reading  = acc_readings[t];
        auto gyro_reading = gyro_readings[t];
        // The data is expected to be in deg,
        // but converting it doesn't change the result??
        gyro_reading[0] = gyro_reading[0] * M_PI / 180.0;
        gyro_reading[1] = gyro_reading[1] * M_PI / 180.0;
        gyro_reading[2] = gyro_reading[2] * M_PI / 180.0;

        auto quat = filter::kalman::fRun_6DOF_GY_KALMAN(filter, acc_reading, gyro_reading, COORDINATE_SYSTEM);
        orientations.emplace_back(quat);
    }

    std::vector<double> errors;
    for (int t = 0; t < int(quaternions.size()); ++t) {
        const auto p     = orientations[t];
        const auto q     = quaternions[t];
        const double dot = p.dot(q);

        errors.emplace_back(std::acos(double(2) * dot * dot - 1.0f));
        // errors.emplace_back(2 * std::acos(std::fabs(dot)));
    }

    std::cout << "Calculating error over " << orientations.size() << " orientation predictions" << std::endl;
    std::cout << "Average Angular Error: "
              << std::accumulate(errors.begin(), errors.end(), 0.0f) / double(errors.size()) << std::endl;

    return 0;
}