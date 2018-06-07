/*
author Jake Fountain
This code is part of mocap-kinect experiments*/

#include "MocapRecording.h"

namespace autocal {

using utility::math::geometry::UnitQuaternion;
using utility::math::matrix::Rotation3D;
using utility::math::matrix::Transform3D;

void MocapRecording::addMeasurement(const std::string& name,
                                    const TimeStamp& timeStamp,
                                    const MocapStream::RigidBodyID& rigidBodyId,
                                    const Transform3D& pose,
                                    bool correctCoordinateSystem) {
    // Create a new stream if one with this name doesnt exist
    if (streams.count(name) == 0) {
        std::cout << "Initialising mocap stream: " << name << std::endl;
        streams[name] = MocapStream(name, false);
    }

    if (performStats) addStats(name, rigidBodyId, pose);
    // Set the data
    streams[name].setRigidBodyInFrame(timeStamp, rigidBodyId, pose, correctCoordinateSystem);
}

void MocapRecording::markStartOfStreams(TimeStamp now) {
    for (auto& stream : streams) {
        stream.second.markStart(now);
    }
}

void MocapRecording::addStats(const std::string& name,
                              const MocapStream::RigidBodyID& rigidBodyId,
                              const Transform3D& pose) {
    if (stats.count(name) == 0) {
        stats[name] = StreamStats();
    }
    if (stats[name].count(rigidBodyId) == 0) {
        stats[name][rigidBodyId] = arma::running_stat_vec<arma::vec>(true);
    }

    // Rotation measurement
    // TODO: try axis angle
    Rotation3D rot = pose.rotation();
    UnitQuaternion q(rot);
    float rotNorm     = Rotation3D::norm(rot);
    float quatAngle   = q.getAngle();
    arma::vec rotMeas = {rotNorm, quatAngle};

    // Position measurement
    arma::vec3 pos = pose.translation();

    arma::vec measurement = arma::join_cols(rotMeas, pos);
    stats[name][rigidBodyId](measurement);
}
}  // namespace autocal
