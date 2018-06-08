/*
author Jake Fountain
This code is part of mocap-kinect experiments*/

#include "MocapStream.h"

namespace autocal {

using utility::math::geometry::UnitQuaternion;
using utility::math::matrix::Rotation3D;
using utility::math::matrix::Transform3D;


std::string MocapStream::Frame::toString() {
    std::stringstream os;
    for (auto& x : rigidBodies) {
        os << "rigidBodies[" << int(x.first) << "] = \n" << x.second.pose << std::endl;
    }
    os << "=======================";
    return os.str();
}

std::string MocapStream::toString() {
    std::stringstream os;
    os << "Mocap Stream: " << name() << " || Size: " << size() << std::endl;
    for (auto& x : *stream) {
        os << "(*stream)[" << int(x.first) << "] = \n" << x.second.toString() << std::endl;
    }
    return os.str();
}


MocapStream::Frame MocapStream::createFrame(arma::mat m, const std::set<int>& allowedIDs) {
    Frame f;
    // std::cout << "Loading " << m.n_cols << " rigid bodies" << std::endl;
    // std::cout << m << std::endl;
    for (int n = 0; n < m.n_cols; n++) {
        arma::vec data = m.col(n);

        RigidBody r;

        arma::vec3 pos = data.rows(1, 3);
        Rotation3D rot;
        int start = 4;
        for (int i = 0; i < 3; i++) {
            rot.row(i) = data.rows(start + 3 * i, start + 3 * i + 2).t();
        }
        // Change back to mocap coords from nubots coords (sigh...)
        if (correctForAutocalibrationCoordinateSystem) {
            r.pose.translation() = arma::vec3{-pos[1], pos[2], -pos[0]};
        }
        else {
            r.pose.translation() = pos;
        }

        if (correctForAutocalibrationCoordinateSystem) {
            UnitQuaternion q(rot);
            // Change back to mocap coords from nubots coords (sigh...)
            UnitQuaternion q_(arma::vec4{
                q.kX(),
                -q.kZ(),
                q.kW(),
                -q.kY(),
            });
            // Turn back into rotation
            r.pose.rotation() = Rotation3D(q_);
        }
        else {
            r.pose.rotation() = rot;
        }

        if (LHInput) {
            transformLHtoRH(r.pose);
        }

        // std::cout << "data: " <<  data << std::endl;
        // std::cout << "id:" << int(data[0]) << std::endl;
        // std::cout << "position:" << r.pose.translation() << std::endl;
        // std::cout << "rotation:" << r.pose.rotation() << std::endl;
        if (!r.pose.is_finite()) {
            std::cout << __FILE__ << " - " << __LINE__ << "Warning: nan data not loaded for RB " << int(data[0])
                      << std::endl;
            continue;
        }

        if (allowedIDs.empty()  // empty allowed IDs means allow any
            || allowedIDs.count(int(data[0])) > 0) {
            f.rigidBodies[int(data[0])] = r;
        }
    }
    return f;
}

MocapStream::Frame MocapStream::getFrame(const std::chrono::system_clock::time_point& t) {
    // Get last frame at current time point
    return getFrame(getTimeStamp(t));
}

MocapStream::Frame MocapStream::getInterpolatedFrame(const TimeStamp& t) {
    // Get last frame at current time point
    if (simulated) return getSimulatedFrame(t);

    if (stream->count(t) != 0) {
        return (*stream)[t];
    }
    else {
        const auto ub = stream->lower_bound(t);
        // auto ub = stream->upper_bound(t);
        if (ub == stream->begin()) return ub->second;
        auto lb = ub;
        lb--;
        if (ub == stream->end()) return lb->second;
        float alpha = float(t - lb->first) / float(ub->first - lb->first);
        return Frame::interpolate(lb->second, ub->second, alpha);
    }
}

MocapStream::Frame MocapStream::getFrame(const TimeStamp& t) {
    // Get last frame at current time point
    if (simulated) return getSimulatedFrame(t);

    if (stream->count(t) != 0) {
        return (*stream)[t];
    }
    else {
        auto ub = stream->lower_bound(t);
        if (ub == stream->begin()) return ub->second;
        ub--;
        return ub->second;
    }
}

MocapStream::Frame MocapStream::getParentFrame(const TimeStamp& t) {
    // Get last frame at current time point
    if (parentStream->count(t) != 0) {
        return (*parentStream)[t];
    }
    else {
        auto ub = parentStream->lower_bound(t);
        if (ub == parentStream->begin()) return ub->second;
        ub--;
        return ub->second;
    }
}

TimeStamp MocapStream::getFrameTime(const TimeStamp& t) {
    if (stream->count(t) != 0) {
        return t;
    }
    else {
        return stream->lower_bound(t)->first;
    }
}

std::map<TimeStamp, MocapStream::Frame>::iterator MocapStream::getUpperBoundIter(const TimeStamp& t) {
    // Get last frame at current time point
    return stream->upper_bound(t);
}

std::map<TimeStamp, MocapStream::Frame>::iterator MocapStream::getLowerBoundIter(const TimeStamp& t) {
    // Get last frame at current time point
    return stream->lower_bound(t);
}

bool MocapStream::loadMocapData(std::string folder_path,
                                const TimeStamp& start_time,
                                const std::chrono::system_clock::time_point& end_time,
                                const std::set<int>& allowedIDs) {
    std::cout << "Loading data ..." << std::endl;

    TimeStamp min = start_time;
    TimeStamp max = getTimeStamp(end_time);

    bool success = true;

    DIR* dir = opendir(folder_path.c_str());
    std::cout << "Folder opened ..." << std::endl;
    dirent* file;
    TimeStamp max_loaded = min;
    TimeStamp min_loaded = max;

    while ((file = readdir(dir)) != NULL) {

        std::string filename = file->d_name;
        // std::cout << "Filename = " << filename << std::endl;
        TimeStamp timestamp;
        try {
            timestamp = std::stoll(filename);
        }
        catch (std::invalid_argument) {
            std::cout << "Skipping file " << filename << std::endl;
            continue;
        }
        if (timestamp < max && timestamp > min) {
            arma::mat frame;

            std::stringstream path;
            path << folder_path << "/" << filename;
            success = success && frame.load(path.str(), arma::arma_binary);

            if (success) {
                // Do not store frame if it has no info
                if (frame.n_cols != 0) {
                    (*stream)[timestamp] = createFrame(frame, allowedIDs);
                }
            }
            else {
                continue;
            }

            max_loaded = std::max(max_loaded, timestamp);
            min_loaded = std::min(min_loaded, timestamp);
        }
    }
    if (max != min_loaded && min != max_loaded) {
        float period_sec = float(max_loaded - min_loaded) * 1e-6;
        std::cout << "Loaded data from " << min_loaded << " to " << max_loaded << ". i.e. " << int(period_sec)
                  << " seconds at " << stream->size() / period_sec << "Hz measurement frequency" << std::endl;
    }

    (void) closedir(dir);
    std::cout << "Loading finished " << (success ? "successfully" : "UNSUCCESSFULLY") << std::endl;


    return success;
}

bool MocapStream::setRigidBodyInFrame(const std::chrono::system_clock::time_point& frame_time,
                                      const unsigned int& id,
                                      const Transform3D& pose,
                                      bool correctCoordinateSystem) {
    // Check that the frame doesn't already exist
    TimeStamp t = getTimeStamp(frame_time);
    setRigidBodyInFrame(t, id, pose, correctCoordinateSystem);
}

bool MocapStream::setRigidBodyInFrame(const TimeStamp& frame_time,
                                      const unsigned int& id,
                                      const Transform3D& pose,
                                      bool correctCoordinateSystem) {
    // Check that the frame doesn't already exist
    TimeStamp t = frame_time;
    if (stream->count(t) == 0) {
        (*stream)[t] = Frame();
    }
    (*stream)[t].rigidBodies[id] = RigidBody({pose});
    RigidBody& r                 = (*stream)[t].rigidBodies[id];

    arma::vec3 pos = r.pose.translation();
    Rotation3D rot = r.pose.rotation();
    // Change back to mocap coords from nubots coords (sigh...)
    if (correctCoordinateSystem) {
        UnitQuaternion q(rot);
        // Change back to mocap coords from nubots coords (sigh...)
        UnitQuaternion q_(arma::vec4{
            q.kX(),
            -q.kZ(),
            q.kW(),
            -q.kY(),
        });
        // Turn back into rotation
        r.pose.rotation()    = Rotation3D(q_);
        r.pose.translation() = arma::vec3{-pos[1], pos[2], -pos[0]};
    }

    if (LHInput) {
        transformLHtoRH(r.pose);
    }
}

std::map<MocapStream::RigidBodyID, Transform3D> MocapStream::getCompleteStates(TimeStamp now) {
    std::map<MocapStream::RigidBodyID, Transform3D> states;

    if (stream->size() != 0) {

        Frame latestFrame = getFrame(now);

        for (auto& rb : latestFrame.rigidBodies) {
            auto rbID      = rb.first;
            auto transform = rb.second.pose;

            states[rbID] = transform;
        }
    }

    return states;
}

void MocapStream::transform(utility::math::matrix::Transform3D T) {
    for (auto& frame : *stream) {
        // Loop through and record transformed rigid body poses
        for (auto& rb : frame.second.rigidBodies) {
            Transform3D newPose = T * rb.second.pose;
            rb.second.pose      = newPose;
        }
    }
}

void MocapStream::setupSimulation(const MocapStream& parentStream_, std::map<int, int> simulationIDs_) {
    simulated     = true;
    simulationIDs = simulationIDs_;
    parentStream  = parentStream_.stream;
}

void MocapStream::setSimulationParameters(const SimulationParameters& sim) {
    simulationParameters = sim;
}

MocapStream::Frame MocapStream::getSimulatedFrame(TimeStamp now) {
    MocapStream::Frame frame;

    int lag_milliseconds = simulationParameters.latency_ms;
    now -= lag_milliseconds * 1000;

    if (parentStream->size() != 0) {

        MocapStream::Frame latestFrame = getParentFrame(now);
        for (auto& key : simulationIDs) {
            int artificialID = key.first;
            int derivedID    = key.second;

            if (simWorldTransform.count(key) == 0) {
                simWorldTransform[key] = Transform3D::getRandomU(1, 0.1);
                // simWorldTransform[key] = arma::eye(4,4);
                //  Transform3D({ 0.1040,  -0.0023,  -0.9946,  -0.3540,
                // 								      -0.1147,   0.9933,  -0.0143,  -0.9437,
                // 								       0.9879,   0.1156,   0.1030,   1.2106,
                // 								            0,        0,        0,   1.0000}).t();//transpose because
                // column
                // major
                // reading
                std::cout << "simWorldTransform = \n" << simWorldTransform[key] << std::endl;
            }
            if (simLocalTransform.count(key) == 0) {
                simLocalTransform[key] = Transform3D::getRandomU(1, 0.1);
                // simLocalTransform[key] = simLocalTransform[key].rotateX(M_PI_2);
                std::cout << "simLocalTransform = \n" << simLocalTransform[key] << std::endl;
            }

            // Noise:
            Transform3D localNoise = Transform3D::getRandomN(simulationParameters.noise.angle_stddev,
                                                             simulationParameters.noise.disp_stddev);
            // std::cout << "noise = " << arma::vec4(localNoise * arma::vec4({0,0,0,1})).t() << std::endl;
            Transform3D globalNoise = Transform3D::getRandomN(simulationParameters.noise.angle_stddev,
                                                              simulationParameters.noise.disp_stddev);
            // Transform3D globalNoise = Transform3D::getRandomN(0.310524198 ,0.052928682);

            Transform3D transform = simWorldTransform[key] * latestFrame.rigidBodies[derivedID].pose * globalNoise
                                    * simLocalTransform[key] * localNoise;

            // Debugging
            // std::cout << "transform = " << transform.translation().t() << std::endl;
            // transform.translation() = arma::vec{0,0,-1};

            frame.rigidBodies[artificialID]      = RigidBody();
            frame.rigidBodies[artificialID].pose = transform;
        }
    }

    (*stream)[now] = frame;

    return frame;
}


void MocapStream::transformLHtoRH(Transform3D& T) {
    if (LHInput) {
        Transform3D L = Transform3D::createScale(arma::vec3({-1, 1, 1}));
        T             = L * T * L;  // last L is actually L.i() = L;
    }
}
}  // namespace autocal
