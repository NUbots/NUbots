#include "MocapRecorder.h"

using utility::math::matrix::Rotation3D;
using utility::math::matrix::Transform3D;

bool MocapRecorder::saveFrame(const std::vector<Transform3D>& poses) {
    arma::mat rigidBodies(13, poses.size());
    int i = 0;
    for (const auto& pose : poses) {

        int id               = i;
        float x              = pose.translation()[0];
        float y              = pose.translation()[1];
        float z              = pose.translation()[2];
        const Rotation3D& r  = pose.rotation();
        rigidBodies.col(i++) = arma::vec({double(id),
                                          x,
                                          y,
                                          z,
                                          r.row(0)[0],
                                          r.row(0)[1],
                                          r.row(0)[2],
                                          r.row(1)[0],
                                          r.row(1)[1],
                                          r.row(1)[2],
                                          r.row(2)[0],
                                          r.row(2)[1],
                                          r.row(2)[2]});
    }
    auto now = std::chrono::system_clock::now();
    std::stringstream filename;
    filename << folder << "/"
             << std::to_string(std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()).count());
    std::cout << "Saving MotionCapture data to " << filename.str() << std::endl;
    return rigidBodies.save(filename.str());
}
