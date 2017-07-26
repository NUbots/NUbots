#ifndef MODULE_INPUT_IMAGEDATA_H
#define MODULE_INPUT_IMAGEDATA_H

#include <Eigen/Core>
#include <string>

namespace module {
namespace input {

    struct ImageData {
        uint32_t format;
        Eigen::Matrix<unsigned int, 2, 1, Eigen::DontAlign> dimensions;
        std::vector<uint8_t> data;
        uint32_t camera_id;
        std::string serial_number;
        NUClear::clock::time_point timestamp;
        bool isLeft;
    };

}  // namespace input
}  // namespace module

#endif  // MODULE_INPUT_IMAGEDATA_H
