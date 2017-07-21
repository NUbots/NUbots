#include "ImageMask.h"

#include "extension/Configuration.h"

namespace module {
namespace vision {

    using extension::Configuration;

    ImageMask::ImageMask(std::unique_ptr<NUClear::Environment> environment) : Reactor(std::move(environment)) {

        on<Configuration>("ImageMask.yaml").then([this](const Configuration& config) {
            // Use configuration here from file ImageMask.yaml

        });
    }
}  // namespace vision
}  // namespace module
