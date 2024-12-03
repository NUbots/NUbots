#ifndef MODULE_TOOLS_POSITIONMODEL_HPP
#define MODULE_TOOLS_POSITIONMODEL_HPP

#include <nuclear>
#include <filesystem>
#include <fstream>
#include <string>

namespace module::tools {
namespace fs = std::filesystem;
class PositionModel : public NUClear::Reactor {
private:
    /// @brief Stores configuration values
    struct Config {
    } cfg;

    std::ofstream csv_ofs;


public:
    /// @brief Called by the powerplant to build and setup the PositionModel reactor.
    explicit PositionModel(std::unique_ptr<NUClear::Environment> environment);
};

}  // namespace module::tools

#endif  // MODULE_TOOLS_POSITIONMODEL_HPP
