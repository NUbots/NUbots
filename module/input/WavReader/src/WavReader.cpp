#include "WavReader.hpp"

#include <cstdint>
#include <fstream>
#include <iostream>
#include <string>

using std::cin;
using std::cout;
using std::endl;
using std::fstream;
using std::string;

#include "extension/Configuration.hpp"
namespace module::input {

    using extension::Configuration;

    WavReader::WavReader(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)), config{} {

        on<Configuration>("WavReader.yaml").then([this](const Configuration& cfg) {
            // Use configuration here from file WavReader.yaml
            this->log_level = cfg["log_level"].as<NUClear::LogLevel>();
        });
    }

}  // namespace module::input
