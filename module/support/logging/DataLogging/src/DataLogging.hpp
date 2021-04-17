#ifndef MODULE_SUPPORT_LOGGING_DATALOGGING_HPP
#define MODULE_SUPPORT_LOGGING_DATALOGGING_HPP

#include <filesystem>
#include <fstream>
#include <nuclear>
#include <zstr.hpp>

#include "utility/type_traits/has_id.hpp"
#include "utility/type_traits/has_timestamp.hpp"

namespace module::support::logging {

/// @brief Returns the timestamp field of data or, if timestamp does not exist, it returns original, both converted
/// to uint64_t
template <typename T>
std::enable_if_t<!utility::type_traits::has_timestamp<T>::value, uint64_t> get_timestamp(
    const NUClear::clock::time_point& original,
    const T& /*data*/) {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(original.time_since_epoch()).count();
}

template <typename T>
std::enable_if_t<utility::type_traits::has_timestamp<T>::value, uint64_t> get_timestamp(
    const NUClear::clock::time_point& /*original*/,
    const T& data) {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(data.timestamp.time_since_epoch()).count();
}

/// @brief Returns the id field of data or, if id does not exist, 0
template <typename T>
std::enable_if_t<!utility::type_traits::has_id<T>::value, uint32_t> get_id(const T& /*data*/) {
    return 0;
}

template <typename T>
std::enable_if_t<utility::type_traits::has_id<T>::value, uint32_t> get_id(const T& data) {
    return data.id;
}

class DataLogging : public NUClear::Reactor {
public:
    struct DataLog {
        /// The timestamp this message was emitted at
        NUClear::clock::time_point timestamp;
        /// The timestamp inside the message or, if message does not exist, the emit timestamp
        uint64_t message_timestamp{};
        /// The type hash generated via NUClear serialise
        uint64_t hash{};
        /// The id field of the message or, if id does not exist, 0.
        uint32_t id = 0;
        /// The raw data we will be writing
        std::vector<char> data;

        DataLog() : timestamp(NUClear::clock::now()) {}
    };

    /// Activate a recorder for a specific type and return the ReactionHandle associated with it
    ReactionHandle activate_recorder(const std::string& name);

    /// @brief Called by the powerplant to build and setup the DataLogging reactor.
    explicit DataLogging(std::unique_ptr<NUClear::Environment> environment);

private:
    struct {
        struct {
            /// The directory that we are saving the file to
            std::filesystem::path directory;
            /// The name of the binary (role) that we are running
            std::string binary;
            /// The threshold of bytes where after this we split the file
            uint64_t split_size{};
        } output;
    } config;

    /// The number of bytes written to the file so far
    uint64_t bytes_written;
    /// The file we are outputting to currently
    std::ofstream output_file;
    std::filesystem::path output_file_path{};
    /// The file we are outputting our index to currently
    /// Using a unique pointer as zstr::ofstream does not have a default constructor
    std::unique_ptr<zstr::ofstream> index_file;
    std::filesystem::path index_file_path{};
    /// The reaction handles for types we have currently bound that we are recording
    std::map<std::string, ReactionHandle> handles;
};

}  // namespace module::support::logging

#endif  // MODULE_SUPPORT_LOGGING_DATALOGGING_HPP
