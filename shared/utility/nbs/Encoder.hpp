#ifndef UTILITY_NBS_ENCODER_HPP
#define UTILITY_NBS_ENCODER_HPP

#include <filesystem>
#include <fstream>
#include <nuclear>
#include <zstr.hpp>

namespace utility::nbs {

    class Encoder {
    private:
        /// The size of the radiation symbol at the start of each packet
        static constexpr int HEADER_SIZE = 3;
        /// The file we are outputting to currently
        std::ofstream output_file{};
        /// The file we are outputting our index to currently
        zstr::ofstream index_file{};
        /// The number of bytes written to the nbs file
        uint64_t bytes_written = 0;

    public:
        Encoder() = default;

        Encoder(std::filesystem::path path);

        Encoder(const std::filesystem::path& path, const std::filesystem::path& index_path);

        /// @brief Write a neutron to the file and returns the number of bytes written
        template <typename T>
        int write(const T& value, const NUClear::clock::time_point& timestamp) {
            return write(timestamp,
                         get_timestamp(value, timestamp),
                         NUClear::util::serialise::Serialise<T>::hash(),
                         get_id(value),
                         NUClear::util::serialise::Serialise<T>::serialise(value));
        }

        /**
         * @brief Writes an entry to the file
         *
         * @param timestamp the timestamp the message was originally emited at
         * @param message_timestamp the timestamp inside the message
         * @param hash The type hash of the message
         * @param id The id of the message
         * @param data The raw message data to be written
         */
        int write(const NUClear::clock::time_point& timestamp,
                  const uint64_t& message_timestamp,
                  const uint64_t& hash,
                  const uint32_t& id,
                  const std::vector<char>& data);

        /// @brief Gets the number of bytes written
        const uint64_t& get_bytes_written() const;

        /// @brief closes the internal nbs file
        void close();

        void open(const std::filesystem::path& path);

        /// @brief checks that the nbs file is open
        bool is_open() const;
    };

}  // namespace utility::nbs

#endif  // UTILITY_NBS_ENCODER_HPP
