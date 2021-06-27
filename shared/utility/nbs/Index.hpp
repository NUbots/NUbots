#ifndef UTILITY_NBS_INDEX_HPP
#define UTILITY_NBS_INDEX_HPP

#include <filesystem>
#include <functional>
#include <vector>

namespace utility::nbs {

#pragma pack(push, 1)
    struct IndexItem {
        /// The hash type of the message
        uint64_t type;
        /// The id field of the message if it exists, else 0 (used for things like camera's id)
        uint32_t id;
        /// The timestamp from the protcol buffer if it exists, else the timestamp from the NBS file
        uint64_t timestamp;
        /// The offset of the message from the start of the file
        uint64_t offset;
        /// The length of the packet (including the header)
        uint32_t length;
        /// When we are opening multiple nbs files at once, this indicates which file it is from
        uint32_t fileno;

        /**
         * @brief Compares this index item with the other based on timestamp
         *
         * @param them The other index item to compare to
         *
         * @return if our timestamp is less than the timestamp of the other index
         */
        bool operator<(const IndexItem& them) const;
    };
#pragma pack(pop)

    class Index {
    public:
        /**
         * @brief Construct a new Index object based on the provided list of paths
         *
         * @param paths     the paths to the nbs files we are getting the index for
         * @param progress  the callback function to give progress updates for building of the index
         */
        Index(const std::vector<std::filesystem::path>& paths,
              const std::function<void(const std::filesystem::path&, const uint64_t&, const uint64_t&)>& progress = {});

        /**
         * @brief Begin iterator to the IndexItems held in this index
         *
         * @return the start iterator for the IndexItems
         */
        [[nodiscard]] std::vector<IndexItem>::const_iterator begin() const;
        /**
         * @brief End iterator to the IndexItems held in this index
         *
         * @return the one past the end iterator for the IndexItems
         */
        [[nodiscard]] std::vector<IndexItem>::const_iterator end() const;

    private:
        /// The indexes that are held in this index
        std::vector<IndexItem> idx;
    };

}  // namespace utility::nbs

#endif  // UTILITY_NBS_INDEX_HPP
