/*
 * MIT License
 *
 * Copyright (c) 2024 NUbots
 *
 * This file is part of the NUbots codebase.
 * See https://github.com/NUbots/NUbots for further info.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#ifndef UTILITY_NBS_INDEX_HPP
#define UTILITY_NBS_INDEX_HPP

#include <array>
#include <cmath>
#include <filesystem>
#include <fmt/format.h>
#include <functional>
#include <initializer_list>
#include <map>
#include <numeric>
#include <span>
#include <utility>
#include <vector>
#include <zstr.hpp>

#include "IndexStorage.hpp"

#include "utility/support/ProgressBar.hpp"
#include "utility/support/enumerate.hpp"
#include "utility/support/si_unit.hpp"
#include "utility/type_traits/range_constructible.hpp"

namespace utility::nbs {

    void build_index(const std::filesystem::path& nbs_path,
                     const std::filesystem::path& idx_path,
                     const bool& show_progress);

    class Index {
    public:
        enum class Ordering {
            /// Don't sort the nbs files, just load the indexes in order
            NONE,
            /// Time ordering orders the messages by their timestamp
            TIME,
            /// Transmission order splits the messages into subtypes and orders them such that increasing transmission
            /// times result in increasing message density (increased resolution)
            /// This ordering is calculated as the following, for each type/subtype combination, the data is treated as
            /// a maxheap based on time. This data is then interleaved so that for each message type/subtype the amount
            /// of bytes of data is kept equal as transfer increases. Eventually some of the message types/subtypes will
            /// be exhausted using this and the remaining ones will transfer. As an example, sensor packets are much
            /// smaller than GPS packets, so many of those will be in the initial order until they run out and only
            /// images remain.
            TRANSMISSION
        };

        /**
         * @brief Default constructor for the Index class.
         */
        Index() = default;

        /**
         * @brief Construct a new Index object based on the provided list of paths
         *
         * @param paths         the paths to the nbs files we are getting the index for
         * @param show_progress if the progress of loading the index should be printed
         */
        template <typename T>
        Index(const T& paths,
              const Ordering& order      = Ordering::TIME,
              const bool& show_progress  = false,
              const bool& big_index_mode = false) requires
            utility::type_traits::range_constructible<T, std::filesystem::path> : idx(big_index_mode) {

            // Get the total size of all the nbs files that we are loading
            int64_t total_data = std::accumulate(std::begin(paths),
                                                 std::end(paths),
                                                 int64_t(0),
                                                 [](const int64_t& total, const std::filesystem::path& p) {
                                                     return total + std::filesystem::file_size(p);
                                                 });

            int64_t loaded_data = 0;
            std::optional<utility::support::ProgressBar> load_progress;
            if (show_progress) {
                load_progress = utility::support::ProgressBar();
            }

            for (auto [i, path] : utility::support::enumerate(paths)) {
                if (show_progress) {
                    auto file_size = utility::support::si_unit(std::filesystem::file_size(path));
                    load_progress.value().update(
                        loaded_data,
                        total_data,
                        "B",
                        fmt::format("Loading {:.2f}{}B from {}", file_size.first, file_size.second, path.string()));
                }

                std::filesystem::path idx_path = path.string() + ".idx";

                // If our index file does not exist we need to create it
                if (!std::filesystem::exists(idx_path)) {
                    build_index(path, idx_path, show_progress);
                }

                // Load the index file
                zstr::ifstream input(idx_path);
                while (input.good()) {
                    IndexItemFile item{};
                    input.read(reinterpret_cast<char*>(&item.item), sizeof(IndexItem));
                    item.fileno = i;

                    if (input.good()) {
                        // Can't bind packed fields, so need to construct the key by copy
                        std::pair<uint64_t, uint32_t> key;
                        key.first  = item.item.type;
                        key.second = item.item.subtype;
                        if (!data_types.contains(key)) {
                            data_types[key] = 0;
                        }
                        data_types[key]++;
                        idx.push(item);
                    }
                }

                loaded_data += std::filesystem::file_size(path);
            }

            if (show_progress) {
                load_progress.value().update(total_data, total_data, "B", "Loaded NBS files");
                load_progress.value().close();
            }

            // Finalise any memory we used and memory map if required, this is all the index items
            idx.finalise();

            // INDEX ORDERING
            switch (order) {
                case Ordering::TIME: {
                    // Sort by time
                    std::stable_sort(this->begin(), this->end(), [](const IndexItemFile& a, const IndexItemFile& b) {
                        return a.item.timestamp < b.item.timestamp;
                    });
                } break;
                case Ordering::TRANSMISSION: {

                    // Use the timestamp sort the list
                    // Once sorted we can use this data to build a binary search tree later
                    int64_t sort_progress    = 0;
                    int64_t total_sort_steps = idx.end() - idx.begin();
                    total_sort_steps         = int64_t(double(total_sort_steps) * std::log2(total_sort_steps));
                    int64_t sort_step        = total_sort_steps / 10000;

                    std::optional<utility::support::ProgressBar> sort_progress_bar;
                    if (show_progress) {
                        sort_progress_bar = utility::support::ProgressBar();
                    }

                    // Sort the data
                    std::sort(idx.begin(), idx.end(), [&](const IndexItemFile& a, const IndexItemFile& b) {
                        if (show_progress) {
                            if (sort_progress++ % sort_step == 0) {
                                // Ensure that if we go over 100% we keep increasing the limit
                                total_sort_steps = std::max(total_sort_steps, sort_progress + 1);

                                sort_progress_bar.value().update(sizeof(IndexItemFile) * ++sort_progress,
                                                                 sizeof(IndexItemFile) * total_sort_steps,
                                                                 "B",
                                                                 "Sorting");
                            }
                        }

                        return a.item.type != b.item.type         ? a.item.type < b.item.type
                               : a.item.subtype != b.item.subtype ? a.item.subtype < b.item.subtype
                                                                  : a.item.timestamp < b.item.timestamp;
                    });
                    if (show_progress) {
                        sort_progress_bar.value().update(sizeof(IndexItemFile) * total_sort_steps,
                                                         sizeof(IndexItemFile) * total_sort_steps,
                                                         "B",
                                                         "Sorting");
                        sort_progress_bar.value().close();
                    }

                    // Split up each of the sections so we can use it for transmission later
                    struct InterleaveTracking {
                        /// The current state of the binary tree partitions
                        std::vector<std::array<IndexItemFile*, 2>> ranges;
                        /// The number of elements we have loaded from the tree
                        int64_t n_loaded{0};
                        /// The number of elements that are available in this type
                        int64_t n_elements{0};
                        /// The number of bytes that have been loaded from this type
                        int64_t size{0};

                        InterleaveTracking(const std::array<IndexItemFile*, 2>& ranges)
                            : ranges({ranges}), n_elements(ranges[1] - ranges[0]) {}

                        inline IndexItemFile& get() {
                            // Current value is the middle of the last range element
                            return *(ranges.back()[0] + ((ranges.back()[1] - ranges.back()[0]) / 2));
                        }

                        inline bool finished() {
                            // We are finished if the current element is the last element
                            return n_loaded + 1 >= n_elements;
                        }

                        /**
                         * Gets the next element that would show up in the BST
                         */
                        inline void advance() {

                            // Advance our counters before we start (we have "loaded" the current element)
                            // Only do this if there is a real element at this point
                            if (ranges.back()[0] != ranges.back()[1]) {
                                this->n_loaded++;
                                this->size += this->get().item.length;
                            }

                            if (n_loaded >= n_elements) {
                                throw std::runtime_error("All elements have already been loaded");
                            }

                            // We start with one element so the initial case of this recalc_point is 0
                            int recalc_point = int(ranges.size()) - 1;

                            while (true) {
                                // We still have elements to check
                                if (recalc_point > 0) {
                                    auto& p = ranges[recalc_point - 1];
                                    auto& c = ranges[recalc_point];

                                    // Check if we are the left child of the parent and should move to its right child
                                    // Our parent needs 3 or more elements if we are to move to it's right child
                                    // It must have enough for left, self, right
                                    if (c[0] == p[0] && p[1] - p[0] > 2) {
                                        // Second half does not include the parents element (+1 to the midpoint)
                                        c = {p[0] + ((p[1] - p[0]) / 2) + 1, p[1]};
                                        break;
                                    }
                                    // If we have two or less elements we advance up the tree
                                    --recalc_point;
                                }
                                // Root element reached, add another layer
                                else {
                                    ranges.emplace_back();
                                    break;
                                }
                            }

                            // Recalculate from the point after the recalc_point to the end
                            for (int i = recalc_point + 1; i < int(ranges.size()); ++i) {
                                auto& prev = ranges[i - 1];
                                ranges[i]  = {prev[0], prev[0] + ((prev[1] - prev[0]) / 2)};
                            }

                            // If we hit an empty element then try again
                            if (ranges.back()[0] == ranges.back()[1]) {
                                advance();
                            }
                        }
                    };
                    std::vector<InterleaveTracking> sections;

                    // Split sections and prepare our interleaving structure
                    for (const auto& type : data_types) {
                        IndexItemFile v{};
                        v.item.type    = type.first.first;
                        v.item.subtype = type.first.second;
                        auto range     = std::equal_range(idx.begin(),
                                                      idx.end(),
                                                      v,
                                                      [](const IndexItemFile& a, const IndexItemFile& b) {
                                                          return a.item.type != b.item.type
                                                                         ? a.item.type < b.item.type
                                                                         : a.item.subtype < b.item.subtype;
                                                      });
                        sections.emplace_back(std::array<IndexItemFile*, 2>{range.first, range.second});
                    }

                    std::optional<utility::support::ProgressBar> interleave_progress_bar;
                    if (show_progress) {
                        interleave_progress_bar = utility::support::ProgressBar();
                    }

                    // Interleave the types based on size back into the original list
                    IndexStorage reordered(big_index_mode);
                    int64_t total_interleave_steps = (idx.end() - idx.begin());
                    int64_t interleave_step        = total_interleave_steps / 10000;
                    for (int64_t i = 0; !sections.empty(); ++i) {
                        // Find which of our data types has the smallest size loaded so far
                        auto s = std::min_element(
                            sections.begin(),
                            sections.end(),
                            [&](const InterleaveTracking& a, const InterleaveTracking& b) { return a.size < b.size; });

                        // Store the current item
                        auto& item = s->get();
                        reordered.push(item);

                        // We loaded the last bit of data, erase this type from consideration
                        if (s->finished()) {
                            sections.erase(s);
                        }
                        // Advance to the next item
                        else {
                            s->advance();
                        }

                        if (show_progress) {
                            if (i % interleave_step == 0) {
                                interleave_progress_bar.value().update(sizeof(IndexItemFile) * i,
                                                                       sizeof(IndexItemFile) * total_interleave_steps,
                                                                       "B",
                                                                       "Interleaving");
                            }
                        }
                    }
                    // Send a final update to the progress bar
                    if (show_progress) {
                        interleave_progress_bar.value().update(sizeof(IndexItemFile) * total_interleave_steps,
                                                               sizeof(IndexItemFile) * total_interleave_steps,
                                                               "B",
                                                               "Interleaving");
                        interleave_progress_bar.value().close();
                    }
                    reordered.finalise();

                    idx = std::move(reordered);
                } break;
                case Ordering::NONE: break;
                default: throw std::runtime_error("An invalid ordering was provided");
            }
        }

        /**
         * @brief Construct a new Index object based on the provided list of paths
         *
         * @param path          the path to the nbs file we are getting the index for
         * @param show_progress if the progress of loading the index should be printed
         */
        Index(const std::filesystem::path& path,
              const Ordering& order     = Ordering::TIME,
              const bool& show_progress = false)
            : Index(std::span(&path, &path + 1), order, show_progress) {}


        /**
         * @brief Begin iterator to the IndexItems held in this index
         *
         * @return the start iterator for the IndexItems
         */
        [[nodiscard]] IndexItemFile* begin();
        /**
         * @brief End iterator to the IndexItems held in this index
         *
         * @return the one past the end iterator for the IndexItems
         */
        [[nodiscard]] IndexItemFile* end();

    private:
        /// The storage for the actual data object which may be either memory mapped or in memory
        IndexStorage idx;

        /// The count of each datatype that we have seen
        std::map<std::pair<uint64_t, uint32_t>, int64_t> data_types;
    };

}  // namespace utility::nbs

#endif  // UTILITY_NBS_INDEX_HPP
