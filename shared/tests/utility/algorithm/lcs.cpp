#include "utility/algorithm/lcs.hpp"

#include <catch2/catch_test_macros.hpp>

SCENARIO("finding the longest common subsequence between two lists", "[utility][algorithm][lcs]") {
    GIVEN("Two empty lists") {
        std::vector<int> a = {};
        std::vector<int> b = {};

        WHEN("Calculating the longest common subsequence") {
            auto result = utility::algorithm::lcs(a, b);

            THEN("Both match vectors should be empty") {
                CHECK(result.first.empty());
                CHECK(result.second.empty());
            }
        }
    }

    GIVEN("One empty list and one non-empty list") {
        std::vector<int> a = {1, 2, 3};
        std::vector<int> b = {};

        WHEN("Calculating the longest common subsequence") {
            auto result = utility::algorithm::lcs(a, b);

            THEN(
                "The first match vector should have all elements as false, and the second match vector should be "
                "empty") {
                CHECK(result.first == std::vector<bool>(a.size(), false));
                CHECK(result.second.empty());
            }
        }
    }

    GIVEN("Two identical lists") {
        std::vector<int> a = {1, 2, 3, 4, 5};
        std::vector<int> b = {1, 2, 3, 4, 5};

        WHEN("Calculating the longest common subsequence") {
            auto result = utility::algorithm::lcs(a, b);

            THEN("Both match vectors should have all elements as true") {
                CHECK(result.first == std::vector<bool>(a.size(), true));
                CHECK(result.second == std::vector<bool>(b.size(), true));
            }
        }
    }

    GIVEN("Two different lists") {
        std::vector<int> a = {1, 2, 3, 4, 5};
        std::vector<int> b = {6, 7, 8, 9, 10};

        WHEN("Calculating the longest common subsequence") {
            auto result = utility::algorithm::lcs(a, b);

            THEN("Both match vectors should have all elements as false") {
                CHECK(result.first == std::vector<bool>(a.size(), false));
                CHECK(result.second == std::vector<bool>(b.size(), false));
            }
        }
    }

    GIVEN("Two partially matching lists") {
        std::vector<int> a = {1, 2, 3, 4, 5};
        std::vector<int> b = {2, 4, 6, 8};

        WHEN("Calculating the longest common subsequence") {
            auto result = utility::algorithm::lcs(a, b);

            THEN(
                "The first match vector should indicate the partially matching elements, and the second match vector "
                "should match the remaining elements") {
                CHECK(result.first == std::vector<bool>({false, true, false, true, false}));
                CHECK(result.second == std::vector<bool>({true, true, false, false}));
            }
        }
    }

    GIVEN("Two lists with a single common element") {
        std::vector<int> a = {1, 2, 3};
        std::vector<int> b = {3, 4, 5};

        WHEN("Calculating the longest common subsequence") {
            auto result = utility::algorithm::lcs(a, b);

            THEN(
                "The first match vector should have the last element as true, and the second match vector should have "
                "the first element as true") {
                CHECK(result.first == std::vector<bool>({false, false, true}));
                CHECK(result.second == std::vector<bool>({true, false, false}));
            }
        }
    }

    GIVEN("Two lists with multiple common subsequences") {
        std::vector<int> a = {1, 2, 3, 4, 5, 6, 7};
        std::vector<int> b = {2, 4, 6, 7, 8, 9};

        WHEN("Calculating the longest common subsequence") {
            auto result = utility::algorithm::lcs(a, b);

            THEN("The match vectors should indicate the longest common subsequences") {
                CHECK(result.first == std::vector<bool>({false, true, false, true, false, true, true}));
                CHECK(result.second == std::vector<bool>({true, true, true, true, false, false}));
            }
        }
    }

    GIVEN("Two vectors with duplicate entries") {
        std::vector<int> a = {1, 2, 2, 3, 4};
        std::vector<int> b = {2, 2, 4, 4, 5};

        WHEN("Finding the longest common subsequence") {
            auto [match_a, match_b] = utility::algorithm::lcs(a, b);

            THEN("The common subsequence should be identified") {
                CHECK(match_a == std::vector<bool>({false, true, true, false, true}));
                CHECK(match_b == std::vector<bool>({true, true, false, true, false}));
            }
        }
    }

    GIVEN("One vector with duplicate entries and one vector without duplicates") {
        std::vector<int> a = {1, 2, 2, 3, 4};
        std::vector<int> b = {2, 4, 6, 8, 10};

        WHEN("Finding the longest common subsequence") {
            auto [match_a, match_b] = utility::algorithm::lcs(a, b);

            THEN("The common subsequence should be identified") {
                CHECK(match_a == std::vector<bool>({false, false, true, false, true}));
                CHECK(match_b == std::vector<bool>({true, true, false, false, false}));
            }
        }
    }
}
