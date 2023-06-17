#include <catch2/catch_test_macros.hpp>

#include "utility/strutil/strutil.hpp"

SCENARIO("Joining a list with a delimiter", "[utility][strutil][join]") {
    GIVEN("An empty list") {
        std::vector<int> list = {};
        WHEN("Joining with a delimiter") {
            std::string result = utility::strutil::join(list, ",");
            THEN("The result should be an empty string") {
                CHECK(result.empty());
            }
        }
    }

    GIVEN("A list of integers") {
        std::vector<int> list = {1, 2, 3, 4, 5};
        WHEN("Joining with a comma delimiter") {
            std::string result = utility::strutil::join(list, ",");
            THEN("The result should be a string with elements separated by commas") {
                CHECK(result == "1,2,3,4,5");
            }
        }
    }

    GIVEN("A list of strings") {
        std::vector<std::string> list = {"hello", "world", "and", "others"};
        WHEN("Joining with a space delimiter") {
            std::string result = utility::strutil::join(list, " ");
            THEN("The result should be a string with elements separated by spaces") {
                CHECK(result == "hello world and others");
            }
        }
    }

    GIVEN("A list with a different delimiter") {
        std::vector<std::string> list = {"pen", "pineapple", "apple", "pen"};
        WHEN("Joining with a hyphen delimiter") {
            std::string result = utility::strutil::join(list, "-");
            THEN("The result should be a string with elements separated by hyphens") {
                CHECK(result == "pen-pineapple-apple-pen");
            }
        }
    }

    GIVEN("A list of strings") {
        std::vector<std::string> list = {"pen", "pineapple", "apple", "pen"};
        WHEN("Joining with an empty delimiter") {
            std::string result = utility::strutil::join(list, "");
            THEN("The result should be a string with elements concatenated without a delimiter") {
                CHECK(result == "penpineappleapplepen");
            }
        }
    }

    GIVEN("A list of strings") {
        std::vector<std::string> list = {"pen", "pineapple", "apple", "pen"};
        WHEN("Joining with a multi-character delimiter") {
            std::string result = utility::strutil::join(list, "-->");
            THEN("The result should be a string with elements separated by the multi-character delimiter") {
                CHECK(result == "pen-->pineapple-->apple-->pen");
            }
        }
    }
}
