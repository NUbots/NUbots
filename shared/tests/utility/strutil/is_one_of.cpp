#include <catch2/catch_test_macros.hpp>

#include "utility/strutil/strutil.hpp"

SCENARIO("Checking if a string is one of a set of options", "[utility][strutil][is_one_of]") {
    GIVEN("Empty options") {
        std::vector<std::string> options = {};
        WHEN("Checking if a string is one of the options") {
            bool result = utility::strutil::is_one_of("hello", options);
            THEN("The result should be false") {
                CHECK_FALSE(result);
            }
        }
    }

    GIVEN("Non-empty options") {
        std::vector<std::string> options = {"hello", "world", "cucumber"};
        WHEN("Checking if a string is one of the options") {
            THEN("The result should be true when the string is one of the options") {
                CHECK(utility::strutil::is_one_of("hello", options));
                CHECK(utility::strutil::is_one_of("world", options));
                CHECK(utility::strutil::is_one_of("cucumber", options));
            }

            THEN("The result should be false when the string is not one of the options") {
                CHECK_FALSE(utility::strutil::is_one_of("foo", options));
                CHECK_FALSE(utility::strutil::is_one_of("bar", options));
                CHECK_FALSE(utility::strutil::is_one_of("hell", options));
            }
        }
    }
}
