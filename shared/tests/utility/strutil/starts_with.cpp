#include <catch2/catch_test_macros.hpp>

#include "utility/strutil/strutil.hpp"

SCENARIO("Check a string starts with another string", "[utility][strutil][starts_with]") {
    GIVEN("A string and a starting substring") {
        std::string str   = "Hello World";
        std::string start = "Hello";

        WHEN("Checking if the string starts with the starting substring") {
            bool result = utility::strutil::starts_with(str, start);

            THEN("It should return true") {
                CHECK(result);
            }
        }
    }

    GIVEN("A string and a starting substring") {
        std::string str   = "Hello World";
        std::string start = "World";

        WHEN("Checking if the string starts with the starting substring") {
            bool result = utility::strutil::starts_with(str, start);

            THEN("It should return false") {
                CHECK_FALSE(result);
            }
        }
    }

    GIVEN("A string and a starting substring") {
        std::string str   = "Hello World";
        std::string start = "Hello World and more";

        WHEN("Checking if the string starts with the starting substring") {
            bool result = utility::strutil::starts_with(str, start);

            THEN("It should return false") {
                CHECK_FALSE(result);
            }
        }
    }
}
