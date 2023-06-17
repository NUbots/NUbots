#include <catch2/catch_test_macros.hpp>

#include "utility/strutil/strutil.hpp"

SCENARIO("Check a string ends with another string", "[utility][strutil][ends_with]") {
    GIVEN("A string and an ending") {
        std::string str    = "Hello World";
        std::string ending = "World";

        WHEN("Checking if the string ends with the ending") {
            bool result = utility::strutil::ends_with(str, ending);

            THEN("It should return true") {
                CHECK(result);
            }
        }
    }

    GIVEN("A string and an ending") {
        std::string str    = "Hello World";
        std::string ending = "Hello";

        WHEN("Checking if the string ends with the ending") {
            bool result = utility::strutil::ends_with(str, ending);

            THEN("It should return false") {
                CHECK_FALSE(result);
            }
        }
    }

    GIVEN("A string and an ending") {
        std::string str    = "Hello";
        std::string ending = "Hello World";

        WHEN("Checking if the string ends with the ending") {
            bool result = utility::strutil::ends_with(str, ending);

            THEN("It should return false") {
                CHECK_FALSE(result);
            }
        }
    }
}
