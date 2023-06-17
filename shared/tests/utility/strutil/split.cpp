#include <catch2/catch_test_macros.hpp>

#include "utility/strutil/strutil.hpp"

SCENARIO("Splitting a string using a delimiter", "[utility][strutil][split]") {
    GIVEN("An empty string and an empty delimiter") {
        std::string input{};
        std::string delimiter{};
        WHEN("Splitting the string") {
            std::vector<std::string> result = utility::strutil::split(input, delimiter);
            THEN("The result should contain a single empty string") {
                CHECK(result == std::vector<std::string>{""});
            }
        }
    }

    GIVEN("A string without the delimiter") {
        std::string input     = "Hello World!";
        std::string delimiter = "x";
        WHEN("Splitting the string") {
            std::vector<std::string> result = utility::strutil::split(input, delimiter);
            THEN("The result should contain the same string") {
                CHECK(result == std::vector<std::string>{"Hello World!"});
            }
        }
    }

    GIVEN("A string with a single delimiter") {
        std::string input     = "Hello World!";
        std::string delimiter = " ";
        WHEN("Splitting the string") {
            std::vector<std::string> result = utility::strutil::split(input, delimiter);
            THEN("The result should contain the substrings split at the delimiter") {
                CHECK(result == std::vector<std::string>{"Hello", "World!"});
            }
        }
    }

    GIVEN("A string with multiple delimiters") {
        std::string input     = "Hello World! How are you!";
        std::string delimiter = " ";
        WHEN("Splitting the string") {
            std::vector<std::string> result = utility::strutil::split(input, delimiter);
            THEN("The result should contain the substrings split at each occurrence of the delimiter") {
                CHECK(result == std::vector<std::string>{"Hello", "World!", "How", "are", "you!"});
            }
        }
    }

    GIVEN("A string with a multi character delimiter") {
        std::string input     = "Hello, World!";
        std::string delimiter = ", ";
        WHEN("Splitting the string") {
            std::vector<std::string> result = utility::strutil::split(input, delimiter);
            THEN("The result should contain the substrings split at the delimiter") {
                CHECK(result == std::vector<std::string>{"Hello", "World!"});
            }
        }
    }

    GIVEN("A string with a single character delimiter") {
        std::string input = "Hello, World!";
        char delimiter    = ',';
        WHEN("Splitting the string") {
            std::vector<std::string> result = utility::strutil::split(input, delimiter);
            THEN("The result should contain the substrings split at the delimiter") {
                CHECK(result == std::vector<std::string>{"Hello", " World!"});
            }
        }
    }

    GIVEN("A non empty string with an empty delimiter") {
        std::string input = "Hello World!";
        std::string delimiter{};
        WHEN("Splitting the string") {
            std::vector<std::string> result = utility::strutil::split(input, delimiter);
            THEN("The result should have split the string into individual characters") {
                CHECK(result == std::vector<std::string>{"H", "e", "l", "l", "o", " ", "W", "o", "r", "l", "d", "!"});
            }
        }
    }
}
