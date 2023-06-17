#include <catch2/catch_test_macros.hpp>

#include "utility/strutil/strutil.hpp"

SCENARIO("Trimming left whitespace from a string", "[utility][strutil][trim][trim_left]") {
    GIVEN("An empty string") {
        std::string input{};
        WHEN("Trimming left whitespace") {
            std::string result = utility::strutil::trim_left(input);
            THEN("The result should be an empty string") {
                CHECK(result.empty());
            }
        }
    }

    GIVEN("A string without leading whitespace") {
        std::string input = "Hello, World!";
        WHEN("Trimming left whitespace") {
            std::string result = utility::strutil::trim_left(input);
            THEN("The result should be the same string") {
                CHECK(result == input);
            }
        }
    }

    GIVEN("A string with leading whitespace") {
        std::string input = "    Hello, World!";
        WHEN("Trimming left whitespace") {
            std::string result = utility::strutil::trim_left(input);
            THEN("The result should have leading whitespace removed") {
                CHECK(result == "Hello, World!");
            }
        }
    }

    GIVEN("A string with only whitespace") {
        std::string input = "\f \n \r \t \v";
        WHEN("Trimming left whitespace") {
            std::string result = utility::strutil::trim_left(input);
            THEN("The result should be an empty string") {
                CHECK(result.empty());
            }
        }
    }
}

SCENARIO("Trimming right whitespace from a string", "[utility][strutil][trim][trim_right]") {
    GIVEN("An empty string") {
        std::string input{};
        WHEN("Trimming right whitespace") {
            std::string result = utility::strutil::trim_right(input);
            THEN("The result should be an empty string") {
                CHECK(result.empty());
            }
        }
    }

    GIVEN("A string without trailing whitespace") {
        std::string input = "Hello, World!";
        WHEN("Trimming right whitespace") {
            std::string result = utility::strutil::trim_right(input);
            THEN("The result should be the same string") {
                CHECK(result == input);
            }
        }
    }

    GIVEN("A string with trailing whitespace") {
        std::string input = "Hello, World!   ";
        WHEN("Trimming right whitespace") {
            std::string result = utility::strutil::trim_right(input);
            THEN("The result should have trailing whitespace removed") {
                CHECK(result == "Hello, World!");
            }
        }
    }

    GIVEN("A string with only whitespace") {
        std::string input = "\f \n \r \t \v";
        WHEN("Trimming right whitespace") {
            std::string result = utility::strutil::trim_right(input);
            THEN("The result should be an empty string") {
                CHECK(result.empty());
            }
        }
    }
}

SCENARIO("Trimming leading and trailing whitespace from a string", "[utility][strutil][trim][trim_all]") {
    GIVEN("An empty string") {
        std::string input{};
        WHEN("Trimming whitespace") {
            std::string result = utility::strutil::trim(input);
            THEN("The result should be an empty string") {
                CHECK(result.empty());
            }
        }
    }

    GIVEN("A string without leading or trailing whitespace") {
        std::string input = "Hello, World!";
        WHEN("Trimming whitespace") {
            std::string result = utility::strutil::trim(input);
            THEN("The result should be the same string") {
                CHECK(result == input);
            }
        }
    }

    GIVEN("A string with leading and/or trailing whitespace") {
        std::string input = "   Hello, World!   ";
        WHEN("Trimming whitespace") {
            std::string result = utility::strutil::trim(input);
            THEN("The result should have leading and trailing whitespace removed") {
                CHECK(result == "Hello, World!");
            }
        }
    }

    GIVEN("A string with only whitespace") {
        std::string input = " \f\n \r \t\v";
        WHEN("Trimming whitespace") {
            std::string result = utility::strutil::trim(input);
            THEN("The result should be an empty string") {
                CHECK(result.empty());
            }
        }
    }
}
