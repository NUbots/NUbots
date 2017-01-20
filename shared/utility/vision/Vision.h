/*
 * This file is part of the NUbots Codebase.
 *
 * The NUbots Codebase is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The NUbots Codebase is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the NUbots Codebase.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright 2013 NUBots <nubots@nubots.net>
 */
#ifndef UTILITY_VISION_VISION_H
#define UTILITY_VISION_VISION_H

namespace utility {
    namespace vision {

        struct Colour {
            enum Value {
                // Main classifications
                UNCLASSIFIED = 0,
                WHITE        = 1,
                GREEN        = 2,
                ORANGE       = 3,
                YELLOW       = 4,
                CYAN         = 5,
                MAGENTA      = 6,

                // Ambiguous Classifications
                WHITE_GREEN  = 7
            };
            Value value;
        
            // Constructors
            Colour()                       : value(Value::UNCLASSIFIED)       {}
            Colour(int const& value)       : value(static_cast<Value>(value)) {}
            Colour(uint8_t const& value)   : value(static_cast<Value>(value)) {}
            Colour(uint32_t const& value)  : value(static_cast<Value>(value)) {}
            Colour(Value const& value)     : value(value) {}
            Colour(std::string const& str) : value(Value::UNCLASSIFIED) {
                if (str == "UNCLASSIFIED") value = Value::UNCLASSIFIED;
                if (str == "WHITE")        value = Value::WHITE;
                if (str == "GREEN")        value = Value::GREEN;
                if (str == "ORANGE")       value = Value::ORANGE;
                if (str == "YELLOW")       value = Value::YELLOW;
                if (str == "CYAN")         value = Value::CYAN;
                if (str == "MAGENTA")      value = Value::MAGENTA;
                if (str == "WHITE_GREEN")  value = Value::WHITE_GREEN;
                throw std::runtime_error("String did not match any enum for Colour");
            }
        
            Colour(char const& c)         : value(Value::UNCLASSIFIED) {
                if (c == 'u') value = Value::UNCLASSIFIED;
                if (c == 'w') value = Value::WHITE;
                if (c == 'g') value = Value::GREEN;
                if (c == 'o') value = Value::ORANGE;
                if (c == 'y') value = Value::YELLOW;
                if (c == 'c') value = Value::CYAN;
                if (c == 'm') value = Value::MAGENTA;
                if (c == 'f') value = Value::WHITE_GREEN;
                throw std::runtime_error("String did not match any enum for Colour");
            }
        
            // Operators
            bool operator <(Colour const& other) const         { return value < other.value;  }
            bool operator >(Colour const& other) const         { return value > other.value;  }
            bool operator <=(Colour const& other) const        { return value <= other.value; }
            bool operator >=(Colour const& other) const        { return value >= other.value; }
            bool operator ==(Colour const& other) const        { return value == other.value; }
            bool operator !=(Colour const& other) const        { return value != other.value; }
            bool operator <(Colour::Value const& other) const  { return value < other;        }
            bool operator >(Colour::Value const& other) const  { return value > other;        }
            bool operator <=(Colour::Value const& other) const { return value <= other;       }
            bool operator >=(Colour::Value const& other) const { return value >= other;       }
            bool operator ==(Colour::Value const& other) const { return value == other;       }
            bool operator !=(Colour::Value const& other) const { return value != other;       }
        
            // Conversions
            operator Value() const    { return value; }
            operator int() const      { return value; }
            operator uint8_t() const  { return value; }
            operator uint32_t() const { return value; }
        
            operator std::string() const {
                switch(value) {
                    case Value::UNCLASSIFIED: return "UNCLASSIFIED";
                    case Value::WHITE:        return "WHITE";
                    case Value::GREEN:        return "GREEN";
                    case Value::ORANGE:       return "ORANGE";
                    case Value::YELLOW:       return "YELLOW";
                    case Value::CYAN:         return "CYAN";
                    case Value::MAGENTA:      return "MAGENTA";
                    case Value::WHITE_GREEN:  return "WHITE_GREEN";
                    default:
                        throw std::runtime_error("enum Colour's value is corrupt, unknown value stored");
                }
            }

            operator char() const {
                switch(value) {
                    case Value::UNCLASSIFIED: return 'u';
                    case Value::WHITE:        return 'w';
                    case Value::GREEN:        return 'g';
                    case Value::ORANGE:       return 'o';
                    case Value::YELLOW:       return 'y';
                    case Value::CYAN:         return 'c';
                    case Value::MAGENTA:      return 'm';
                    case Value::WHITE_GREEN:  return 'f';
                    default:
                        throw std::runtime_error("enum Colour's value is corrupt, unknown value stored");
                }
            }
        };

        struct Pixel {
            Pixel() : rgba(0) {}
            Pixel(uint32_t rgba) : rgba(rgba) {}
            Pixel(uint8_t r, uint8_t g, uint8_t b, uint8_t a) : 
                        components({ r, g, b, a}) {}
            Pixel(uint8_t r, uint8_t g, uint8_t b) :
                        components({ r, g, b, 0}) {}
            Pixel(const Pixel& pixel) : rgba(pixel.rgba) {}

            union {
                struct {
                    union {
                        uint8_t r;
                        uint8_t y;
                    };
                    union {
                        uint8_t g;
                        uint8_t u;
                        uint8_t cb;
                    };
                    union {
                        uint8_t b;
                        uint8_t v;
                        uint8_t cr;
                    };

                    uint8_t a;
                } components;

                uint32_t rgba;
            };
        };

    } // vision
} // utility

#endif // MESSAGE_VISION_VISION_H
