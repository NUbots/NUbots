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
                if      (str == "UNCLASSIFIED") value = Value::UNCLASSIFIED;
                else if (str == "WHITE")        value = Value::WHITE;
                else if (str == "GREEN")        value = Value::GREEN;
                else if (str == "ORANGE")       value = Value::ORANGE;
                else if (str == "YELLOW")       value = Value::YELLOW;
                else if (str == "CYAN")         value = Value::CYAN;
                else if (str == "MAGENTA")      value = Value::MAGENTA;
                else if (str == "WHITE_GREEN")  value = Value::WHITE_GREEN;
                else throw std::runtime_error("String " + str + " did not match any enum for Colour");
            }
        
            Colour(char const& c)         : value(Value::UNCLASSIFIED) {
                if      (c == 'u') value = Value::UNCLASSIFIED;
                else if (c == 'w') value = Value::WHITE;
                else if (c == 'g') value = Value::GREEN;
                else if (c == 'o') value = Value::ORANGE;
                else if (c == 'y') value = Value::YELLOW;
                else if (c == 'c') value = Value::CYAN;
                else if (c == 'm') value = Value::MAGENTA;
                else if (c == 'f') value = Value::WHITE_GREEN;
                else throw std::runtime_error("String " + std::to_string(c) + " did not match any enum for Colour");
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
                        throw std::runtime_error("enum Colour's value is corrupt, unknown value stored" + std::to_string(static_cast<uint8_t>(value)));
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
                        throw std::runtime_error("enum Colour's value is corrupt, unknown value stored" + std::to_string(static_cast<uint8_t>(value)));
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
