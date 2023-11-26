/*
 * MIT License
 *
 * Copyright (c) 2020 NUbots
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
#ifndef MODULE_INPUT_CAMERA_SETTINGS_HPP
#define MODULE_INPUT_CAMERA_SETTINGS_HPP

#include <fmt/format.h>
#include <stdexcept>
#include <string>

extern "C" {
#include <aravis-0.8/arv.h>
}

namespace module::input {
    template <typename T>
    struct SettingsFunctions;

    template <>
    struct SettingsFunctions<ArvGcInteger> {
        static int64_t read(ArvGcInteger* setting, GError** error) {
            return arv_gc_integer_get_value(setting, error);
        }
        static void write(ArvGcInteger* setting, const int64_t& v, GError** error) {
            arv_gc_integer_set_value(setting, v, error);
        }
        static bool valid(ArvGcInteger* setting, const int64_t& v) {
            GError* error = nullptr;
            std::string u = unit(setting);
            int64_t min   = arv_gc_integer_get_min(setting, &error);
            if (error != nullptr) {
                g_error_free(error);
                error = nullptr;
            }
            int64_t max = arv_gc_integer_get_max(setting, &error);
            if (error != nullptr) {
                g_error_free(error);
                error = nullptr;
            }
            if (min > v || v > max) {
                throw std::runtime_error(fmt::format("{0} is not between {1}{3} and {2}{3}", v, min, max, u));
            }
            return true;
        }
        static std::string unit(ArvGcInteger* setting) {
            const char* u = arv_gc_integer_get_unit(setting);
            return u == nullptr ? "" : u;
        }
    };

    template <>
    struct SettingsFunctions<ArvGcFloat> {
        static double read(ArvGcFloat* setting, GError** error) {
            return arv_gc_float_get_value(setting, error);
        }
        static void write(ArvGcFloat* setting, const double& v, GError** error) {
            arv_gc_float_set_value(setting, v, error);
        }
        static bool valid(ArvGcFloat* setting, const double& v) {
            GError* error = nullptr;
            std::string u = unit(setting);
            double min    = arv_gc_float_get_min(setting, &error);
            if (error != nullptr) {
                g_error_free(error);
                error = nullptr;
            }
            double max = arv_gc_float_get_max(setting, &error);
            if (error != nullptr) {
                g_error_free(error);
                error = nullptr;
            }
            if (min > v || v > max) {
                throw std::runtime_error(fmt::format("{0} is not between {1}{3} and {2}{3}", v, min, max, u));
            }
            return true;
        }
        static std::string unit(ArvGcFloat* setting) {
            const char* u = arv_gc_float_get_unit(setting);
            return u == nullptr ? "" : u;
        }
    };

    template <>
    struct SettingsFunctions<ArvGcBoolean> {
        static bool read(ArvGcBoolean* setting, GError** error) {
            return arv_gc_boolean_get_value(setting, error) != 0;
        }
        static void write(ArvGcBoolean* setting, const bool& v, GError** error) {
            arv_gc_boolean_set_value(setting, static_cast<gboolean>(v), error);
        }
        static bool valid(ArvGcBoolean* /* setting */, const bool& /* error */) {
            return true;
        }
        static std::string unit(ArvGcBoolean* /* setting */) {
            return "";
        }
    };

    template <>
    struct SettingsFunctions<ArvGcEnumeration> {
        static std::string read(ArvGcEnumeration* setting, GError** error) {
            const char* s = arv_gc_enumeration_get_string_value(setting, error);
            return s == nullptr ? "" : s;
        }
        static void write(ArvGcEnumeration* setting, const std::string& v, GError** error) {
            arv_gc_enumeration_set_string_value(setting, v.c_str(), error);
        }
        static bool valid(ArvGcEnumeration* setting, const std::string& v) {
            GError* error       = nullptr;
            unsigned int len    = 0;
            const char** values = arv_gc_enumeration_dup_available_string_values(setting, &len, &error);
            if (error != nullptr) {
                g_error_free(error);
                error = nullptr;
            }
            std::vector<std::string> options;
            for (unsigned int i = 0; i < len; ++i) {
                options.emplace_back(values[i]);
            }
            g_free(values);

            if (!options.empty() && std::find(options.begin(), options.end(), v) == options.end()) {
                std::stringstream s;
                s << "Value must be one of { ";
                s << options.front();
                for (std::size_t i = 1; i < options.size(); ++i) {
                    s << ", " << options[i];
                }
                s << " }";

                throw std::runtime_error(s.str());
            }

            return true;
        }
        static std::string unit(ArvGcEnumeration* /* setting */) {
            return "";
        }
    };

    template <typename T, typename U>
    std::string set_setting(T* setting, U&& value) {
        GError* error = nullptr;

        // Get our current value
        auto current = SettingsFunctions<T>::read(setting, &error);
        if (error) {
            std::string msg = fmt::format("Failed to read the current value: \"{}\"", error->message);
            g_error_free(error);
            throw std::runtime_error(msg);
        }

        // Get the unit that it is measured in
        std::string unit = SettingsFunctions<T>::unit(setting);

        // Check if this feature is locked
        bool locked = arv_gc_feature_node_is_locked(reinterpret_cast<ArvGcFeatureNode*>(setting), &error);
        if (error) {
            g_error_free(error);
            error = nullptr;
        }
        if (locked) {
            throw std::runtime_error(
                fmt::format("Cannot change from {0}{2} to {1}{2} as it is locked", current, value, unit));
        }

        // If we are planning to change the value
        if (value != current) {

            if (SettingsFunctions<T>::valid(setting, std::forward<U>(value))) {
                SettingsFunctions<T>::write(setting, std::forward<U>(value), &error);
                if (error) {
                    std::string msg = fmt::format("Failed changing from {0}{2} to {1}{2}: \"{3}\"",
                                                  current,
                                                  value,
                                                  unit,
                                                  error->message);
                    g_error_free(error);
                    throw std::runtime_error(msg);
                }
                return fmt::format("changed {0}{2} to {1}{2}", current, value, unit);
            }
        }
        return "";
    }
}  // namespace module::input

#endif  // MODULE_INPUT_CAMERA_SETTINGS_HPP
