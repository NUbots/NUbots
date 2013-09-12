#ifndef UTILITY_STRUTIL_H
#define UTILITY_STRUTIL_H

#include <string>

namespace utility {
    namespace strutil {

        // http://stackoverflow.com/a/874160/1387006
        bool endsWith(const std::string& str, const std::string& ending) {

            if (str.length() >= ending.length()) {
                return (0 == str.compare (str.length() - ending.length(), ending.length(), ending));
            }
            else {
                return false;
            }
        }

        bool startsWith(const std::string& str, const std::string& start) {

            if (str.length() >= start.length()) {
                return (0 == str.compare (0, start.length(), start));
            }
            else {
                return false;
            }
        }
    }
}
#endif
