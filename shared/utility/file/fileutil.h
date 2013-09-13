#ifndef UTILITY_FILEUTIL_H
#define UTILITY_FILEUTIL_H

#include <string>
#include <vector>

namespace utility {
namespace file {
    std::string loadFromFile(const std::string& path);

    void writeToFile(const std::string& path, const std::string& data, bool append = false);

    bool exists(const std::string& path);

    bool isDir(const std::string& path);

    std::vector<std::string> listDir(const std::string& path);
}
}
#endif
