#include "fileutil.h"

extern "C" {
    #include <sys/stat.h>
    #include <dirent.h>
}

#include <sstream>
#include <fstream>
#include <system_error>

namespace utility {
namespace file {
    std::string loadFromFile(const std::string& path) {
        std::ifstream data(path, std::ios::in);

        // There are a lot of nice ways to read a file into a string but this is one of the quickest.
        // See: http://stackoverflow.com/a/116228
        std::stringstream stream;
        stream << data.rdbuf();

        return stream.str();
    }

    void writeToFile(const std::string& path, const std::string& data, bool append) {
        std::ofstream file(path, 
            append 
                ? std::ios::out | std::ios::app
                : std::ios::out);
        file << data;
    }

    bool exists(const std::string& path) {
        // Shamelessly stolen from: http://stackoverflow.com/a/12774387/1387006 
        struct stat buffer;   
        return (stat (path.c_str(), &buffer) == 0);
    }

    // Test if a passed path is a directory
    bool isDir(const std::string& path) {

        int status;
        struct stat st_buf;

        // Get the status of the file system object.
        status = stat(path.c_str(), &st_buf);
        if (status != 0) {
            throw std::system_error(errno, std::system_category(), "Error checking if path is file or directory");
        }

        // Return if our varible is a directory
        return S_ISDIR(st_buf.st_mode);
    }

    // List the contents of a directory
    std::vector<std::string> listDir(const std::string& path) {

        auto dir = opendir(path.c_str());
        std::vector<std::string> result;

        if(dir != nullptr) {
            for(dirent* ent = readdir(dir); ent != nullptr; ent = readdir(dir)) {

                auto file = std::string(ent->d_name);

                if(file == "." || file == "..") {
                    continue;
                }

                if(ent->d_type & DT_DIR) {
                    result.push_back(file + "/");
                }
                else {
                    result.push_back(file);
                }
            }

            closedir(dir);
        }
        else {
            // TODO Throw an error or something
        }

        return result;
    }
}
}
