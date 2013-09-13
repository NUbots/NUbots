extern "C" {
    #include <sys/stat.h>
}

#include <sstream>
#include <fstream>
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
}
}
