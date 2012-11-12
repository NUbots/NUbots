#ifndef NUBOTDATACONFIG_H
#define NUBOTDATACONFIG_H
#ifdef TARGET_IS_PC
    #define DATA_DIR (std::string(getenv("HOME")) + std::string("/robocup/"))
    #define CONFIG_DIR (DATA_DIR + std::string("Config/Darwin/"))
    #define RULE_DIR (CONFIG_DIR + std::string("Rules/"))
#else
    #define DATA_DIR (std::string(getenv("HOME")) + std::string("/nubot/"))
    #define CONFIG_DIR (DATA_DIR + std::string("Config/"))
    #define RULE_DIR (CONFIG_DIR + std::string("Rules/"))
#endif

#endif // NUBOTDATACONFIG_H
