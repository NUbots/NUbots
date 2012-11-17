#ifndef NUBOTDATACONFIG_H
#define NUBOTDATACONFIG_H
#define DATA_DIR (std::string(getenv("HOME")) + std::string("/nubot/"))
#define CONFIG_DIR (DATA_DIR + std::string("Config/"))
#define RULE_DIR (CONFIG_DIR + std::string("Rules/"))
#endif // NUBOTDATACONFIG_H
