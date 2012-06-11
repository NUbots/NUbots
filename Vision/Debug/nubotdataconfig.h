#ifndef NUBOTDATACONFIG_H
#define NUBOTDATACONFIG_H

#define DATA_DIR (std::string(getenv("HOME")) + std::string("/robocup/"))
#define CONFIG_DIR (DATA_DIR + std::string("Config/Darwin/"))
#define RULE_DIR (CONFIG_DIR + std::string("Rules/"))

#endif // NUBOTDATACONFIG_H
