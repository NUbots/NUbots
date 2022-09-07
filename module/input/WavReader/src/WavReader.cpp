#include "WavReader.hpp"

#include <cstdint>
#include <fstream>
#include <iostream>
#include <string>

#include "extension/Configuration.hpp"
namespace module::input {

    using extension::Configuration;
 
    //function to read wav file - See example of functions in
 
    
    WavReader::WavReader(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)), config{} {
    
	FILE *wavin;
	char buf[3200];
	int nread = 1, final = 0; 
	
	//Wasn't complaing when there was no test.wav, where is this coming from?
	wavin = fopen("test.wav", "rb");
	fseek(wavin, 44, SEEK_SET);
	while (!feof(wavin)) {
	 nread = fread(buf, 1, sizeof(buf), wavin);

	 if (final) {
	     printf("test1\n");
	 } else {
	     printf("test2\n");
	 }
	}

	fclose(wavin);

    
        on<Configuration>("WavReader.yaml").then([this](const Configuration& cfg) {
            // Use configuration here from file WavReader.yaml
            this->log_level = cfg["log_level"].as<NUClear::LogLevel>();
            this->config.wav_path = cfg["wav_path"].as<std::string>(); //config is the name of the struct
            std::cout << config.wav_path << std::endl;
            log<NUClear::DEBUG>(config.wav_path);
        });
    }

}  // namespace module::input
