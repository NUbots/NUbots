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
    
        on<Configuration>("WavReader.yaml").then([this](const Configuration& cfg) {
            // Use configuration here from file WavReader.yaml
            this->log_level = cfg["log_level"].as<NUClear::LogLevel>();
            this->config.wav_path = cfg["wav_path"].as<std::string>(); //config is the name of the struct
            std::cout << config.wav_path << std::endl;
            log<NUClear::DEBUG>(config.wav_path);
		read();
        });
		
	WavReader::read(){
		FILE *wavin;
		char* buf;
		int nread = 1, var = 0; //What is var, need to understand what these two variables are doing

		//Wasn't complaing when there was no test.wav, where is this coming from?
		wavin = fopen(config.wav_path, "rb");
		fseek(wavin, 44, SEEK_SET);
		
		fessk(wavin, 0, SEEK_END);
		lSize = ftell (wavin);
		rewind(pFile);
		while (!feof(wavin)) {
		 nread = fread(buf, 1, sizeof(buf), wavin);

		 if (var) {
		     printf("test1\n");
		 } else {
		     printf(wavin->);
		 }
		}

		fclose(wavin);
	}
    }

}  // namespace module::input
