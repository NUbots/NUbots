#include "WavReader.hpp"

#include <cstdint>
#include <fstream>
#include <iostream>
#include <string>
#include <stdio.h>
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
		readWav();
        });
		
	
		//generate new speech recognition module
		//make reactor onTrigger audiodata (receives audiodata from this file)
    }
    
    void WavReader::readWav(){
		FILE *wavin;
		char* buf = new char();
		int nread = 1, var = 0; //Not sur what var is, was called 'final' earlier

		wavin = fopen(config.wav_path.c_str(), "rb");
		fseek(wavin, 44, SEEK_SET);
		
		//fessk(wavin, 0, SEEK_END); //NOT SURE WHAT THIS IS SUPOSED TO DO
		long size = ftell (wavin);
		rewind(wavin);
		
		nread = fread(buf, 1, size, wavin);
		if(nread != size){
        		throw std::invalid_argument( "received negative value" );
		}
		else{
			std::cout << buf << std::endl;
		}
		
		fclose(wavin);
		
		auto audioData = std::make_unique<AudioData>();
		emit(audioData);
	}

}  // namespace module::input
