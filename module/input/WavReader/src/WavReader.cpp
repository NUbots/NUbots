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
		
	//make reactor onTrigger audiodata (receives audiodata from this file)
		//generate new speech recognition module
    }
    
    void WavReader::readWav(){
		FILE *wavin;
		char* buf = new char();
		int nread = 0; 
// 	    	int var = 0; //var stores some int value which depicts if a return result is partial or final, as in if a particular utterance is completed its considered  a final speech, its based on a few rules we shouldn't worry about 

		wavin = fopen(config.wav_path.c_str(), "rb");
		fseek(wavin, 44, SEEK_SET);
		
		long size = ftell (wavin);
		rewind(wavin);
		
		nread = fread(buf, 1, size, wavin); //nread is the audio's chunk which has been read from the stream 
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
