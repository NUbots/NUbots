#include "WavReader.hpp"

#include <cstdint>
#include <fstream>
#include <iostream>
#include <string>
#include <stdio.h>

#include "extension/Configuration.hpp"

#include "message/support/GlobalConfig.hpp"
#include "message/input/AudioData.hpp"

namespace module::input {

    using extension::Configuration;
    using message::input::AudioData;
    
    WavReader::WavReader(std::unique_ptr<NUClear::Environment> environment)
        : Reactor(std::move(environment)), config{} {
	
	//uncomment if you want to check if the path exists.
	//bool filepathExists = std::filesystem::exists("/home/nubots/NUbots/module/input/WavReader/data/audio/test.wav");;
	//std::cout << filepathExists << std::endl;
            
        
        on<Configuration>("WavReader.yaml").then([this](const Configuration& cfg) {
            // Use configuration here from file WavReader.yaml

            this->log_level = cfg["log_level"].as<NUClear::LogLevel>();
            this->config.wav_path = cfg["wav_path"].as<std::string>(); //config is the name of the struct
            std::cout << config.wav_path << std::endl;
            log<NUClear::DEBUG>(config.wav_path);
		readWav();
        });
		
	//This reactor receives audiodata from emission
	//on<Trigger<AudioData>>().then([this] (AudioData& audioData ) {
	    // reactor code
	//});

    }
    
    void WavReader::readWav(){
		FILE *wavin;
		char* buf = new char();
		int nread = 0; 

		wavin = fopen(config.wav_path.c_str(), "rb");
		fseek(wavin, 44, SEEK_SET);
	    	

		long size = ftell(wavin);
		rewind(wavin);
			    	
		while (!feof(wavin)) {
		//for(int i = 0; i< 20; i++){
			nread = fread(buf, 1, size, wavin); //nread is the audio's chunk which has been read from the stream 
			
			
			if(nread != size){
        			throw std::invalid_argument( "received negative value" );
			}
			
			else{
				std::cout << "New buf created printing first 10 values of it.\n" << std::endl;
				for(int i = 0; i < size; i++){
					std::cout << buf[i] << std::endl;
				}
			}
		}
		
		fclose(wavin);
		
		auto audioData = std::make_unique<AudioData>();
		emit(audioData);
	}

}  // namespace module::input
