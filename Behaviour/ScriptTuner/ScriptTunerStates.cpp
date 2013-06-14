#include "ScriptTunerStates.h"

using std::vector;
using std::string;

void ScriptTunerState::doState()
{


    std::cout<< "Load Script - Type File Name (Must be in ~/nubot/Config/Darwin/Motion/Scripts): "<< std::endl;
    char file[256];
    std::cin.getline(file,256);
    string filename(file);

    if((filename.compare("exit") ==0) or (filename.compare("quit") == 0) ){
        std::cout<< "Shutting down script tuner."<< std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        std::cout<< "Just kidding. That would be useless."<< std::endl;
        return;
    }

    bool loaded = loadScript(filename);

    if(loaded){
        while(script.isActive()){
            applyCurrentFrameToRobot();
            std::cout << "Script \""<< filename << "\" loaded successfully."<< std::endl;
            std::cout<< "Frame number "<<script.getCurrentFrameNumber()<<" out of " << script.numberOfFrames()<<" applied."<<std::endl;
            std::cout<<"Time period for frame to complete is "<< script.timeForCompletionOfCurrentFrame()<< " seconds."<<std::endl;

            editCurrentFrame();
        }


    } else {
        std::cout <<"File not found! Try again!" << std::endl;
    }


}

void ScriptTunerState::editCurrentFrame(){
    while(true){
        std::stringstream command;
        char str[256];
        std::cin.getline(str, 256);
        command << str;
        string first_argument;
        string second_argument;
        command >> first_argument >> second_argument;

        if(first_argument.compare("saveframe")==0){
            std::cout << "Saving frame. It is recommended all torques are on during saving."<< std::endl;
            saveCurrentFrame();
        }else if(first_argument.compare("savescript")==0){
            std::cout << "Script \""<< filename << "\" saving."<< std::endl;
            saveScriptToFile(filename);
        }else if(first_argument.compare("exit")==0){
            std::cout << "Exiting."<< std::endl;
            exitScript();
            break;
        }else if(first_argument.compare("next")==0){
            std::cout << "Moving to next frame."<< std::endl;
            moveToFrame(script.getCurrentFrameNumber()+1);
            break;
        }else if(first_argument.compare("newframe")==0){
            std::cout << "Adding new frame."<< std::endl;
            addFrame(second_argument);//second_argument should be time to complete the new frame. New frame should be identical to previous.
            break;
        }else if(first_argument.compare("play")==0){
            std::cout << "Playing script starting from the current frame."<< std::endl;
            playScript();
        }else if(first_argument.compare("seek")==0){
            interpretSeekCommand(command.str());
            break;
        } else { //Otherwise we interpret it as a motor position request
            std::cout << "Performing motor manipulation."<<first_argument<<" "<<command.str()<< std::endl;
            applyRequestToMotors(first_argument,command.str());
        }
    }
}

bool ScriptTunerState::loadScript(string filename){
    std::cout << "Oh wait this isn't implemented yet."<< std::endl;
    return true;
}

void ScriptTunerState::applyFrameToRobot(){
    std::cout << "Oh wait this isn't implemented yet."<< std::endl;
}

void ScriptTunerState::saveCurrentFrame(){
    std::cout << "Oh wait this isn't implemented yet."<< std::endl;
}

void ScriptTunerState::saveScriptToFile(string filename){
    std::cout << "Oh wait this isn't implemented yet."<< std::endl;
}
void ScriptTunerState::addFrame(string argument){
std::cout << "Oh wait this isn't implemented yet."<< std::endl;
}

void ScriptTunerState::interpretSeekCommand(string command){
    int frame_number;
    stringstream s;
    s << command;
    if(!(s >>frame_number)){
        std::cout << "Please input an integer as the second argument."<< std::endl;
        return;
    }
    std::cout << "Moving to frame "<<frame_number<<"."<< std::endl;
    moveToFrame(frame_number);
}

void ScriptTunerState::moveToFrame(int frame_number){
    std::cout << "Oh wait this isn't implemented yet."<< std::endl;
}

void ScriptTunerState::exitScript(){
    std::cout << "Oh wait this isn't implemented yet."<< std::endl;
}

void ScriptTunerState::playScript(){
    std::cout << "Oh wait this isn't implemented yet."<< std::endl;
}

void ScriptTunerState::applyRequestToMotors(string motorID,string other_parameters){
    std::cout << "Oh wait this isn't implemented yet."<< std::endl;
}

