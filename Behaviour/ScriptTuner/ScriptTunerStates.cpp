#include "ScriptTunerStates.h"

using std::vector;
using std::string;



ScriptTunerState(ScriptTunerProvider* provider) : ScriptTunerSubState(provider) :string_id_to_int_id() {
    string_id_to_int_id["RSP"] = (int)JointData::ID_R_SHOULDER_PITCH;
    string_id_to_int_id["LSP"] = (int)JointData::ID_L_SHOULDER_PITCH;
    string_id_to_int_id["RSR"] = (int)JointData::ID_R_SHOULDER_ROLL;
    string_id_to_int_id["LSR"] = (int)JointData::ID_L_SHOULDER_ROLL;
    string_id_to_int_id["RE"] = (int)JointData::ID_R_ELBOW;
    string_id_to_int_id["LE"] = (int)JointData::ID_L_ELBOW;
    string_id_to_int_id["RHY"] = (int)JointData::ID_R_HIP_YAW;
    string_id_to_int_id["LHY"] = (int)JointData::ID_L_HIP_YAW;
    string_id_to_int_id["RHR"] = (int)JointData::ID_R_HIP_ROLL;
    string_id_to_int_id["LHR"] = (int)JointData::ID_L_HIP_ROLL;
    string_id_to_int_id["RHP"] = (int)JointData::ID_R_HIP_PITCH;
    string_id_to_int_id["LHP"] = (int)JointData::ID_L_HIP_PITCH;
    string_id_to_int_id["RK"] = (int)JointData::ID_R_KNEE;
    string_id_to_int_id["LK"] = (int)JointData::ID_L_KNEE;
    string_id_to_int_id["RAP"] = (int)JointData::ID_R_ANKLE_PITCH;
    string_id_to_int_id["LAP"] = (int)JointData::ID_L_ANKLE_PITCH;
    string_id_to_int_id["RAR"] = (int)JointData::ID_R_ANKLE_ROLL;
    string_id_to_int_id["LAR"] = (int)JointData::ID_L_ANKLE_ROLL;
    string_id_to_int_id["HP"] = (int)JointData::ID_HEAD_PAN;
    string_id_to_int_id["HT"] = (int)JointData::ID_HEAD_TILT;
}


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

    auto loaded = loadScript(filename);

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
            saveManuallyMovedMotors();
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
            std::cout << "Performing motor manipulation. "<<first_argument<<" "<<command.str()<< std::endl;
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

void ScriptTunerState::saveManuallyMovedMotors(){
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
    std::stringstream s;
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

void ScriptTunerState::applyRequestToMotors(string motor_id_string,string other_parameters){
    std::stringstream command;
    command << other_parameters;
    try {
        int motor_id = string_id_to_int_id.at(motor_id_string); // map::at throws an out-of-range if no key (C++11 feature)
        
        int positional_change, gain_change;
        if(command >> positional_change){
            changeMotorPosition(motor_id, positional_change);
            if(command >> gain_change){
                changeMotorGain(motor_id, gain_change);
            }
            return;
        } 

        string instruction;
        command >> instruction;
        if(instruction.compare("on")){
            turnOffMotor(motor_id);
            return;
        }else if(instruction.compare("off")){
            turnOnMotor(motor_id);
            return;
        }
        
        std::cout<<"Invalid Manipulation. Try <Motor Name> (<\"on\"/\"off\">) or (<position_change> <gain_change>)"<<std::endl;

    } catch (const std::out_of_range& e) {
        std::cout << "There is no such motor. Please try again"<< std::endl;
    }    
}


void changeMotorPostition(int motor_id, float pos_change){
    std::cout<< "Motor "<< motor_id << " changed by "<< pos_change << std::endl;
}

void changeMotorGain(int motor_id, float gain_change){
    std::cout<< "Motor "<< motor_id << " gain changed by"<< gain_change<< std::endl;
}

void turnOffMotor(int motor_id){
    std::cout<< "Motor "<< motor_id << " torque off."<< << std::endl;
}

void turnOnMotor(int motor_id){
    std::cout<< "Motor "<< motor_id << " torque on."<< << std::endl;
}