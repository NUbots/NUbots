#include "ScriptTunerStates.h"
#include "ScriptTunerProvider.h"
#include "Framework/darwin/Framework/include/JointData.h"

using std::vector;
using std::string;



ScriptTunerState::ScriptTunerState(ScriptTunerProvider* provider) : ScriptTunerSubState(provider),string_id_to_int_id(), script() {
    string_id_to_int_id["RSP"] = (int)Robot::JointData::ID_R_SHOULDER_PITCH;
    string_id_to_int_id["LSP"] = (int)Robot::JointData::ID_L_SHOULDER_PITCH;
    string_id_to_int_id["RSR"] = (int)Robot::JointData::ID_R_SHOULDER_ROLL;
    string_id_to_int_id["LSR"] = (int)Robot::JointData::ID_L_SHOULDER_ROLL;
    string_id_to_int_id["RE"] = (int)Robot::JointData::ID_R_ELBOW;
    string_id_to_int_id["LE"] = (int)Robot::JointData::ID_L_ELBOW;
    string_id_to_int_id["RHY"] = (int)Robot::JointData::ID_R_HIP_YAW;
    string_id_to_int_id["LHY"] = (int)Robot::JointData::ID_L_HIP_YAW;
    string_id_to_int_id["RHR"] = (int)Robot::JointData::ID_R_HIP_ROLL;
    string_id_to_int_id["LHR"] = (int)Robot::JointData::ID_L_HIP_ROLL;
    string_id_to_int_id["RHP"] = (int)Robot::JointData::ID_R_HIP_PITCH;
    string_id_to_int_id["LHP"] = (int)Robot::JointData::ID_L_HIP_PITCH;
    string_id_to_int_id["RK"] = (int)Robot::JointData::ID_R_KNEE;
    string_id_to_int_id["LK"] = (int)Robot::JointData::ID_L_KNEE;
    string_id_to_int_id["RAP"] = (int)Robot::JointData::ID_R_ANKLE_PITCH;
    string_id_to_int_id["LAP"] = (int)Robot::JointData::ID_L_ANKLE_PITCH;
    string_id_to_int_id["RAR"] = (int)Robot::JointData::ID_R_ANKLE_ROLL;
    string_id_to_int_id["LAR"] = (int)Robot::JointData::ID_L_ANKLE_ROLL;
    string_id_to_int_id["HP"] = (int)Robot::JointData::ID_HEAD_PAN;
    string_id_to_int_id["HT"] = (int)Robot::JointData::ID_HEAD_TILT;
    std::cout<< "==================================================="<< std::endl;
    std::cout<< "--------------Welcome to Script Tuner--------------"<< std::endl;
    std::cout<< "==================================================="<< std::endl;
}


void ScriptTunerState::doState()
{
    NUActionatorsData* m_actions = Blackboard->Actions;
    std::cout<< "Load Script - Type File Name (Must be in "<<filepath"): "<< std::endl;
    char file[256];
    std::cin.getline(file,256);
    string filename = (string)file;

    if((filename.compare("exit") ==0) or (filename.compare("quit") == 0) ){
        std::cout<< "Shutting down script tuner."<< std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        std::cout<< "Just kidding. That would be useless."<< std::endl;
        return;
    }

    auto loaded = loadScript(filename);//Sets m_file_name, and use this afterwards.


    if(loaded){
        bool done = false;
        while(!done){
            std::cout<<"Script \""<< m_file_name << "\" loaded successfully."<< std::endl;
            std::cout<< "Play script or edit? (Type \"play\" or \"edit\")"<< std::endl;
            std::stringstream command;
            char str[256];
            std::cin.getline(str, 256);
            command << str;
            string first_argument;
            command >> first_argument;

            if(first_argument.compare("play")==0){            
                std::cout << "Playing script from beginning in real time."<< std::endl;
                playScript();
            }else if(first_argument.compare("edit")==0){
                std::cout<<"Script \""<< m_file_name << "\" loaded successfully for editing."<< std::endl;                    
                
                while(scriptIsActive()){
                    applyCurrentFrameToRobot();
                    std::cout<<"Frame number "<<getCurrentFrameNumber()<<" out of " << totalNumberOfFrames()<<" applied."<<std::endl;
                    std::cout<<"Time period for frame to complete is "<< durationOfCurrentFrame()<< " seconds."<<std::endl;               
                  
                    editCurrentFrame();
                }
            } else if((first_argument.compare("exit") ==0) or (first_argument.compare("quit") == 0) or (first_argument.compare("q") == 0) ){
                std::cout<< "Exiting Script..."<< std::endl;                
                done = true;
            }
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
            std::cout << "Saving manually adjusted motor positions. It is recommended all torques are on during saving."<< std::endl;
            saveManuallyMovedMotors();
        }else if(first_argument.compare("savescript")==0){
            if(saveScriptToFile(m_file_name)){
                std::cout << "Script \""<< m_file_name << "\" saved."<< std::endl;
            } else {
                std::cout << "!!!!!!!!!!! SAVING FAILED !!!!!!!!!!!."<< std::endl;
            }
            
        }else if(first_argument.compare("exit")==0 or (first_argument.compare("quit") == 0) or (first_argument.compare("q") == 0) ){
            std::cout << "Exiting."<< std::endl;
            exitScript();
            break;
        }else if(first_argument.compare("next")==0){
            std::cout << "Moving to next frame."<< std::endl;
            moveToFrame(getCurrentFrameNumber()+1);
            break;
        }else if(first_argument.compare("newframe")==0){
            std::cout << "Adding new frame."<< std::endl;
            addFrame(second_argument);//second_argument should be time to complete the new frame. New frame should be identical to previous.
            break;        
        }else if(first_argument.compare("seek")==0){
            interpretSeekCommand(second_argument);
            break;
        }else if(first_argument.compare("duration")==0){
            setCurrentFrameDuration(second_argument);
            break;
        } else { //Otherwise we interpret it as a motor position request
            std::cout << "Performing motor manipulation. "<<command.str()<< std::endl;
            applyRequestToMotors(command.str());
        }
    }
}

bool ScriptTunerState::loadScript(string filename){    
    script = MotionScript2013::Load(path,filename);
    return (bool)script;
}

void ScriptTunerState::applyFrameToRobot(){
    script->ApplyCurrentFrameToRobot(m_actions);
}

void ScriptTunerState::saveManuallyMovedMotors(){
    std::cout << "Oh wait this isn't implemented yet."<< std::endl;
}

bool ScriptTunerState::saveScriptToFile(string filename){
    return MotionScript2013::Save(script/*Dereference? Takes a Motionscript2013&*/,m_file_path,filename);
}
void ScriptTunerState::addFrame(string argument){
std::cout << "Oh wait this isn't implemented yet."<< std::endl;
}

void ScriptTunerState::interpretSeekCommand(string frame_number_string){
    int frame_number;
    std::stringstream s;
    s << frame_number_string;
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

void ScriptTunerState::applyRequestToMotors(string parameters){
    std::stringstream command;
    command << parameters;
    
    string motor_id_string;
    command >> motor_id_string;
        try {
            int motor_id = string_id_to_int_id.at(motor_id_string); // map::at throws an out-of-range if no key (C++11 feature)
            
            float positional_change, gain_change;
            if(command >> positional_change){
                changeMotorPosition(motor_id, positional_change);
                if(command >> gain_change){
                    changeMotorGain(motor_id, gain_change);
                }
                return;
            } 

            string instruction;
            command >> instruction;
            if(instruction.compare("off")==0){
                turnOffMotor(motor_id);
                return;
            }else if(instruction.compare("on")==0){
                turnOnMotor(motor_id);
                return;
            }
            
            std::cout<<"Invalid Manipulation. Try <Motor Name> (<\"on\"/\"off\">) or (<position_change> <gain_change>)."<<std::endl;

        } catch (const std::out_of_range& e) {
            std::cout << "There is no such motor. Try again."<< std::endl;
        }    
}


void ScriptTunerState::changeMotorPosition(int motor_id, float pos_change){
    //DEBUG TEXT:
    std::cout<< "Motor "<< motor_id << " position changed by "<< pos_change <<" radians."<< std::endl;
}

void ScriptTunerState::changeMotorGain(int motor_id, float gain_change){
    //DEBUG TEXT:
    std::cout<< "Motor "<< motor_id << " gain changed by "<< gain_change<< std::endl;
}

void ScriptTunerState::turnOffMotor(int motor_id){
    //DEBUG TEXT:
    std::cout<< "Motor "<< motor_id << " torque off."<< std::endl;
}

void ScriptTunerState::turnOnMotor(int motor_id){
    //DEBUG TEXT:
    std::cout<< "Motor "<< motor_id << " torque on."<< std::endl;
}
    

    
int ScriptTunerState::getCurrentFrameNumber(){
    return 1;
}
    
int ScriptTunerState::totalNumberOfFrames(){
    return 1;
}
    
float ScriptTunerState::durationOfCurrentFrame(){
    return 1.0;
}

bool ScriptTunerState::scriptIsActive(){
    return true;
}

void ScriptTunerState::applyCurrentFrameToRobot(){

}

void setCurrentFrameDuration(string duration_string){
    std::stringstream stream;
    stream << duration_string;
    int duration;
    if(stream >> duration){
        std::cout<< "Setting current frame duration to "<< duration <<"."<< std::endl;
        //MAIN CODE HERE

    } else {
        std::cout<< "Invalid duration time." << std::endl;
    }
}