#include "ScriptTunerStates.h"
#include "ScriptTunerProvider.h"
#include "Framework/darwin/Framework/include/JointData.h"
#include "Motion/Kicks/MotionScript2013.h"

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
    m_script_active = false;
    m_file_path = (CONFIG_DIR + std::string("/Motion/Scripts"));
    m_actionators_data = Blackboard->Actions;
    m_sensors_data = Blackboard->Sensors;
}


void ScriptTunerState::doState()
{
    
    std::cout<< "Load Script - Type File Name (script must be in "<<m_file_path<<"): "<< std::endl;
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
            std::cout<< "==================================================="<< std::endl;
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
                std::cout<< "==================================================="<< std::endl;
                playScript();
            }else if(first_argument.compare("edit")==0){
                std::cout<<"Script \""<< m_file_name << "\" loaded successfully for editing."<< std::endl; 
                std::cout<< "---------------------------------------------------"<< std::endl;                   
                
                m_script_active =true;
                applyFrameToRobot();
                while(m_script_active){                    
                    std::cout<<"Frame number "<<getCurrentFrameNumber()<<" out of " << totalNumberOfFrames()<<" applied."<<std::endl;
                    std::cout<<"Frame duration is "<< durationOfCurrentFrame()<< " seconds."<<std::endl;               
                    std::cout<< "---------------------------------------------------"<< std::endl;

                    editCurrentFrame();
                }
            } else if((first_argument.compare("exit") ==0) or (first_argument.compare("quit") == 0) or (first_argument.compare("q") == 0) ){
                std::cout<< "Exiting Script..."<< std::endl;  
                std::cout<< "==================================================="<< std::endl;              
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
            applyFrameToRobot();
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
            applyFrameToRobot();
            break;
        }else if(first_argument.compare("newframe")==0){
            std::cout << "Adding new frame."<< std::endl;
            addFrame(second_argument);//second_argument should be time to complete the new frame. New frame should be identical to previous.
            applyFrameToRobot();
            break;        
        }else if(first_argument.compare("seek")==0){
            interpretSeekCommand(second_argument);
            applyFrameToRobot();
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
    script = MotionScript2013::LoadFromConfigSystem(m_file_path+filename);
    return (bool)script;
}

void ScriptTunerState::applyFrameToRobot(){
    script->ApplyCurrentFrameToRobot(m_actionators_data);
}

void ScriptTunerState::saveManuallyMovedMotors(){
    MotionScriptFrame* frame = script-> GetCurrentFrame();
    for(int i = 0; i<motors_to_be_saved.size();i++){      

        if(motorTorqueIsOn(motors_to_be_saved[i])){
            ScriptJointDescriptor* descriptor;
            frame->GetDescriptor(motors_to_be_saved[i],descriptor);//Sets descriptor pointer
            descriptor->SetServoId(motors_to_be_saved[i]);
            descriptor->SetPosition(getMotorPosition(motors_to_be_saved[i]));
        }  else {
            std::cout<< "Motor "<< motors_to_be_saved[i]<< "still has torque off!"<<std::endl;
        }

    }
}

bool ScriptTunerState::saveScriptToFile(string filename){
    return MotionScript2013::SaveToConfigSystem(*(script),m_file_path+filename);
}
void ScriptTunerState::addFrame(string argument){
    script->InsertFrame(getCurrentFrameNumber(),script->GetCurrentFrame());
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
    script->SeekFrame(frame_number);
}

void ScriptTunerState::exitScript(){
    m_script_active = false;
}

void ScriptTunerState::playScript(){
    script->StartScript(m_actionators_data);
}

void ScriptTunerState::applyRequestToMotors(string parameters){
    std::stringstream command;
    command << parameters;
    
    string motor_id_string;
    command >> motor_id_string;
        try {
            int  motor_id = string_id_to_int_id.at(motor_id_string); // map::at throws an out-of-range if no key (C++11 feature)
            
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


void ScriptTunerState::changeMotorPosition(int  motor_id, float pos_change){
    //DEBUG TEXT:
    std::cout<< "Motor "<< motor_id <<" position changed by "<< pos_change <<" radians."<< std::endl;

    MotionScriptFrame* current_frame = script->GetCurrentFrame();
    ScriptJointDescriptor* descriptor;
    current_frame->GetDescriptor(motor_id, descriptor);                
    descriptor->SetPosition(descriptor->GetPosition()+pos_change);                
    current_frame->AddDescriptor(motor_id,*descriptor);
}



void ScriptTunerState::changeMotorGain(int  motor_id, float gain_change){
    //DEBUG TEXT:
    std::cout<< "Motor "<< motor_id << " gain changed by "<< gain_change<< std::endl;

    MotionScriptFrame* current_frame = script->GetCurrentFrame();
    ScriptJointDescriptor* descriptor;
    current_frame->GetDescriptor(motor_id, descriptor);                
    descriptor->SetGain(descriptor->GetGain()+gain_change);                
    current_frame->AddDescriptor(motor_id,*descriptor);
}

void ScriptTunerState::turnOffMotor(int  motor_id){
    //DEBUG TEXT:
    std::cout<< "Motor "<< motor_id << " torque off."<< std::endl;
    //Get current motor position.    
    
    
    m_actionators_data->add(MotionScriptFrame::MapServoIdToNUDataId(motor_id),
                            100,//This value should be arbitrary (time in ms to execute)
                            getMotorPosition(motor_id),
                            0);//Gain to zero for motor off

}

void ScriptTunerState::turnOnMotor(int  motor_id){
    //DEBUG TEXT:
    std::cout<< "Motor "<< motor_id << " torque on."<< std::endl;
    motors_to_be_saved.push_back(motor_id);
    //Get current motor position.
    getMotorPosition(motor_id);
    //Get current gain as dictated by script.
    MotionScriptFrame* current_frame = script->GetCurrentFrame();
    ScriptJointDescriptor* descriptor;
    current_frame->GetDescriptor(motor_id, descriptor);   

    m_actionators_data->add(MotionScriptFrame::MapServoIdToNUDataId(motor_id),
                            100,
                            getMotorPosition(motor_id),
                            descriptor->GetGain());
}
    

    
int ScriptTunerState::getCurrentFrameNumber(){
    return script->GetCurrentFrameIndex();
}
    
int ScriptTunerState::totalNumberOfFrames(){
    return script->GetFrameCount();
}
    
            float ScriptTunerState::durationOfCurrentFrame(){
                MotionScriptFrame* current_frame = script->GetCurrentFrame();
                float duration = current_frame->GetTime();
                return duration;
            }

bool ScriptTunerState::scriptIsActive(){
    return m_script_active;
}

            void ScriptTunerState::setCurrentFrameDuration(string duration_string){
                std::stringstream stream;
                stream << duration_string;
                float duration;
                if(stream >> duration){
                    std::cout<< "Setting current frame duration to "<< duration <<"."<< std::endl;
                    MotionScriptFrame* current_frame = script->GetCurrentFrame();
                    current_frame->SetTime(duration);

                } else {
                    std::cout<< "Invalid duration time." << std::endl;
                }
            }

float ScriptTunerState::getMotorPosition(int motor_id){
    float current_position;
    m_sensors_data->getPosition(MotionScriptFrame::MapServoIdToNUDataId(motor_id),current_position);
    return current_position;
}

bool ScriptTunerState::motorTorqueIsOn(int motor_id){
    vector<float> pos_and_gain;
    m_sensors_data->getTarget(MotionScriptFrame::MapServoIdToNUDataId(motor_id),pos_and_gain);
    std::cout<< "Checking gain. Gain is " << pos_and_gain[1]<< " so torque is "<< (pos_and_gain[1]==0?"off.":"on.")<< std::endl;
    return (pos_and_gain[1]==0); 
}