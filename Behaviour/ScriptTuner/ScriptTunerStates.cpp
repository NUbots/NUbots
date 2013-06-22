#include "ScriptTunerStates.h"
#include "ScriptTunerProvider.h"
#include "Framework/darwin/Framework/include/JointData.h"
#include "Motion/Kicks/MotionScript2013.h"

using std::vector;
using std::string;



ScriptTunerState::ScriptTunerState(ScriptTunerProvider* provider) : 
    ScriptTunerSubState(provider), string_id_to_int_id(), script()
{
    std::cout<< "==================================================="<< std::endl;
    std::cout<< "--------------Welcome to Script Tuner--------------"<< std::endl;
    std::cout<< "==================================================="<< std::endl;
    m_script_active = false;
    m_file_path = (CONFIG_DIR + std::string("/Motion/Scripts/"));
    m_actionators_data = Blackboard->Actions;
    m_sensors_data = Blackboard->Sensors;
}


void ScriptTunerState::doState()
{
    std::cout<< "Load Script - Type File Name (script must be in "
             << m_file_path << "): "<< std::endl;
    char file[256];
    std::cin.getline(file,256);
    m_file_name = (string)file;

    if(!m_file_name.compare("exit") or !m_file_name.compare("quit")) {
        std::cout<< "Shutting down script tuner."<< std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        std::cout<< "Just kidding. That would be useless."<< std::endl;
        return;
    }

    //! Sets m_file_name, and use this afterwards.
    auto loaded = loadScript(m_file_name);

    if(!loaded) {
        std::cout <<"File not found! Try again!" << std::endl;
        return;
    }

    bool exit_script_loop = false;
    while(!exit_script_loop) {
        std::cout<< "==================================================="<< std::endl;
        std::cout<<"Script \""<< m_file_name << "\" loaded successfully."<< std::endl;
        std::cout<< "Play script or edit? (Type \"play\" or \"edit\")"<< std::endl;
        
        char str[256];
        std::cin.getline(str, 256);
        auto command = ScriptTunerCommand::ParseCommand(str);

        switch(command.command_type())
        {
            case ScriptTunerCommand::CommandType::kPlay: {
                HandlePlayCommand(command);
            } break;
            case ScriptTunerCommand::CommandType::kEdit: {
                HandleEditCommand(command);
            } break;
            case ScriptTunerCommand::CommandType::kExit: {
                std::cout<< "Exiting Script..."<< std::endl;  
                std::cout<< "==================================================="<< std::endl;              
                exit_script_loop = true;
            } break;
            default: {
                PrintCommandError(command);
            } break;
        }
    }
}

void ScriptTunerState::PrintCommandError(ScriptTunerCommand command)
{
    if(command.command_type() == ScriptTunerCommand::CommandType::kUnknown)
    {
        std::cout << "Error: Invalid command." << std::endl;
    }
    else
    {
        std::cout << "Error: The command can not be used at this time" << std::endl;
    }
}

void ScriptTunerState::HandleEditCommand(ScriptTunerCommand command)
{
    std::cout<<"Script \""<< m_file_name << "\" loaded successfully for editing."<< std::endl; 
    std::cout<< "---------------------------------------------------"<< std::endl;                   
    
    m_script_active = true;
    
    while(m_script_active) { 
        script->ApplyCurrentFrameToRobot(m_actionators_data);

        std::cout << "Frame number " << getCurrentFrameNumber() 
                  << " out of " << script->GetFrameCount()
                  << " applied." << std::endl;
        std::cout << "Frame duration is "<< durationOfCurrentFrame()
                  << " seconds." << std::endl;               
        std::cout << "---------------------------------------------------" << std::endl;

        editCurrentFrame();
    }
}

void ScriptTunerState::editCurrentFrame() {
    while(true)
    {
        char str[256];
        std::cin.getline(str, 256);
        auto command = ScriptTunerCommand::ParseCommand(str);

        switch(command.command_type())
        {
            case ScriptTunerCommand::CommandType::kSaveFrame: {
                HandleSaveFrameCommand(command);
            } break;
            case ScriptTunerCommand::CommandType::kSaveScript: {
                HandleSaveScriptCommand(command);
            } break;
            case ScriptTunerCommand::CommandType::kExit: {
                HandleExitScriptCommand(command);
                return;
            } break;
            case ScriptTunerCommand::CommandType::kNextFrame: {
                HandleNextFrameCommand(command);
            } break;
            case ScriptTunerCommand::CommandType::kNewFrame: {
                HandleNewFrameCommand(command);
            } break;
            case ScriptTunerCommand::CommandType::kFrameSeek: {
                HandleFrameSeekCommand(command);
            } break;
            case ScriptTunerCommand::CommandType::kFrameDuration: {
                HandleFrameDurationCommand(command);
            } break;
            case ScriptTunerCommand::CommandType::kAllOn: {
                HandleAllOnCommand(command);
            } break;
            case ScriptTunerCommand::CommandType::kJointOff: {
                HandleJointOffCommand(command);
            } break;
            case ScriptTunerCommand::CommandType::kJointOn: {
                HandleJointOnCommand(command);
            } break;
            case ScriptTunerCommand::CommandType::kJointPosition: {
                HandleJointPositionCommand(command);
            } break;
            case ScriptTunerCommand::CommandType::kJointGain: {
                HandleJointGainCommand(command);
            } break;
            case ScriptTunerCommand::CommandType::kJointPositionGain: {
                HandleJointPositionGainCommand(command);
            } break;
            default: {
                PrintCommandError(command);
            } break;
        }
    }
}

void ScriptTunerState::HandleSaveFrameCommand(ScriptTunerCommand command)
{
    std::cout << "Saving manually adjusted motor positions. It is recommended all torques are on during saving."<< std::endl;
    saveManuallyMovedMotors();
}

void ScriptTunerState::HandleSaveScriptCommand(ScriptTunerCommand command)
{
    if(motors_to_be_saved.size() != 0){
        std::cout << "!SAVE FAILED - Save manually moved motors to script first!"<< std::endl;
    }else if(saveScriptToFile(m_file_name)){
        std::cout << "Script \""<< m_file_name << "\" saved."<< std::endl;
    } else {
        std::cout << "!!!!!!!!!!! SAVING FAILED !!!!!!!!!!!."<< std::endl;
    }
}

void ScriptTunerState::HandleExitScriptCommand(ScriptTunerCommand command)
{
    std::cout << "Exiting." << std::endl;
    m_script_active = false;
}

void ScriptTunerState::HandleNextFrameCommand(ScriptTunerCommand command)
{
    std::cout << "Moving to next frame."<< std::endl;
    script->SeekFrame(getCurrentFrameNumber() +1);
}

void ScriptTunerState::HandleNewFrameCommand(ScriptTunerCommand command)
{
    std::cout << "Adding new frame."<< std::endl;
    // Second_argument should be time to complete the new frame.
    // New frame should be identical to previous.
    script->DuplicateFrame(getCurrentFrameNumber());
}

void ScriptTunerState::HandleFrameSeekCommand(ScriptTunerCommand command)
{
    script->SeekFrame(command.frame_number());
}

void ScriptTunerState::HandleFrameDurationCommand(ScriptTunerCommand command)
{
    std::cout << "Setting current frame duration to " << command.duration() 
              << "." << std::endl;
    auto* current_frame = script->GetCurrentFrame();
    current_frame->SetDuration(command.duration());
}

void ScriptTunerState::HandleAllOnCommand(ScriptTunerCommand command)
{
    std::cout << "Turning all motor torques on."<< std::endl;
    turnOnAllMotors();
}

void ScriptTunerState::HandleJointOnCommand(ScriptTunerCommand command)
{
    turnOnMotor(command.joint_number());
}

void ScriptTunerState::HandleJointOffCommand(ScriptTunerCommand command)
{
    turnOffMotor(command.joint_number());
}

void ScriptTunerState::HandleJointPositionCommand(ScriptTunerCommand command)
{
    try {
        changeMotorPosition(command.joint_number(), command.position());
    } catch (const std::out_of_range& e) {
        std::cout << "There is no such motor. Try again."<< std::endl;
    }  
}

void ScriptTunerState::HandleJointGainCommand(ScriptTunerCommand command)
{
    try {
        changeMotorGain(command.joint_number(), command.gain());
    } catch (const std::out_of_range& e) {
        std::cout << "There is no such motor. Try again."<< std::endl;
    }  
}

void ScriptTunerState::HandleJointPositionGainCommand(ScriptTunerCommand command)
{
    try {
        changeMotorPosition(command.joint_number(), command.position());
        changeMotorGain(command.joint_number(), command.gain());
    } catch (const std::out_of_range& e) {
        std::cout << "There is no such motor. Try again." << std::endl;
    }  
}

bool ScriptTunerState::loadScript(string filename)
{    
    script = MotionScript2013::LoadOldScript(m_file_path + filename);
    return script != nullptr;
}

void ScriptTunerState::saveManuallyMovedMotors()
{
    MotionScriptFrame* frame = script-> GetCurrentFrame();
    for(int i = 0; i<motors_to_be_saved.size();i++){      

        if(motorTorqueIsOff(motors_to_be_saved[i])){
            ScriptJointDescriptor descriptor;
            frame->GetDescriptor(motors_to_be_saved[i],&descriptor);//Sets descriptor pointer
            descriptor.SetPosition(getMotorPosition(motors_to_be_saved[i]));
            frame->AddDescriptor(motors_to_be_saved[i],descriptor);
        }  else {
            std::cout<< "Motor "<< motors_to_be_saved[i]<< "still has torque off!"<<std::endl;
        }

    }
}

bool ScriptTunerState::saveScriptToFile(string filename){
    return MotionScript2013::SaveToConfigSystem(*(script),m_file_path+filename);
}

void ScriptTunerState::HandlePlayCommand(ScriptTunerCommand command)
{
    std::cout << "Playing script from beginning in real time."<< std::endl;
    std::cout<< "==================================================="<< std::endl;
    script->StartScript(m_actionators_data);
}


void ScriptTunerState::changeMotorPosition(int  motor_id, float pos_change){
    //DEBUG TEXT:
    std::cout<< "Motor "<< motor_id <<" position changed by "<< pos_change <<" radians."<< std::endl;

    MotionScriptFrame* current_frame = script->GetCurrentFrame();
    ScriptJointDescriptor descriptor;
    current_frame->GetDescriptor(motor_id, &descriptor);                
    descriptor.SetPosition(descriptor.GetPosition()+pos_change);                
    current_frame->AddDescriptor(motor_id,descriptor);
}



void ScriptTunerState::changeMotorGain(int  motor_id, float gain_change){
    //DEBUG TEXT:
    std::cout<< "Motor "<< motor_id << " gain changed by "<< gain_change<< std::endl;

    MotionScriptFrame* current_frame = script->GetCurrentFrame();
    ScriptJointDescriptor descriptor;
    current_frame->GetDescriptor(motor_id, &descriptor);                
    descriptor.SetGain(descriptor.GetGain()+gain_change);                
    current_frame->AddDescriptor(motor_id,descriptor);
}

void ScriptTunerState::turnOffMotor(int  motor_id){
    //DEBUG TEXT:
    std::cout<< "Motor "<< motor_id << " torque off."<< std::endl;
    MotionScriptFrame* current_frame = script->GetCurrentFrame();
    ScriptJointDescriptor descriptor;
    current_frame->GetDescriptor(motor_id, &descriptor);                
    descriptor.SetDisable(true);                
    current_frame->AddDescriptor(motor_id,descriptor);

}

void ScriptTunerState::turnOnMotor(int  motor_id){
    //DEBUG TEXT:
    std::cout<< "Motor "<< motor_id << " torque on."<< std::endl;
    motors_to_be_saved.push_back(motor_id);

    MotionScriptFrame* current_frame = script->GetCurrentFrame();
    ScriptJointDescriptor descriptor;
    current_frame->GetDescriptor(motor_id, &descriptor);                
    descriptor.SetGain(false);                
    current_frame->AddDescriptor(motor_id,descriptor);    
}
    
int ScriptTunerState::getCurrentFrameNumber(){
    return script->GetCurrentFrameIndex();
}

float ScriptTunerState::durationOfCurrentFrame(){
    MotionScriptFrame* current_frame = script->GetCurrentFrame();
    float duration = current_frame->GetDuration();
    return duration;
}

float ScriptTunerState::getMotorPosition(int motor_id){
    float current_position;
    m_sensors_data->getPosition(
        MotionScriptFrame::MapServoIdToNUDataId(motor_id),
        current_position);
    return current_position;
}

bool ScriptTunerState::motorTorqueIsOff(int motor_id){
    auto* current_frame = script->GetCurrentFrame();
    ScriptJointDescriptor descriptor;
    return descriptor.GetDisable(); 
}

void ScriptTunerState::turnOnAllMotors(){

    for(auto key_value:string_id_to_int_id) {
        auto motor_id = key_value.second;

        if(motorTorqueIsOff(motor_id)) {
            turnOnMotor(motor_id);
        }
    }
}