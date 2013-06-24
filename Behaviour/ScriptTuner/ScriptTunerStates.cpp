#include "ScriptTunerStates.h"
#include "ScriptTunerProvider.h"
#include "Framework/darwin/Framework/include/JointData.h"
#include "Motion/Kicks/MotionScript2013.h"


using std::vector;
using std::string;



ScriptTunerState::ScriptTunerState(ScriptTunerProvider* provider) : 
    ScriptTunerSubState(provider), 
    kSetPoseCommandGains(10), 
    script_()
{
    std::cout<< "==================================================="<< std::endl;
    std::cout<< "--------------Welcome to Script Tuner--------------"<< std::endl;
    std::cout<< "==================================================="<< std::endl;
    script_active_ = false;
    file_path_ = (CONFIG_DIR + std::string("/Motion/Scripts/"));
    actionators_data_ = Blackboard->Actions;
    sensors_data_ = Blackboard->Sensors;
}


void ScriptTunerState::doState()
{
    std::cout << "Load Script - Type File Name (script must be in "
              << file_path_ << "): "<< std::endl;
    std::cout << "> ";
    char str[256];
    std::cin.getline(str, 256);
    auto command = ScriptTunerCommand::ParseCommand(str);

    switch(command.command_type())
    {
        case ScriptTunerCommand::CommandType::kLoadScript: {
            HandleLoadScriptCommand(command);
        } break;
        case ScriptTunerCommand::CommandType::kLoadOldScript: {
            HandleLoadOldScriptCommand(command);
        } break;
        case ScriptTunerCommand::CommandType::kHelp: {
                HandleHelpCommand(command);
            } break;
        case ScriptTunerCommand::CommandType::kExit: {
            std::cout << "Shutting down script tuner." << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(2000));
            std::cout << "Just kidding. That would be useless."<< std::endl;
            return;
        } break;
        default: {
            PrintCommandError(command);
            return;
        } break;
    }

    if(script_ == nullptr)
        return;

    bool exit_script_loop = false;
    while(!exit_script_loop) {
        std::cout<< "==================================================="<< std::endl;
        std::cout<<"Script \""<< file_name_ << "\" loaded successfully."<< std::endl;
        std::cout<< "Play script or edit? (Type \"play\" or \"edit\")"<< std::endl;
        std::cout << "> ";

        char str[256];
        std::cin.getline(str, 256);
        auto command = ScriptTunerCommand::ParseCommand(str);

        switch(command.command_type())
        {
            case ScriptTunerCommand::CommandType::kPrintScript: {
                HandlePrintScriptCommand(command);
            } break;
            case ScriptTunerCommand::CommandType::kPlay: {
                HandlePlayCommand(command);
            } break;
            case ScriptTunerCommand::CommandType::kHelp: {
                HandleHelpCommand(command);
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

void ScriptTunerState::HandlePrintScriptCommand(ScriptTunerCommand command)
{
    std::cout << "Name: '" << file_name_ << "'." <<  std::endl;
    std::cout << "Duration = " << script_->GetScriptDuration() << "." <<  std::endl;
    
    int frame_count = script_->GetFrameCount();
    std::cout << "FrameCount = " << frame_count << "." <<  std::endl;

    for(int i = 0; i < frame_count; i++)
    {
        std::cout << "Frame " << i << ":" << std::endl;

        script_->SeekFrame(i);
        auto* frame = script_->GetCurrentFrame();

        double duration = frame->GetDuration();
        std::cout << "  Duration = " << duration << "." << std::endl;

        for(int j = 0; j < Robot::JointData::NUMBER_OF_JOINTS; j++)
        {
            ScriptJointDescriptor descriptor;
            bool success = frame->GetDescriptor(j, &descriptor);

            if(!success)
                continue;

            float position = descriptor.GetPosition();
            float gain = descriptor.GetGain();
            float servo_id = descriptor.GetServoId();

            std::cout << "  Joint " << j << ":" << std::endl;
            std::cout << "    servo_id = " << servo_id << "." << std::endl;
            std::cout << "    position = " << position << "." << std::endl;
            std::cout << "    gain = " << gain << "." << std::endl;
        }
    }
}

void ScriptTunerState::HandleLoadOldScriptCommand(ScriptTunerCommand command)
{
    file_name_ = command.script_path();
    script_ = MotionScript2013::LoadOldScript(file_path_ + file_name_);

    if(script_ == nullptr)
        std::cout << "File not found! Try again!" << std::endl;
}

void ScriptTunerState::HandleLoadScriptCommand(ScriptTunerCommand command)
{
    file_name_ = command.script_path();
    script_ = MotionScript2013::LoadFromConfigSystem(
        "motion.scripts." + file_name_);

    if(script_ == nullptr)
        std::cout << "Load from config system failed!" << std::endl;
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
    std::cout<<"Script \""<< file_name_ << "\" loaded successfully for editing."<< std::endl; 
    std::cout<< "---------------------------------------------------"<< std::endl;                   
    
    script_active_ = true;
    
    while(script_active_) { 
        script_->ApplyCurrentFrameToRobot(actionators_data_);      
        PrintFrameInfo();
        editCurrentFrame();
    }
}

void ScriptTunerState::editCurrentFrame() {
    std::cout << "> ";
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
        case ScriptTunerCommand::CommandType::kPreviousFrame: {
            HandlePreviousFrameCommand(command);
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
         case ScriptTunerCommand::CommandType::kAllOff: {
            HandleAllOffCommand(command);
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
        case ScriptTunerCommand::CommandType::kHelp: {
            HandleHelpCommand(command);
        } break;
        case ScriptTunerCommand::CommandType::kSetPose: {
            HandleSetPoseCommand(command);
        } break;
        case ScriptTunerCommand::CommandType::kDeleteFrame: {
            HandleDeleteFrameCommand(command);
        } break;
        default: {
            PrintCommandError(command);
        } break;
    }
    
}

void ScriptTunerState::HandleSetPoseCommand(ScriptTunerCommand command){
    float button_state;
    std::cout<<"Torques set to low. Manually set position and press centre button to save to script."<<std::endl;
    Blackboard->Sensors->getButton(NUSensorsData::MainButton,button_state);
    MotionScriptFrame* frame = script_->GetCurrentFrame();
    MotionScriptFrame temp_frame(*frame);//Copy constructor
    
    while(button_state <= 0){
        UpdateScriptMotorGoalsToCurrentPosition(&temp_frame);
        temp_frame.ApplyToRobot(actionators_data_);
        Blackboard->Sensors->getButton(NUSensorsData::MainButton,button_state);         
    }
    CopyPositionsFromTempScript(&temp_frame);
    std::cout<<"Pose saved."<<std::endl;
}

void ScriptTunerState::HandleDeleteFrameCommand(ScriptTunerCommand command){
    script_->RemoveFrame(script_->GetCurrentFrameIndex());
}

void ScriptTunerState::UpdateScriptMotorGoalsToCurrentPosition(MotionScriptFrame* frame){
    for(int motor_id = 1; motor_id<Robot::JointData::NUMBER_OF_JOINTS;motor_id++){
        ScriptJointDescriptor descriptor;          
        descriptor.SetGain(kSetPoseCommandGains);       
        // motors_to_be_saved_.push_back(motor_id);                     
        descriptor.SetPosition(getMotorPosition(motor_id));
        descriptor.SetServoId(motor_id);                
        frame->AddDescriptor(descriptor);    
    }
}

void ScriptTunerState::CopyPositionsFromTempScript(MotionScriptFrame* temp_frame){
    MotionScriptFrame* current_frame = script_->GetCurrentFrame();
    for(int motor_id = 1; motor_id<Robot::JointData::NUMBER_OF_JOINTS;motor_id++){
        ScriptJointDescriptor descriptor, temp_descriptor;
        if (!current_frame->GetDescriptor(motor_id, &descriptor)){
            std::cout << "Descriptor doesn't exist for this frame for motor "<<motor_id<< ". Not saving." << std::endl;
            continue;    
        }      

        temp_frame->GetDescriptor(motor_id, &temp_descriptor);
        float new_position = temp_descriptor.GetPosition();
        float old_position = descriptor.GetPosition();
        descriptor.SetPosition(new_position);

        std::cout << "Saving motor " << motor_id << "."
                  << " Change in position is " << new_position - old_position << " rad" 
                  // << " with gain " << descriptor.GetGain()
                  // << " and temp_gain " << temp_descriptor.GetGain()
                  << std::endl;
        
        current_frame->AddDescriptor(descriptor);
    }
}

void ScriptTunerState::PrintFrameInfo(){
    std::cout << "Frame number " << script_->GetCurrentFrameIndex()+1
              << " out of " << script_->GetFrameCount()
              << " applied." << std::endl;
    auto* current_frame = script_->GetCurrentFrame();
    std::cout << "Frame duration is "<< current_frame->GetDuration()
              << " milliseconds." << std::endl;               
    std::cout << "---------------------------------------------------" << std::endl;
}

void ScriptTunerState::HandleHelpCommand(ScriptTunerCommand command){
    ScriptTunerCommand::PrintCommandsLongHelp();
}

void ScriptTunerState::HandleSaveFrameCommand(ScriptTunerCommand command)
{
    std::cout << "Saving manually adjusted motor positions. It is recommended all torques are on during saving."<< std::endl;
    saveManuallyMovedMotors();
}

void ScriptTunerState::HandleSaveScriptCommand(ScriptTunerCommand command)
{
    if(motors_to_be_saved_.size() != 0){
        std::cout << "!SAVE FAILED - Save manually moved motors to script first!"<< std::endl;
    }else if(saveScriptToFile(file_name_)){
        std::cout << "Script \""<< file_name_ << "\" saved."<< std::endl;
    } else {
        std::cout << "!!!!!!!!!!! SAVING FAILED !!!!!!!!!!!."<< std::endl;
    }
}

void ScriptTunerState::HandleExitScriptCommand(ScriptTunerCommand command)
{
    std::cout << "Exiting." << std::endl;
    script_active_ = false;
}

void ScriptTunerState::HandleNextFrameCommand(ScriptTunerCommand command)
{
    std::cout << "Moving to next frame."<< std::endl;    
    script_->SeekFrame(script_->GetCurrentFrameIndex() + 1);
}

void ScriptTunerState::HandlePreviousFrameCommand(ScriptTunerCommand command)
{
    std::cout << "Moving to previous frame."<< std::endl;    
    script_->SeekFrame(script_->GetCurrentFrameIndex() - 1);
}

void ScriptTunerState::HandleNewFrameCommand(ScriptTunerCommand command)
{
    std::cout << "Adding new frame."<< std::endl;
    // Second_argument should be time to complete the new frame.
    // New frame should be identical to previous.
    script_->DuplicateFrame(script_->GetCurrentFrameIndex());
    script_->AdvanceToNextFrame();
}

void ScriptTunerState::HandleFrameSeekCommand(ScriptTunerCommand command)
{
    std::cout << "Seek new frame."<< std::endl;
    script_->SeekFrame(command.frame_number());    
}

void ScriptTunerState::HandleFrameDurationCommand(ScriptTunerCommand command)
{
    std::cout << "Setting current frame duration to " << command.duration() 
              << "." << std::endl;
    auto* current_frame = script_->GetCurrentFrame();
    current_frame->SetDuration(command.duration());
}

void ScriptTunerState::HandleAllOnCommand(ScriptTunerCommand command)
{
    std::cout << "Turning all motor torques on."<< std::endl;
    turnOnAllMotors();
}

void ScriptTunerState::HandleAllOffCommand(ScriptTunerCommand command)
{
    std::cout << "Turning all motor torques off."<< std::endl;
    turnOffAllMotors();
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

void ScriptTunerState::saveManuallyMovedMotors()
{
    MotionScriptFrame* frame = script_-> GetCurrentFrame();
    for(int i = 0; i < motors_to_be_saved_.size(); i++){      
        if(motorTorqueIsOff(motors_to_be_saved_[i])){
            ScriptJointDescriptor descriptor;
            frame->GetDescriptor(motors_to_be_saved_[i],&descriptor);//Sets descriptor pointer
            descriptor.SetPosition(getMotorPosition(motors_to_be_saved_[i]));
            frame->AddDescriptor(descriptor);
        }  else {
            std::cout<< "Motor "<< motors_to_be_saved_[i]<< "still has torque off!"<<std::endl;
        }
    }
}

bool ScriptTunerState::saveScriptToFile(string filename){
    return script_->SaveToConfigSystem("motion.scripts." + filename);
}

void ScriptTunerState::HandlePlayCommand(ScriptTunerCommand command)
{
    std::cout << "Playing script from beginning in real time."<< std::endl;
    std::cout<< "==================================================="<< std::endl;
    script_->PlayScript(actionators_data_);
    script_->Reset();
}


void ScriptTunerState::changeMotorPosition(int  motor_id, float pos_change){
    //DEBUG TEXT:
    std::cout<< "Motor "<< motor_id <<" position changed by "<< pos_change <<" radians."<< std::endl;

    MotionScriptFrame* current_frame = script_->GetCurrentFrame();
    ScriptJointDescriptor descriptor;
    current_frame->GetDescriptor(motor_id, &descriptor);                
    descriptor.SetPosition(descriptor.GetPosition()+pos_change);                
    current_frame->AddDescriptor(descriptor);
}



void ScriptTunerState::changeMotorGain(int  motor_id, float gain_change){
    //DEBUG TEXT:
    std::cout<< "Motor "<< motor_id << " gain changed by "<< gain_change<< std::endl;

    MotionScriptFrame* current_frame = script_->GetCurrentFrame();
    ScriptJointDescriptor descriptor;
    current_frame->GetDescriptor(motor_id, &descriptor);                
    descriptor.SetGain(descriptor.GetGain()+gain_change);                
    current_frame->AddDescriptor(descriptor);
}

void ScriptTunerState::turnOffMotor(int  motor_id){
    //DEBUG TEXT:
    std::cout<< "Motor "<< motor_id << " torque off."<< std::endl;
    MotionScriptFrame* current_frame = script_->GetCurrentFrame();
    ScriptJointDescriptor descriptor;
    current_frame->GetDescriptor(motor_id,&descriptor);                
    descriptor.SetDisable(true);                
    current_frame->AddDescriptor(descriptor);

}

void ScriptTunerState::turnOnMotor(int  motor_id){
    //DEBUG TEXT:
    std::cout<< "Motor "<< motor_id << " torque on."<< std::endl;
    motors_to_be_saved_.push_back(motor_id);
    MotionScriptFrame* current_frame = script_->GetCurrentFrame();
    ScriptJointDescriptor descriptor;
    current_frame->GetDescriptor(motor_id, &descriptor);      
    descriptor.SetPosition(getMotorPosition(motor_id));                
    current_frame->AddDescriptor( descriptor);    
}

float ScriptTunerState::getMotorPosition(int motor_id){
    float current_position;
    auto nudata_id = MotionScriptFrame::MapServoIdToNUDataId(motor_id);
    sensors_data_->getPosition(nudata_id, current_position);
    return current_position;
}

bool ScriptTunerState::motorTorqueIsOff(int motor_id){
    auto* current_frame = script_->GetCurrentFrame();
    ScriptJointDescriptor descriptor;
    current_frame->GetDescriptor(motor_id, &descriptor);
    return descriptor.GetDisable(); 
}

void ScriptTunerState::turnOnAllMotors() {
    for(int i = 1; i<Robot::JointData::NUMBER_OF_JOINTS;i++){
        if(motorTorqueIsOff(i)) {
            turnOnMotor(i);
        }
    }
}

void ScriptTunerState::turnOffAllMotors() {
    for(int i = 1; i<Robot::JointData::NUMBER_OF_JOINTS;i++){
        turnOffMotor(i);
    }        
}