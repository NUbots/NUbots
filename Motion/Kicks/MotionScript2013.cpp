#include <string>
#include <stdexcept>
#include <unordered_map>
#include <iostream>
#include "MotionScript2013.h"
#include "Infrastructure/NUData.h"
#include "Infrastructure/NUActionatorsData/NUActionatorsData.h"
#include "Framework/darwin/Framework/include/JointData.h"
#include "Motion/Tools/MotionFileTools.h"
#include "debug.h"
#include "Infrastructure/NUBlackboard.h"
#include "ConfigSystem/ConfigManager.h"
#include "NUPlatform/NUPlatform.h"

NUData::id_t MotionScriptFrame::MapServoIdToNUDataId(int sensor_id)
{
    switch(sensor_id)
    {
        case Robot::JointData::ID_R_SHOULDER_PITCH: return NUData::RShoulderPitch;
        case Robot::JointData::ID_L_SHOULDER_PITCH: return NUData::LShoulderPitch;
        case Robot::JointData::ID_R_SHOULDER_ROLL:  return NUData::RShoulderRoll;
        case Robot::JointData::ID_L_SHOULDER_ROLL:  return NUData::LShoulderRoll;
        case Robot::JointData::ID_R_ELBOW:          return NUData::RElbowPitch;
        case Robot::JointData::ID_L_ELBOW:          return NUData::LElbowPitch;
        case Robot::JointData::ID_R_HIP_YAW:        return NUData::RHipYaw;
        case Robot::JointData::ID_L_HIP_YAW:        return NUData::LHipYaw;
        case Robot::JointData::ID_R_HIP_ROLL:       return NUData::RHipRoll;
        case Robot::JointData::ID_L_HIP_ROLL:       return NUData::LHipRoll;
        case Robot::JointData::ID_R_HIP_PITCH:      return NUData::RHipPitch;
        case Robot::JointData::ID_L_HIP_PITCH:      return NUData::LHipPitch;
        case Robot::JointData::ID_R_KNEE:           return NUData::RKneePitch;
        case Robot::JointData::ID_L_KNEE:           return NUData::LKneePitch;
        case Robot::JointData::ID_R_ANKLE_PITCH:    return NUData::RAnklePitch;
        case Robot::JointData::ID_L_ANKLE_PITCH:    return NUData::LAnklePitch;
        case Robot::JointData::ID_R_ANKLE_ROLL:     return NUData::RAnkleRoll;
        case Robot::JointData::ID_L_ANKLE_ROLL:     return NUData::LAnkleRoll;
        case Robot::JointData::ID_HEAD_PAN:         return NUData::HeadYaw;
        case Robot::JointData::ID_HEAD_TILT:        return NUData::HeadPitch;
        default: {
            std::cout << __PRETTY_FUNCTION__ 
                      << " - Invalid sensor_id: " << sensor_id << ";";
            return NUData::id_t();
        }
    }
}

int MapRowIndexToServoId(int index)
{
    switch(index)
    {
         case( 0): return Robot::JointData::ID_HEAD_TILT;        
         case( 1): return Robot::JointData::ID_HEAD_PAN;         
         case( 2): return Robot::JointData::ID_L_SHOULDER_ROLL;  
         case( 3): return Robot::JointData::ID_L_SHOULDER_PITCH; 
         case( 4): return Robot::JointData::ID_L_ELBOW;          
         case( 5): return Robot::JointData::ID_R_SHOULDER_ROLL;  
         case( 6): return Robot::JointData::ID_R_SHOULDER_PITCH; 
         case( 7): return Robot::JointData::ID_R_ELBOW;          
         case( 8): return Robot::JointData::ID_L_HIP_ROLL;       
         case( 9): return Robot::JointData::ID_L_HIP_PITCH;      
         case(10): return Robot::JointData::ID_L_HIP_YAW;        
         case(11): return Robot::JointData::ID_L_KNEE;           
         case(12): return Robot::JointData::ID_L_ANKLE_ROLL;     
         case(13): return Robot::JointData::ID_L_ANKLE_PITCH;    
         case(14): return Robot::JointData::ID_R_HIP_ROLL;       
         case(15): return Robot::JointData::ID_R_HIP_PITCH;      
         case(16): return Robot::JointData::ID_R_HIP_YAW;        
         case(17): return Robot::JointData::ID_R_KNEE;           
         case(18): return Robot::JointData::ID_R_ANKLE_ROLL;     
         case(19): return Robot::JointData::ID_R_ANKLE_PITCH;    
        default: {
            std::cout   << __PRETTY_FUNCTION__
                        << ": Invalid index: " << index << ";"
                        << std::endl;
            return -1;
        }
    }
}

void MotionScriptFrame::ApplyToRobot(NUActionatorsData* actionators_data)
{
    for(auto key_value : joints_)
    {
        auto& joint = key_value.second;

        if(!joint.GetDisable())
        {
            actionators_data->add(
                MapServoIdToNUDataId(joint.GetServoId()),
                duration_,
                joint.GetPosition(),
                joint.GetGain());            
        } else {
            actionators_data->add(
                MapServoIdToNUDataId(joint.GetServoId()),
                duration_,
                joint.GetPosition(),
                0);
        }
    }
}

void MotionScriptFrame::AddDescriptor(int servo_id, ScriptJointDescriptor descriptor)
{
    joints_[servo_id] = descriptor;
}

void MotionScriptFrame::DeleteDescriptor(int servo_id)
{
    joints_.erase(servo_id);
}

bool MotionScriptFrame::GetDescriptor(int servo_id, ScriptJointDescriptor* descriptor)
{
    if(descriptor == nullptr)
        return nullptr;

    try
    {
        *descriptor =  joints_.at(servo_id);
        return true;
    }
    catch(std::out_of_range)
    {
        return false;
    }
}


MotionScript2013::~MotionScript2013()
{
    for(MotionScriptFrame* frame : script_frames_)
        delete frame;
}

bool MotionScript2013::SaveToConfigSystem(const std::string& path)
{
    ConfigSystem::ConfigManager* config = Blackboard->Config;

    bool success;

    long script_format_version = 0;
    config->CreateParam(path, "script_format_version", script_format_version);
    config->SetValue(path, "script_format_version", script_format_version);

    long num_frames = GetFrameCount();
    config->CreateParam(path, "num_frames", num_frames);
    config->SetValue(path, "num_frames", num_frames);

    for(int i = 0; i < script_frames_.size(); i++)
    {
        std::cout << "save frame " << i << std::endl;

        auto *frame = script_frames_[i];

        std::stringstream frame_ss;
        frame_ss << path << ".frame_" << i;
        const std::string frame_path = frame_ss.str();
        std::cout << "    saving to :" << frame_path << std::endl;
        frame->SaveToConfigSystem(frame_path);
    }

    return config->SaveConfiguration("defaultConfig");
}

bool MotionScriptFrame::SaveToConfigSystem(const std::string& frame_path)
{
    ConfigSystem::ConfigManager* config = Blackboard->Config;

    double duration = GetDuration();
    config->CreateParam(frame_path, "duration", duration);
    config->SetValue(frame_path, "duration", duration);

    for(auto& key_value : joints_)
    {
        auto &joint = key_value.second;

        std::stringstream joint_ss;
        joint_ss << frame_path << ".joint_" << joint.GetServoId();
        const std::string joint_path = joint_ss.str();

        double position = joint.GetPosition();
        config->CreateParam(joint_path, "position", position);
        config->SetValue(joint_path, "position", position);

        double gain = joint.GetGain();
        config->CreateParam(joint_path, "gain", gain);
        config->SetValue(joint_path, "gain", gain);
    }
}

MotionScript2013* MotionScript2013::LoadFromConfigSystem(
    const std::string& path)
{
    /* Format:
    'path.to.script': {
        'script_format_version': 0,
        'num_frames': 2,
        'frame_0': {
            'duration_': '1.5',
            'servo_id_3': {
                'position_': 1.0,
                'gain_': 1.0
            }
        }
    }
    */

    ConfigSystem::ConfigManager* config = Blackboard->Config;

    bool success;

    long script_format_version;
    success = config->ReadValue(path, "script_format_version", &script_format_version);

    long num_frames;
    success = config->ReadValue(path, "num_frames", &num_frames);

    if(!success)
        return nullptr;

    auto* script = new MotionScript2013();

    for(int i = 0; i < num_frames; i++)
    {
        auto* frame = MotionScriptFrame::LoadFromConfigSystem(path, i);
        script->AddFrame(frame);
    }
    
    return script;
}

MotionScriptFrame* MotionScriptFrame::LoadFromConfigSystem(
    const std::string& path,
    int frame_number)
{
    ConfigSystem::ConfigManager* config = Blackboard->Config;
    
    std::stringstream frame_path_ss;
    frame_path_ss << path << ".frame_" << frame_number;

    double duration;
    bool success = config->ReadValue(frame_path_ss.str(), "duration", &duration);

    if(!success)
        return nullptr;
    
    auto* frame = new MotionScriptFrame();
    frame->SetDuration(duration);

    for(int j = 0; j < Robot::JointData::NUMBER_OF_JOINTS; j++)
    {
        ScriptJointDescriptor descriptor;
        success = MotionScriptFrame::LoadJointFromConfigSystem(
            frame_path_ss.str(), j, &descriptor);

        if(!success)
            continue;

        frame->AddDescriptor(j, descriptor);
    }

     return frame;
}

bool MotionScriptFrame::LoadJointFromConfigSystem(
        const std::string& frame_path,
        int servo_id,
        ScriptJointDescriptor* descriptor)
{
    ConfigSystem::ConfigManager* config = Blackboard->Config;
    
    std::stringstream joint_path_ss;
    joint_path_ss << frame_path << ".joint_" << servo_id;

    double position;
    bool success = config->ReadValue(joint_path_ss.str(), "position", &position);

    double gain;
    success = config->ReadValue(joint_path_ss.str(), "gain", &gain);

    if(!success)
        return false;

    descriptor->SetServoId(servo_id);
    descriptor->SetPosition(position);
    descriptor->SetGain(gain);

     return true;
}

MotionScript2013* MotionScript2013::LoadOldScript(const std::string& path){
    //Method modified from Motion/Tools/MotionScript.h
    std::cout<<"Loading script from old script file in location " << path << std::endl;
    std::ifstream file(path);
    if (!file.is_open())
    {
        errorlog << "MotionScript2013::LoadOldScript(). Unable to open. " << path << std::endl;
        return nullptr;
    }
    
    MotionFileTools::toFloat(file);
    MotionFileTools::toBool(file);
    auto labels = MotionFileTools::toStringVector(file);
    if (labels.empty())
    {
        errorlog << "MotionScript::load(). Unable to load " << path << " the file labels are invalid " << std::endl;
        return nullptr;
    }
    float playspeed = 1.0;
    
    int numjoints = labels.size() - 1;
    std::vector<vector<double> > times = vector<vector<double> >(numjoints, vector<double>());
    std::vector<vector<float> > positions = vector<vector<float> >(numjoints, vector<float>());
    std::vector<vector<float> > gains = vector<vector<float> >(numjoints, vector<float>());
    
    float time;
    vector<vector<float> > row;
    
    while (!file.eof())
    {
        MotionFileTools::toFloatWithMatrix(file, time, row);
        if (row.size() >= numjoints)
        {   // discard rows that don't have enough joints
            for (int i=0; i<numjoints; i++)
            {
                if (row[i].size() > 0)
                {   // if there is an entry then the first must be a position
                    times[i].push_back(1000*time);
                    positions[i].push_back(row[i][0]);
                    
                    // because the way the joint actionators are designed we must also specify a gain
                    if (row[i].size() > 1)
                    {   // if there is a second entry then it is the gain
                        gains[i].push_back(row[i][1]);
                    }
                    else
                    {   // however if there is no second entry we reuse the previous entry or use 100% if this is the first one
                        if (gains[i].empty())
                            gains[i].push_back(100.0);
                        else
                            gains[i].push_back(gains[i].back());
                    }
                    
                }
            }
        }
        row.clear();
    }
    file.close();        

    std::cout<<"Loading old script data."<< std::endl;
    std::cout<<"Times = [ "<< std::endl;
    for(int i = 0; i<times.size();i++){
        if(times[i].empty()) 
                continue;
        for(int j = 0; j<times[i].size();j++){
            std::cout<<" "<<times[i][j];
        }
        std::cout<<std::endl;
    }
    std::cout<<" ]"<<std::endl;

    std::cout<<"Positions = [ "<< std::endl;
    for(int i = 0; i<positions.size();i++){
        if(positions[i].empty()) 
                continue;
        for(int j = 0; j<positions[i].size();j++){
            std::cout<<" "<<positions[i][j];
        }
        std::cout<<std::endl;
    }
    std::cout<<" ]"<<std::endl;

    std::cout<<"Gains = [ "<< std::endl;
    for(int i = 0; i<gains.size();i++){
        if(gains[i].empty()) 
                continue;
        for(int j = 0; j<gains[i].size();j++){
            std::cout<<" "<<gains[i][j];
        }
        std::cout<<std::endl;
    }
    std::cout<<" ]"<<std::endl;

    return InitialiseScriptFromOld(times,positions,gains);
    
}

MotionScript2013* MotionScript2013::InitialiseScriptFromOld(vector<vector<double> > times, vector<vector<float> > positions, vector<vector<float> > gains){
    
    MotionScript2013* script = new MotionScript2013();
    std::vector<MotionScriptFrame*> new_frames;
    size_t num_frames = 0;
    size_t first_non_empty_motor;
    size_t num_motors = times.size();

    if(positions.size() != num_motors || gains.size() != num_motors)
    {
        std::cout << "Error: times, positions and size matrices not equal in number of motors!" << std::endl;
        return script;
    }

    for(int motor_index = 0; motor_index < num_motors; motor_index++)
    {
        if(times[motor_index].size() > 0) {
            first_non_empty_motor = motor_index;
            num_frames = times[motor_index].size();
            if(positions[motor_index].size() != num_frames || gains[motor_index].size() != num_frames)
            {
                std::cout << "Error: first non-empty times, positions and gains vector not equal in number of frames." << std::endl;
                return script;
            }
            break;
        }
    }

	for(int frame_number = 0; frame_number < num_frames; frame_number++)
    {
        new_frames.push_back(new MotionScriptFrame());
        new_frames[frame_number]->SetDuration(frame_number==0 ?
                                              times[first_non_empty_motor][frame_number]:
                                              times[first_non_empty_motor][frame_number] - times[first_non_empty_motor][frame_number-1]);
        
		for(int motor_index = 0; motor_index < num_motors; motor_index++)
        {
            if(positions[motor_index].empty()) 
                continue;
            int servo_id = MapRowIndexToServoId(motor_index);
            float pos = positions[motor_index][frame_number];
            float gain = gains[motor_index][frame_number];

            std::cout<<"Loading script descriptor." << std::endl;
            std::cout<<"Servo Number     " << servo_id << std::endl;
            std::cout<<"Position         " << pos << std::endl;
            std::cout<<"Gain             " << gain << std::endl;
            std::cout<<"Time             " << times[motor_index][frame_number] << std::endl; 
                   
            ScriptJointDescriptor descriptor;
            descriptor.SetServoId(servo_id);
            descriptor.SetPosition(pos);
            descriptor.SetGain(gain);
            new_frames[frame_number]->AddDescriptor(servo_id, descriptor);
        }
    }

    for(int i = 0; i<new_frames.size();i++){
        script->AddFrame(new_frames[i]);
    }

    return script;

}



void MotionScript2013::AdvanceToNextFrame()
{
    current_frame_index_++;
}

void MotionScript2013::SeekFrame(int frame)
{
    if(frame < 0 || frame >= script_frames_.size())
    {
        std::cout << "This script does not contain the frame: " 
                  << frame << ";" << std::endl;
        return;
    }
    current_frame_index_ = frame;
}

void MotionScript2013::ApplyCurrentFrameToRobot(NUActionatorsData* actionators_data)
{
    if(current_frame_index_ >= script_frames_.size()){
        return;
    }

    auto current_frame = script_frames_[current_frame_index_];
    current_frame->ApplyToRobot(actionators_data);
}


void MotionScript2013::Reset()
{
    current_frame_index_ = 0;
}

void MotionScript2013::StartScript()
{
    script_start_time_ = Platform->getTime();
    current_frame_index_ = 0;    
}

void MotionScript2013::PlayScript(NUActionatorsData* actionators_data){
    StartScript();
    double current_time = Platform->getTime();
    while(!HasCompleted(current_time)){ 
        // If it's time for the next frame
        current_time = Platform->getTime();
        double next_frame_time = GetNextFrameTime();
        // std::cout<<"Frame number "<< GetCurrentFrameIndex() << std::endl;
        // std::cout <<"Current time = "<< current_time<< std::endl;
        // std::cout <<"Next Frame time = "<< next_frame_time<< std::endl;

        if(current_time >= next_frame_time)
        {
            // Schedule the next joint positions
            AdvanceToNextFrame();
            ApplyCurrentFrameToRobot(actionators_data);
            std::cout << "Frame number "<< GetCurrentFrameIndex()<<std::endl;
        }
    }
    std::cout <<"Script Finished in "<< (current_time-script_start_time_) << "ms."<< std::endl;
}

bool MotionScript2013::HasCompleted(float current_time)
{

    if(current_frame_index_ >= script_frames_.size()){
        return true;
    } else {
        return false;
    }
        

    
    // if(current_frame_index_ < script_frames_.size())
    //     return false;

    // float script_time = current_time - script_start_time_;
    // auto& current_frame = script_frames_[current_frame_index_];

    // if(script_time < GetScriptDuration())
    //     return false;

    // return true;
}

float MotionScript2013::GetNextFrameTime()
{
    float next_frame_time = script_start_time_;
    for(int i = 0; i<=current_frame_index_;i++){
        next_frame_time += script_frames_[i]->GetDuration();
    }
    return next_frame_time;
}

int MotionScript2013::GetFrameCount()
{
    return script_frames_.size();
}

int MotionScript2013::GetCurrentFrameIndex()
{
    return current_frame_index_;
}

MotionScriptFrame* MotionScript2013::GetCurrentFrame()
{
    return script_frames_[current_frame_index_];
}

void MotionScript2013::InsertFrame(int index, MotionScriptFrame* frame)
{
    auto it = script_frames_.begin();

    script_frames_.insert(it + index, frame);
}

void MotionScript2013::RemoveFrame(int index)
{
    auto it = script_frames_.begin();

    script_frames_.erase(it + index);
}

void MotionScript2013::DuplicateFrame(int index)
{
    auto* current_frame = script_frames_[current_frame_index_];

    auto* new_frame = new MotionScriptFrame(*current_frame);

    InsertFrame(index, current_frame);
}

float MotionScript2013::GetScriptDuration()
{
    float script_duration = 0;

    for(auto* frame : script_frames_)
        script_duration += frame->GetDuration();

    return script_duration;
}

void MotionScript2013::AddFrame(MotionScriptFrame* frame)
{
    script_frames_.push_back(frame);
}
