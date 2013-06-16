#ifndef MOTIONSCRIPT2013_H
#define MOTIONSCRIPT2013_H

#include <unordered_map>
#include "Infrastructure/NUData.h"
#include "Infrastructure/NUActionatorsData/NUActionatorsData.h"

class ScriptJointDescriptor
{
public:
    int SetServoId(int servo_id) { servo_id_ = servo_id; }
    float SetPosition(float position) { position_ = position; }
    float SetGain(float gain) { gain_ = gain; }
    int GetServoId() { return servo_id_; }
    float GetPosition() { return position_; }
    float GetGain() { return gain_; }

private:
    int servo_id_;
    float position_;
    float gain_;
    bool disable_;
};

class MotionScriptFrame
{
public:
    //! Schedules all joint poitions in this frame using the given actionators.
    void ApplyToRobot(float script_start_time, NUActionatorsData* actionators_data);

    //! Returns the NUData id_t corresponding to the servo motor with the given
    //! id.
    static NUData::id_t MapServoIdToNUDataId(int sensor_id);

    float GetTime() { return time_; }
    float SetTime(float new_time) { time_ = new_time; }

    //! Adds a new descriptor for the servo with the given id, or replaces
    //! the current one.
    void AddDescriptor(int servo_id, ScriptJointDescriptor descriptor);

    //! Deletes the descriptor for the given servo id from this script frame.
    //! Servos without descriptors will not have their state changed by this
    //! script frame when ApplyToRobot is called.
    void DeleteDescriptor(int servo_id);

private:
    std::unordered_map<int, ScriptJointDescriptor> joints_;

    //! Time since the start of the motion script
    float time_;
};

class MotionScript2013
{
public:
    MotionScript2013();
    ~MotionScript2013();


    static MotionScript2013* LoadFromConfigSystem(const std::string& path);

    static bool SaveToConfigSystem(
        const MotionScript2013& script,
        const std::string& path);

    //! Advances this script to the next frame.
    //! (does not apply the next frame)
    void AdvanceToNextFrame();

    //! Seeks to the given frame, if it exists.
    //! (does not apply the frame to the robot)
    void SeekFrame(int frame);

    //! Schedules all joint poitions for the current frame using
    //! the given actionators.
    void ApplyCurrentFrameToRobot(NUActionatorsData* actionators_data);

    //! Return the script to the first frame, and prepare it to be run again.
    //! (this method is cheap and idempotent)
    void Reset();

    void StartScript(NUActionatorsData* actionators_data);

    //! Returns true if the script has finished playing.
    bool HasCompleted(float current_time);

    //! Returns the actual time at which the next frame should begin
    float GetNextFrameTime(float current_time);

    int GetFrameCount();

    int GetCurrentFrameIndex();

    MotionScriptFrame* GetCurrentFrame();

    //! Insert the given frame before the frame at the specified index
    void InsertFrame(int index, MotionScriptFrame* frame);

    //! Rmoves the frame at the given index from the script
    void RemoveFrame(int index);

private:
    float kick_enable_time_;
    float script_start_time_;
    float script_end_time_;

    int current_frame_index_;

    std::vector<MotionScriptFrame*> script_frames_;
};

#endif