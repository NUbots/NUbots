#ifndef MOTIONSCRIPT2013_H
#define MOTIONSCRIPT2013_H

#include <unordered_map>
#include <iostream>
#include "Infrastructure/NUData.h"
#include "Infrastructure/NUActionatorsData/NUActionatorsData.h"

class NUSensorsData;

class ScriptJointDescriptor
{
public:
    ScriptJointDescriptor() : servo_id_(0), position_(0), gain_(0), disable_(0) {}

    void SetServoId(int servo_id) { servo_id_ = servo_id; }
    void SetPosition(float position) { position_ = position; }
    void SetGain(float gain) { gain_ = gain; }
    void SetDisable( bool disable) { disable_= disable; }
    int GetServoId() { return servo_id_; }
    float GetPosition() { return position_; }
    float GetGain() { return gain_; }
    bool GetDisable() { return disable_; }

private:
    int servo_id_;
    float position_;
    float gain_;
    bool disable_;
};

class MotionScriptFrame
{
public:
    MotionScriptFrame() : duration_(0) {}

    bool SaveToConfigSystem(const std::string& frame_path);

    static MotionScriptFrame* LoadFromConfigSystem(const std::string& path,
                                                  int frame_number);

    //! Schedules all joint poitions in this frame using the given actionators.
    void ApplyToRobot(NUActionatorsData* actionators_data);

    //! Causes this frame to be applied to the robot at the given time.
    void ApplyToRobotAtTime(double start_time,
                            NUActionatorsData* actionators_data);

    //! Returns the NUData id_t corresponding to the servo motor with the given
    //! id.
    static NUData::id_t MapServoIdToNUDataId(int sensor_id);

    //! Adds a new descriptor for the servo with the given id, or replaces
    //! the current one.
    void AddDescriptor(ScriptJointDescriptor descriptor);

    //! Deletes the descriptor for the given servo id from this script frame.
    //! Servos without descriptors will not have their state changed by this
    //! script frame when ApplyToRobot is called.
    void DeleteDescriptor(int servo_id);

    //! Returns a copy of the descriptor corresponding to the given id in
    //! descriptor.
    //! Returns whether the operation succeeded.
    bool GetDescriptor(int servo_id, ScriptJointDescriptor* descriptor);

    //! Returns the duration of this frame
    float GetDuration() { return duration_; }
    //! Sets this frame's duration
    void SetDuration(float duration)
    {
        if(duration < 0)
        {
            std::cout << __PRETTY_FUNCTION__ << " - "
                      << "duration must be non-negative (not "
                      << duration << ");";
            return;
        }

        duration_ = duration;
    }

private:
    static bool LoadJointFromConfigSystem(
        const std::string& frame_path,
        int servo_id,
        ScriptJointDescriptor* descriptor);


    std::unordered_map<int, ScriptJointDescriptor> joints_;

    //! This frame's duration, in milliseconds
    float duration_;
};

class MotionScript2013
{
public:
    MotionScript2013() : script_start_time_(0),
                         current_frame_index_(0) { }
    ~MotionScript2013();

    bool SaveToConfigSystem(const std::string& path);

    static MotionScript2013* LoadFromConfigSystem(const std::string& path);

    static MotionScript2013* LoadOldScript(const std::string& path);

    static MotionScript2013* InitialiseScriptFromOld(std::vector<std::vector<double> > times, std::vector<std::vector<float> > positions, std::vector<std::vector<float> > gains);


    //! Advances this script to the next frame.
    //! (does not apply the next frame)
    void AdvanceToNextFrame();

    //!Plays entire script from the current frame.
    void PlayScript(NUActionatorsData* actionators_data);

    //! Seeks to the given frame, if it exists.
    //! (does not apply the frame to the robot)
    void SeekFrame(int frame);

    //! Schedules all joint poitions for the current frame using
    //! the given actionators.
    void ApplyCurrentFrameToRobot(NUActionatorsData* actionators_data);

    //! Return the script to the first frame, and prepare it to be run again.
    //! (this method is cheap and idempotent)
    void Reset();

    void StartScript();

    //! Returns true if the script has finished playing.
    bool HasCompleted(float current_time);

    //! Returns the actual time at which the next frame should begin
    float GetNextFrameTime();

    //! Returns the duration of the entire script
    //! (i.e. the sum of the duration of its frames)
    float GetScriptDuration();

    int GetFrameCount();

    int GetCurrentFrameIndex();

    MotionScriptFrame* GetCurrentFrame();

    //! Adds the given frame to the end of the script
    void AddFrame(MotionScriptFrame* frame);

    //! Insert the given frame before the frame at the specified index
    void InsertFrame(int index, MotionScriptFrame* frame);

    //! Rmoves the frame at the given index from the script
    void RemoveFrame(int index);

    //! Creates a copy of the frame at the given index, and inserts it before
    //! the given index.
    void DuplicateFrame(int index);


    void ScheduleEntireScript(NUSensorsData* sensors_data,
                              NUActionatorsData* actionators_data);

    float GetStartTime(){return script_start_time_;}

    bool IsActive();
    bool IsUsingHead();
    bool IsUsingArms(){return IsActive();}
    bool IsUsingLegs(){return IsActive();}
    bool IsReady();
    
    bool RequiresHead();
    bool RequiresArms(){return true;}
    bool RequiresLegs(){return true;}

    float TimeFinished();
    float TimeFinishedWithHead();
    float TimeFinishedWithLArm();
    float TimeFinishedWithRArm();

private:
    float script_start_time_;

    int current_frame_index_;

    std::vector<MotionScriptFrame*> script_frames_;
};

#endif
