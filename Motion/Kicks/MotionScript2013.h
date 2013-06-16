#ifndef MOTIONSCRIPT2013_H
#define MOTIONSCRIPT2013_H

class MotionScript2013
{
public:
    MotionScript2013();
    ~MotionScript2013();

    // Loads a 
    static MotionScript2013* LoadFromConfigSystem(
        const std::string& path,
        const std::string& name);

    static bool SaveToConfigSystem(
        const MotionScript& script,
        const std::string& path,
        const std::string& name);

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

private:
    float kick_enable_time_;
    float script_start_time_;
    float script_end_time_;

    int current_frame_index_;

    std::vector<MotionScriptFrame*> script_frames_;
};

class MotionScriptFrame
{
public:
    //! Schedules all joint poitions in this frame using the given actionators.
    void ApplyToRobot(float script_start_time, NUActionatorsData* actionators_data);

    //! Returns the NUData id_t corresponding to the servo motor with the given
    //! id.
    static NUData::id_t MapServoIdToNUDataId(int sensor_id);
    
private:
    std::unordered_map<int, ScriptJointDescriptor> joints_;

    //! Time since the start of the motion script
    double time_;
};

class ScriptJointDescriptor
{
public:
    int GetServoId() { return servo_id_; }
    float GetPosition() { return position_; }
    float GetGain() { return gain_; }

private:
    int servo_id_;
    float position_;
    float gain_;
    bool disable_;
};

#endif