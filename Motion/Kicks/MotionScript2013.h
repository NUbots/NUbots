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

    //! Schedules all joint poitions for the current frame using
    //! the given actionators.
    void ApplyCurrentFrameToRobot(m_actions);

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

    
private:
    std::unordered_map<int, ScriptJointDescriptor> joints_;

    //! Time since the start of the motion script
    double time_;
};

class ScriptJointDescriptor
{

private:
    int servo_id_;
    float position_;
    float gain_;
    bool disable_;
};

#endif