#include "ScriptKick2013.h"
#include "MotionScript2013.h"
#include "debug.h"
#include "Infrastructure/NUSensorsData/NUSensorsData.h"
#include "Motion/NUWalk.h"
#include "NUPlatform/NUPlatform.h"
#include "Infrastructure/NUBlackboard.h"
#include "Infrastructure/FieldObjects/FieldObjects.h"

//for configs
#include "nubotdataconfig.h"
#include <boost/algorithm/string.hpp>


ScriptKick2013::ScriptKick2013(NUWalk* walk, NUSensorsData* data, NUActionatorsData* actions) : NUKick(walk, data, actions)
{
    // Load scripts for all kick types:
    left_kick_script_       = MotionScript2013::LoadFromConfigSystem("motion.scripts.KickLeft");
    right_kick_script_      = MotionScript2013::LoadFromConfigSystem("motion.scripts.KickRight");
    side_left_kick_script_  = MotionScript2013::LoadFromConfigSystem("motion.scripts.SideKickLeft");
    side_right_kick_script_ = MotionScript2013::LoadFromConfigSystem("motion.scripts.SideKickRight");

    // Load the kickboxes
    loadKickboxes();

    // Set to null while not kicking
    current_script_ = nullptr;
}

ScriptKick2013::~ScriptKick2013()
{
    // Delete all scripts:
    delete left_kick_script_;
    delete right_kick_script_;
    delete side_left_kick_script_;
    delete side_right_kick_script_;
}

void ScriptKick2013::kickToPoint(const std::vector<float>& position, const std::vector<float>& target)
{
    // Ignore calls to this method while a kick is in progress
    if(isActive())
        return;

    // Robot world coordinates:
    float robot_x = Blackboard->Objects->self.wmX();
    float robot_y = Blackboard->Objects->self.wmY();
    float robot_heading = Blackboard->Objects->self.Heading();

    // // Ball world coordinates:
    // float ball_x = ball_r_x + robot_x;
    // float ball_y = ball_r_y + robot_y;

    // Target world coordinates: 
    float target_w_x = target[0];
    float target_w_y = target[1];

    // Note: use robot relative space (i.e. Robot = (0,0), x = forward, y = right )

    // Ball coords relative to robot:
    float b_x = position[0];
    float b_y = position[1];

    // Target relative coordinates:
    float t_x = target_w_x - robot_x;
    float t_y = target_w_y - robot_y;

    // World relative target heading
    float target_heading = atan2f(t_y, t_x);

    // Robot relative target heading:
    float t_theta = mathGeneral::normaliseAngle(target_heading - robot_heading);
    // float kickbox_ball_x = std::cos(-robot_heading) * ball_r_x - std::sin(-robot_heading) * ball_r_y;
    // float kickbox_ball_y = std::cos(-robot_heading) * ball_r_y + std::sin(-robot_heading) * ball_r_x;

    // std::cout << __PRETTY_FUNCTION__ << ": b_x = " << b_x << std::endl;
    // std::cout << __PRETTY_FUNCTION__ << ": b_y = " << b_y << std::endl;
    // std::cout << __PRETTY_FUNCTION__ << ": t_x = " << t_x << std::endl;
    // std::cout << __PRETTY_FUNCTION__ << ": t_y = " << t_y << std::endl;
    // std::cout << __PRETTY_FUNCTION__ << ": t_theta = " << t_theta << std::endl;

    // std::cout << __PRETTY_FUNCTION__ << ": side_left = " << side_left_kick_area_.PointInside(b_x, b_y) << std::endl;
    // std::cout << __PRETTY_FUNCTION__ << ": left_kick = " << left_kick_area_.PointInside(b_x, b_y) << std::endl;
    // std::cout << __PRETTY_FUNCTION__ << ": side_right = " << side_right_kick_area_.PointInside(b_x, b_y) << std::endl;
    // std::cout << __PRETTY_FUNCTION__ << ": right_kick = " << right_kick_area_.PointInside(b_x, b_y) << std::endl;



    // std::cout << __PRETTY_FUNCTION__ << "robot_x = " << robot_x << std::endl;
    // std::cout << __PRETTY_FUNCTION__ << "robot_y = " << robot_y << std::endl;
    // std::cout << __PRETTY_FUNCTION__ << "robot_heading = " << robot_heading << std::endl;
    // std::cout << __PRETTY_FUNCTION__ << "ball_x = " << ball_x << std::endl;
    // std::cout << __PRETTY_FUNCTION__ << "ball_y = " << ball_y << std::endl;
    // std::cout << __PRETTY_FUNCTION__ << "ball_r_x = " << ball_r_x  << std::endl;
    // std::cout << __PRETTY_FUNCTION__ << "ball_r_y = " << ball_r_y  << std::endl;
    // std::cout << __PRETTY_FUNCTION__ << "target_x = " << target_x << std::endl;
    // std::cout << __PRETTY_FUNCTION__ << "target_y = " << target_y << std::endl;
    // std::cout << __PRETTY_FUNCTION__ << "kickbox_ball_x = " << kickbox_ball_x << std::endl;
    // std::cout << __PRETTY_FUNCTION__ << "kickbox_ball_y = " << kickbox_ball_y << std::endl;
    // std::cout << __PRETTY_FUNCTION__ << "kicktarget_heading = " << kicktarget_heading << std::endl;

    // The tolerance for whether the robot is considered to be facing the target.
    float target_angle_tolerance = mathGeneral::PI / 4.0f;

    double pi_3_4 = mathGeneral::PI * 3.0 / 4.0;

    // Don't kick if robot is not facing the target:
    if(-target_angle_tolerance < t_theta && t_theta < target_angle_tolerance)
    {
        if(left_kick_script_ != nullptr &&
           left_kick_area_.PointInside(b_x, b_y)) {
            StartKick(left_kick_script_, leftLeg);
        } else if(right_kick_script_ != nullptr &&
                  right_kick_area_.PointInside(b_x, b_y)) {
            StartKick(right_kick_script_, rightLeg);
        }
    } else if(-pi_3_4 < t_theta && t_theta < pi_3_4) {
        if((side_left_kick_script_ != nullptr &&
            side_left_kick_area_.PointInside(b_x, b_y))) {
            StartKick(side_left_kick_script_, rightLeg);
        } else if(side_right_kick_script_ != nullptr &&
                  side_right_kick_area_.PointInside(b_x, b_y)) {
            StartKick(side_right_kick_script_, leftLeg);
        }
    }
}

void ScriptKick2013::StartKick(
    MotionScript2013* kick_script, 
    KickingLeg kicking_leg)
{
    m_kicking_leg = kicking_leg;
    current_script_ = kick_script;

    m_walk->stop();
    setArmEnabled(true, true);
    setHeadEnabled(true);
    m_kick_ready = true;
    m_kick_enabled = true;

    // Begin the kick script
    current_script_->StartScript();
    current_script_->ApplyCurrentFrameToRobot(m_actions);
}

void ScriptKick2013::doKick()
{
    m_kick_ready = false;

    // If doKick is called while the robot is not kicking, just return.
    if(!isActive())
        return;

    float current_time = Platform->getTime();

    // If the current script is complete
    if(current_script_->HasCompleted(current_time))
    {
        kill();
        return;
    }

    // If it's time for the next frame
    if(current_time >= current_script_->GetNextFrameTime())
    {
        // Schedule the next joint positions
        current_script_->AdvanceToNextFrame();
        current_script_->ApplyCurrentFrameToRobot(m_actions);
    }
}

void ScriptKick2013::stop() 
{ 
    #warning This could cause issues. The original stop() method only 'stops' the kick if it's alreafy finished...'
    kill(); 
}
void ScriptKick2013::kill() 
{ 
    current_script_ = nullptr;
    m_kick_ready = true;
    m_kick_enabled = false;
    setArmEnabled(false, false);
    setHeadEnabled(false);
    m_kicking_leg = noLeg;
}

bool ScriptKick2013::isActive()
{
    return current_script_ != nullptr && current_script_->IsActive();
}

bool ScriptKick2013::isUsingHead() { return current_script_ != nullptr && current_script_->IsUsingHead(); }
bool ScriptKick2013::isUsingArms() { return current_script_ != nullptr && current_script_->IsUsingLegs(); }
bool ScriptKick2013::isUsingLegs() { return current_script_ != nullptr && current_script_->IsUsingArms(); }
bool ScriptKick2013::requiresHead() { return current_script_ != nullptr && current_script_->RequiresHead(); }
bool ScriptKick2013::requiresArms() { return true; }
bool ScriptKick2013::requiresLegs() { return true; }

float ScriptKick2013::GetCurrentScriptTime()
{
    if(current_script_ == nullptr)
    {
        std::cout << __PRETTY_FUNCTION__ << ": WARNING: current_script_ == nullptr;" << std::cout;
        return 0;
    } 

    return Platform->getTime() - current_script_->GetStartTime();
}

void ScriptKick2013::loadKickboxes()
{
    //defaults to guarantee at least old performance in case of bad file
    float x_min_right_forward = 5.0f;
    float x_max_right_forward = 11.0f;
    float y_min_right_forward = -3.2f;
    float y_max_right_forward = -9.5f;

    float x_min_left_forward = 5.0f;
    float x_max_left_forward = 11.0f;
    float y_min_left_forward = 3.2f;
    float y_max_left_forward = 9.5f;

    float x_min_right_side = 5.0f;
    float x_max_right_side = 11.0f;
    float y_min_right_side = -3.2f;
    float y_max_right_side = -6.5f;

    float x_min_left_side= 5.0f;
    float x_max_left_side = 11.0f;
    float y_min_left_side = 3.2f;
    float y_max_left_side = 6.5f;

    std::string filename = CONFIG_DIR + std::string("kickboxes.cfg");
    std::ifstream input(filename.c_str());
    std::string id_str;
    if(input.good()) {
        float xmin, xmax, ymin, ymax;
        while(input.good()) {
            // read in the rule name
            getline(input, id_str, ':');
            boost::trim(id_str);
            boost::to_lower(id_str);

            input.ignore(30, '(');
            input >> xmin;
            input.ignore(10, ',');
            input >> xmax;
            input.ignore(10, ',');
            input >> ymin;
            input.ignore(10, ',');
            input >> ymax;
            input.ignore(10, ')');

            // ignore the rest of the line
            input.ignore(128, '\n');
            input.peek();               //trigger eofbit being set in the case of this being the last rule

            if(id_str.compare("rightforward") == 0) {
                x_min_right_forward = xmin;
                x_max_right_forward = xmax;
                y_min_right_forward = ymin;
                y_max_right_forward = ymax;
            }
            else if(id_str.compare("leftforward") == 0) {
                x_min_left_forward = xmin;
                x_max_left_forward = xmax;
                y_min_left_forward = ymin;
                y_max_left_forward = ymax;
            }
            else if(id_str.compare("rightside") == 0) {
                x_min_right_side = xmin;
                x_max_right_side = xmax;
                y_min_right_side = ymin;
                y_max_right_side = ymax;
            }
            else if(id_str.compare("leftside") == 0) {
                x_min_left_side= xmin;
                x_max_left_side = xmax;
                y_min_left_side = ymin;
                y_max_left_side = ymax;
            }
            else
                errorlog << "ScriptKick::loadKickParameters - invalid kick name: " << id_str << std::endl;
        }
    }
    else {
        errorlog << "ScriptKick::loadKickParameters - failed to load kickboxes.cfg" << std::endl;
    }

    

    right_kick_area_ = Rectangle(x_min_right_forward, x_max_right_forward, y_min_right_forward, y_max_right_forward);
    left_kick_area_  = Rectangle( x_min_left_forward,  x_max_left_forward,  y_min_left_forward,  y_max_left_forward); //HACK: move right kick box three cm to right
    side_right_kick_area_ = Rectangle(x_min_right_side, x_max_right_side, y_min_right_side, y_max_right_side); //HACK: kick box less wide for side kicks
    side_left_kick_area_  = Rectangle( x_min_left_side,  x_max_left_side,  y_min_left_side,  y_max_left_side);
    return;
}
