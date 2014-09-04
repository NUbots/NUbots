#include "Gait.h"
#include <iostream>


Gait::Gait(const Gait & other){
    setGaitFromGait(other);
    //std::cout << "from Gait: "<<std::endl<< toString() <<std::endl;

}
Gait::Gait(const AbstractGait & other){
    setGaitFromGait(other);
    //std::cout << "from Abstract gait: "<<std::endl<< toString() <<std::endl;
}

Gait::Gait(
    const float _stance_config[WP::LEN_STANCE_CONFIG],
    const float _step_config[WP::LEN_STEP_CONFIG],
    const float _zmp_config[WP::LEN_ZMP_CONFIG],
    const float _joint_hack_config[WP::LEN_HACK_CONFIG],
    const float _sensor_config[WP::LEN_SENSOR_CONFIG],
    const float _stiffness_config[WP::LEN_STIFF_CONFIG],
    const float _odo_config[WP::LEN_ODO_CONFIG],
    const float _arm_config[WP::LEN_ARM_CONFIG])
{
    setGaitFromArrays(_stance_config,
                  _step_config,
                  _zmp_config,
                  _joint_hack_config,
                  _sensor_config,
                  _stiffness_config,
                  _odo_config,
                  _arm_config);
    //std::cout << "from arrays: "<<std::endl<< toString() <<std::endl;
}


//Default constructor - use at your own risk
Gait::Gait(){
    std::cout << "Danger Mr. Robinson - are you sure you want to default"
        " construct walk parameters?"<<std::endl;
}
