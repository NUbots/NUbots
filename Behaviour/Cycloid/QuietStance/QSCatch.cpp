/*! @file QSCatch.cpp
    @brief Implementation of behaviour state class

    @author Jason Kulk
 
 Copyright (c) 2010 Jason Kulk
 
    This file is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This file is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with NUbot.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "QSCatch.h"
#include "QSRelax.h"
#include "QSBallisticController.h"

#include "Infrastructure/NUBlackboard.h"
#include "Infrastructure/NUSensorsData/NUSensorsData.h"
#include "Infrastructure/NUActionatorsData/NUActionatorsData.h"
#include "NUPlatform/NUPlatform.h"

#include "Motion/Tools/MotionCurves.h"

#include "debug.h"
#include "debugverbositybehaviour.h"

#include <boost/random.hpp>
#include <cmath>

QSCatch::QSCatch(const NUData::id_t& joint, const QSBallisticController* parent)
{
    #if DEBUG_BEHAVIOUR_VERBOSITY > 4
        debug << "QSCatch::QSCatch" << endl;
    #endif
    m_joint = joint;
    m_parent = parent;
    
    m_catch_issued = false;
    m_finish_time = 0;
    
    m_strength = 1.0;
    m_catch_duration = 280;
    m_tonic_duration = 150;
    m_catch_duration_variance = 50;
}

QSCatch::~QSCatch()
{
}

void QSCatch::doState()
{
    #if DEBUG_BEHAVIOUR_VERBOSITY > 0
        debug << "QSCatch::doState" << endl;
    #endif
    if (not m_catch_issued)
    {
        float currenttarget;                        // we use the currenttarget as the start point of the motion curve for maximal smoothness
        m_data->getTarget(m_joint, currenttarget);
        float target = -0.02;
        target += m_strength*(target - currenttarget);
        
        float total_duration = normalDistribution(m_catch_duration, m_catch_duration_variance);
        float tone_duration = normalDistribution(m_tonic_duration, (m_tonic_duration/m_catch_duration)*m_catch_duration_variance);
        
        vector<double> times;
        vector<float> positions, velocities;
        MotionCurves::calculate(m_actions->CurrentTime, m_actions->CurrentTime + (total_duration - tone_duration), currenttarget, target, 1, 13, times, positions, velocities);
        m_actions->add(m_joint, times, positions, 100);
        m_catch_issued = true;
        m_finish_time = m_actions->CurrentTime + total_duration;
    }
}

BehaviourState* QSCatch::nextState()
{
    #if DEBUG_BEHAVIOUR_VERBOSITY > 0
        debug << "QSCatch::nextState" << endl;
    #endif
    if (m_catch_issued and m_actions->CurrentTime > m_finish_time)
    {
        m_catch_issued = false;
        return m_parent->getRelax();
    }
    else
        return this;
}

/*! @brief Returns a normal random variable from the normal distribution with mean and sigma
 * 	@param mean the mean of the normal distribution
 * 	@param sigma the standard deviation of the normal distribution
 * 	@return a float from the normal distribution
 */
float QSCatch::normalDistribution(float mean, float sigma)
{
    static unsigned int seed = 1e6*Platform->getRealTime()*Platform->getRealTime()*Platform->getRealTime();          // I am hoping that at least one of the three calls is different for each process
    static boost::mt19937 generator(seed);                       // you need to seed it here with an unsigned int!
    static boost::normal_distribution<float> distribution(0,1);
    static boost::variate_generator<boost::mt19937, boost::normal_distribution<float> > standardnorm(generator, distribution);
    
    float z = standardnorm();       // take a random variable from the standard normal distribution
    float x = mean + z*sigma;       // then scale it to belong to the specified normal distribution
    
    return x;
}
