/*! @file Profiler.cpp
    @brief Implemenation of Profiler class
 
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

#include "Profiler.h"
#include "Infrastructure/NUBlackboard.h"
#include "NUPlatform/NUPlatform.h"

#include "debug.h"

/*! @brief Constructs a profiler class with name
    @param name the name of the profiler (probably the module/thread/function you are profiling)
 */
Profiler::Profiler(std::string name)
{
    m_name = name;
}

Profiler::~Profiler()
{
}

/*! @brief Starts the profiler
 */
void Profiler::start()
{
    m_start_thread_time = Platform->getThreadTime();
    m_start_process_time = Platform->getProcessTime();
    m_start_real_time = Platform->getRealTime();
}

/*! @brief Stops the profiler
 */
void Profiler::stop()
{
    split("total");
}

/*! @brief A split time is added to the profiler. The time will be the difference since the last split
    @param name the name of the split
 */
void Profiler::split(std::string name)
{
    double threadtime = Platform->getThreadTime();
    double processtime = Platform->getProcessTime();
    double realtime = Platform->getRealTime();

    if (m_split_names.empty())
    {   // if it is the first split, then we time is the difference from the start
        m_diff_thread_times.push_back(threadtime - m_start_thread_time);
        m_diff_process_times.push_back(processtime - m_start_process_time);
        m_diff_real_times.push_back(realtime - m_start_real_time);
    }
    else
    {   // otherwise it is the difference since the last split
        m_diff_thread_times.push_back(threadtime - m_split_thread_times.back());
        m_diff_process_times.push_back(processtime - m_split_process_times.back());
        m_diff_real_times.push_back(realtime - m_split_real_times.back());
    }
    
    m_split_names.push_back(name);
    m_split_thread_times.push_back(threadtime);
    m_split_process_times.push_back(processtime);
    m_split_real_times.push_back(realtime);
}

/*! @brief Resets the profiler
 */
void Profiler::reset()
{
    m_split_thread_times.clear();
    m_split_process_times.clear();
    m_split_real_times.clear();
    
    m_diff_thread_times.clear();
    m_diff_process_times.clear();
    m_diff_real_times.clear();
    
    m_split_names.clear();
}

/*! @brief Prints the results of the profiler to the output stream, and also resets the profiler.
    @relates Profiler
 */
ostream& operator<<(ostream& output, Profiler& profiler)
{
    if (not profiler.m_split_names.empty())
    {
        output << profiler.m_name << " " << profiler.m_split_thread_times.back() - profiler.m_start_thread_time << ": ";
        for (size_t i=0; i<profiler.m_split_names.size(); i++)
        {
            output << profiler.m_split_names[i] << ": [t:" << profiler.m_diff_thread_times[i] << " p:" << profiler.m_diff_process_times[i] << "] ";
        }
        output << " other processes: " << (profiler.m_split_real_times.back() - profiler.m_start_real_time) - (profiler.m_split_process_times.back() - profiler.m_start_process_time) << endl;
        profiler.reset();
    }
    return output;
}

