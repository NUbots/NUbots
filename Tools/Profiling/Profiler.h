/*! @file Profiler.h
    @brief Declaration of Profiler class
 
    @class Profiler
    @brief A module to move the robot into a standing position.
 
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

#ifndef PROFILER_H
#define PROFILER_H

#include <string>
#include <vector>
#include <iostream>


class Profiler
{
public:
    Profiler(std::string name);
    ~Profiler();
    
    void start();
    void stop();
    void split(std::string name);
    void reset();
    
    friend std::ostream& operator<<(std::ostream& output, Profiler& profiler);
private:
    std::string m_name;
    
    double m_start_thread_time;
    double m_start_process_time;
    double m_start_real_time;
    
    std::vector<double> m_split_thread_times;
    std::vector<double> m_diff_thread_times;
    std::vector<double> m_split_process_times;
    std::vector<double> m_diff_process_times;
    std::vector<double> m_split_real_times;
    std::vector<double> m_diff_real_times;
    
    
    std::vector<std::string> m_split_names;
};

#endif

