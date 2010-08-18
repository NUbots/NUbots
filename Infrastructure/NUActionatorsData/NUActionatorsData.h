/*! @file NUActionatorsData.h
    @brief Declaration of a actionator data class to store actionator data in a platform independent way
    @author Jason Kulk
 
    @class NUActionatorsData
    @brief A actionator class to store actionator data in a platform independent way
 
    @author Jason Kulk
 
  Copyright (c) 2009, 2010 Jason Kulk
 
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

#ifndef NUACTIONATORSDATA_H
#define NUACTIONATORSDATA_H

class Actionator;
#include "Infrastructure/NUData.h"

#include <vector>
#include <string>
using namespace std;

class NUActionatorsData : public NUData
{
public:
    // Led actionators
    static id_t ChestLed;
    static id_t LeftFootLed;
    static id_t RightFootLed;
    static id_t LeftEyeLed;
    static id_t RightEyeLed;
    static id_t LeftEarLed;
    static id_t RightEarLed;
    // Led groups
    static id_t FaceLeds;
    static id_t FeetLeds;
    static id_t AllLeds;
    // Other actionators
    static id_t Sound;
    static id_t Teleporter;
public:
    NUActionatorsData();
    ~NUActionatorsData();
    
    void addActionators(const vector<string>& hardwarenames);
    
    void preProcess(double currenttime);
    void get(vector<float>& data);
    void get(vector<vector<float> >& data);
    void get(vector<vector<vector<float> > >& data);
    void get(vector<vector<vector<vector<float> > > >& data);
    void get(vector<string>& data);
    void postProcess();
    
    vector<int>& getIndices(id_t actionatorid);
    size_t getSize(id_t actionatorid);
    
    void add(id_t actionatorid, double time, float data);
    void add(id_t actionatorid, double time, float data, float gain);
    void add(id_t actionatorid, double time, const vector<float>& data);
    void add(id_t actionatorid, double time, const vector<float>& data, float gain);
    void add(id_t actionatorid, double time, const vector<float>& data, const vector<float>& gain);
    void add(id_t actionatorid, double time, const vector<vector<float> >& data);
    void add(id_t actionatorid, double time, const vector<vector<vector<float> > >& data);
    void add(id_t actionatorid, double time, const string& data);
    
    void add(id_t actionatorid, const vector<double>& time, const vector<float>& data);
    void add(id_t actionatorid, const vector<double>& time, const vector<float>& data, float gain);
    void add(id_t actionatorid, const vector<double>& time, const vector<float>& data, const vector<float>& gain);
    void add(id_t actionatorid, const vector<double>& time, const vector<vector<float> >& data);
    void add(id_t actionatorid, const vector<double>& time, const vector<vector<vector<float> > >& data);
    void add(id_t actionatorid, const vector<double>& time, const vector<vector<vector<vector<float> > > >& data);
    void add(id_t actionatorid, const vector<double>& time, const vector<string>& data);

    void add(id_t actionatorid, const vector<vector<double> >& time, const vector<vector<float> >& data);
    void add(id_t actionatorid, const vector<vector<double> >& time, const vector<vector<float> >& data, float gain);
    void add(id_t actionatorid, const vector<vector<double> >& time, const vector<vector<float> >& data, const vector<float>& gain);
    void add(id_t actionatorid, const vector<vector<double> >& time, const vector<vector<float> >& data, const vector<vector<float> >& gain);
    void add(id_t actionatorid, const vector<vector<double> >& time, const vector<vector<vector<float> > >& data);
    void add(id_t actionatorid, const vector<vector<double> >& time, const vector<vector<vector<vector<float> > > >& data);
    
    void summaryTo(ostream& output);
    void csvTo(ostream& output);
    
    friend ostream& operator<< (ostream& output, const NUActionatorsData& p_sensor);
    friend istream& operator>> (istream& input, NUActionatorsData& p_sensor);
    
private:
    vector<vector<int> > m_id_to_indices;                      //!< a member to map id_t's to indices in m_actionators for each actionator in the id_t group 
    vector<Actionator> m_actionators;                          //!< a vector containing every Actionator
};

#endif

