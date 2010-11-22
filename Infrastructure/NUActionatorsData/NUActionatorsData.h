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
    const static id_t ChestLed;
    const static id_t LFootLed;
    const static id_t RFootLed;
    const static id_t LEyeLed;
    const static id_t REyeLed;
    const static id_t LEarLed;
    const static id_t REarLed;
    // Led groups
    const static id_t FaceLeds;
    const static id_t FeetLeds;
    const static id_t AllLeds;
    // Other actionators
    const static id_t Sound;
    const static id_t Teleporter;
public:
    NUActionatorsData();
    ~NUActionatorsData();
    
    void addActionators(const vector<string>& hardwarenames);
    
    void preProcess(double currenttime);
    void getNextServos(vector<float>& positions, vector<float>& gains);
    void getNextLeds(vector<vector<float> >& leds);
    void postProcess();
    
    bool isMemberOfGroup(const string& name, const id_t& group);
    size_t getSize(const id_t& actionatorid);
    
    void add(const id_t& actionatorid, double time, float data);
    void add(const id_t& actionatorid, double time, float data, float gain);
    void add(const id_t& actionatorid, double time, const vector<float>& data);
    void add(const id_t& actionatorid, double time, const vector<float>& data, float gain);
    void add(const id_t& actionatorid, double time, const vector<float>& data, const vector<float>& gain);
    void add(const id_t& actionatorid, double time, const vector<vector<float> >& data);
    void add(const id_t& actionatorid, double time, const vector<vector<vector<float> > >& data);
    void add(const id_t& actionatorid, double time, const string& data);
    
    void add(const id_t& actionatorid, const vector<double>& time, const vector<float>& data);
    void add(const id_t& actionatorid, const vector<double>& time, const vector<float>& data, float gain);
    void add(const id_t& actionatorid, const vector<double>& time, const vector<float>& data, const vector<float>& gain);
    void add(const id_t& actionatorid, const vector<double>& time, const vector<vector<float> >& data);
    void add(const id_t& actionatorid, const vector<double>& time, const vector<vector<vector<float> > >& data);
    void add(const id_t& actionatorid, const vector<double>& time, const vector<vector<vector<vector<float> > > >& data);
    void add(const id_t& actionatorid, const vector<double>& time, const vector<string>& data);

    void add(const id_t& actionatorid, const vector<vector<double> >& time, const vector<vector<float> >& data);
    void add(const id_t& actionatorid, const vector<vector<double> >& time, const vector<vector<float> >& data, float gain);
    void add(const id_t& actionatorid, const vector<vector<double> >& time, const vector<vector<float> >& data, const vector<float>& gain);
    void add(const id_t& actionatorid, const vector<vector<double> >& time, const vector<vector<float> >& data, const vector<vector<float> >& gain);
    void add(const id_t& actionatorid, const vector<vector<double> >& time, const vector<vector<vector<float> > >& data);
    void add(const id_t& actionatorid, const vector<vector<double> >& time, const vector<vector<vector<vector<float> > > >& data);
    
    void summaryTo(ostream& output);
    
    friend ostream& operator<< (ostream& output, const NUActionatorsData& p_sensor);
    friend istream& operator>> (istream& input, NUActionatorsData& p_sensor);

private:
    bool belongsToGroup(const id_t& member, const id_t& group);
    bool belongsToGroup(const string& member, const id_t& group);
    float interpolate(const double& time, const float& current, const float& target);

private:
    static vector<id_t*> m_ids;								   //!< a vector containing ALL of the actionator ids (even the ones which aren't available)
    vector<Actionator> m_actionators;                          //!< a vector containing ALL actionators (even the ones which aren't available)
};

#endif

