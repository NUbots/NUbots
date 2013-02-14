
#ifndef Testing_MM_Utils_H
#define Testing_MM_Utils_H

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/foreach.hpp>
#include <string>
#include <sstream>
#include <vector>
#include <exception>
#include <boost/random/uniform_int.hpp>
#include <boost/random/uniform_real.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/variate_generator.hpp>

#include <boost/format.hpp> 

#include <sys/time.h>

#include "ConfigManager.h"
#include "ConfigRange.h"
//#include "ConfigStorageManager.h"

#include "Module.h"

#include "Testing_MM_Utils.h"

using namespace ConfigSystem;

extern boost::mt19937 seed; // defined in module

void initRandom();

void printTestResult(std::string testName, bool result);
timespec diff(timespec start, timespec end);
void startTimedTest();
void endTimedTest();

std::string makeRandomName(int len = 20);

template <typename T>
void createTestValue(
    T &val_out,
    T v_min = 0,
    T v_max = 100
    )
{
    boost::uniform_real<> distR(v_min, v_max);
    boost::variate_generator<boost::mt19937&, boost::uniform_real<> > randReal(seed,distR);
    val_out = (T)randReal();
}
template <typename T>
void create1DTestVector(
    std::vector<T> &vec_out,
    int s_min = 0, 
    int s_max = 5,
    T v_min = 0,
    T v_max = 100
    )
{
    // timeval t;
    // gettimeofday(&t,NULL);
    // boost::mt19937 seed( (int)t.tv_sec );
    boost::uniform_real<> distR(v_min, v_max);
    boost::variate_generator<boost::mt19937&, boost::uniform_real<> > randReal(seed,distR);
    boost::uniform_int<> distI(s_min, s_max);
    boost::variate_generator<boost::mt19937&, boost::uniform_int<> > randInt(seed,distI);

    vec_out = std::vector<T>();
    
    int  vecS = randInt();
    for(int i = 0; i < vecS; i++)
    {
        T val = (T)randReal();
        vec_out.push_back(val);
    }
}
template <typename T>
void create2DTestVector(
    std::vector<std::vector<T> > &vec_out,
    int s_min = 0, 
    int s_max = 10,
    T v_min = 0,
    T v_max = 100
    )
{
    // timeval t;
    // gettimeofday(&t,NULL);
    // boost::mt19937 seed( (int)t.tv_sec );
    boost::uniform_real<> distR(v_min, v_max);
    boost::variate_generator<boost::mt19937&, boost::uniform_real<> > randReal(seed,distR);
    boost::uniform_int<> distI(s_min, s_max);
    boost::variate_generator<boost::mt19937&, boost::uniform_int<> > randInt(seed,distI);

    vec_out = std::vector<std::vector<T> >();
    
    int  vecS = randInt();
    for(int i = 0; i < vecS; i++)
    {
        std::vector<T> val;
        create1DTestVector(val, s_min, s_max, v_min, v_max);
        vec_out.push_back(std::vector<T>(val));
    }
}
template <typename T>
void create3DTestVector(
    std::vector<std::vector<std::vector<T> > > &vec_out,
    int s_min = 0, 
    int s_max = 5,
    T v_min = 0,
    T v_max = 100
    )
{
    // timeval t;
    // gettimeofday(&t,NULL);
    // boost::mt19937 seed( (int)t.tv_sec );
    boost::uniform_real<> distR(v_min, v_max);
    boost::variate_generator<boost::mt19937&, boost::uniform_real<> > randReal(seed,distR);
    boost::uniform_int<> distI(s_min, s_max);
    boost::variate_generator<boost::mt19937&, boost::uniform_int<> > randInt(seed,distI);

    vec_out = std::vector<std::vector<std::vector<T> > >();
    
    int  vecS = randInt();
    for(int i = 0; i < vecS; i++)
    {
        std::vector<std::vector<T> > val;
        create2DTestVector(val, s_min, s_max, v_min, v_max);
        vec_out.push_back(std::vector<std::vector<T> >(val));
    }
}

template <typename T>
bool compareValues(T a, T b)
{
    // return ((float)a == (float)b);
    return ((long)(a * 100000) == (long)(b * 100000));
}

template <typename T>
bool compare1DVectors(
    std::vector<T> u, 
    std::vector<T> v, 
    bool print = true,
    int indent = 0
    )
{
    // Note: should define a null stream rather than using loads of 
    //       'if' statements for 'print'.

    std::stringstream ss;
    for(int i = 0; i < indent; i ++)
    {
        ss.width(4);
        ss << "|";
    }
    std::string str_i = ss.str();

    bool res_c = true;

    int su = u.size();
    int sv = v.size();
    int s_max = (su > sv) ? su : sv;
    int s_min = (su < sv) ? su : sv;

    res_c &= (su == sv);

    if(print) std::cout << str_i << "(" << su << ", " << sv << ")" << std::endl;;
    for(int i = 0; i < s_max; i++)
    {
        if(print) 
        {
            std::cout << str_i;

            std::cout.width(8);
            if(i < su)  std::cout << u[i];
            else        std::cout << "_";
            
            std::cout << ", ";

            std::cout.width(8);
            if(i < sv)  std::cout << v[i];
            else        std::cout << "_";
        }

        if(i < s_min)
        {
            // bool t_c = ((float)u[i] == (float)v[i]);
            bool t_c = compareValues(u[i], v[i]);
            res_c &= t_c;
            if(print) if(!t_c) std::cout << " *";
        }

        if(print) std::cout << std::endl;
    }

    return res_c;
}

template <typename T>
bool compare2DVectors(
    std::vector<std::vector<T> > u,
    std::vector<std::vector<T> > v,
    bool print = true,
    int indent = 0
    )
{
    std::stringstream ss;
    for(int i = 0; i < indent; i ++)
    {
        ss.width(4);
        ss << "|";
    }
    std::string str_i = ss.str();


    bool res_c = true;

    int su = u.size();
    int sv = v.size();
    int s_max = (su > sv) ? su : sv;
    int s_min = (su < sv) ? su : sv;

    res_c &= (su == sv);

    if(print) std::cout << str_i << "(" << su << ", " << sv << ")" << std::endl;
    for(int i = 0; i < s_max; i++)
    {
        std::vector<T> empty;
        if(print) std::cout << str_i << std::endl;
        if(i >=   su) compare1DVectors(empty, v[i], print, indent + 1);
        if(i >=   sv) compare1DVectors(u[i], empty, print, indent + 1);
        if(i < s_min) compare1DVectors(u[i],  v[i], print, indent + 1);
    }

    return res_c;
}


template <typename T>
bool compare3DVectors(
    std::vector<std::vector<std::vector<T> > > u,
    std::vector<std::vector<std::vector<T> > > v,
    bool print = true,
    int indent = 0
    )
{
    std::stringstream ss;
    for(int i = 0; i < indent; i ++)
    {
        ss.width(4);
        ss << "|";
    }
    std::string str_i = ss.str();


    bool res_c = true;

    int su = u.size();
    int sv = v.size();
    int s_max = (su > sv) ? su : sv;
    int s_min = (su < sv) ? su : sv;

    res_c &= (su == sv);

    if(print) std::cout << str_i << "(" << su << ", " << sv << ")" << std::endl;
    for(int i = 0; i < s_max; i++)
    {
        std::vector<std::vector<T> > empty;

        if(print) std::cout << str_i << std::endl;
        if(i >=   su) compare2DVectors(empty, v[i], print, indent + 1);
        if(i >=   sv) compare2DVectors(u[i], empty, print, indent + 1);
        if(i < s_min) compare2DVectors(u[i],  v[i], print, indent + 1);
    }

    return res_c;
}

#endif
