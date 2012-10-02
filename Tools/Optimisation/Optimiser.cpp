/*! @file Optimiser.cpp
    @brief Implemenation of Optimiser class
 
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

#include "Optimiser.h"
#include "Parameter.h"

#include <boost/random.hpp>

#include "debug.h"
#include "nubotdataconfig.h"

/*! @brief Constructor for abstract optimiser
 	@param name the name of the optimiser. The name is used in debug logs, and is used for load/save filenames by default
 	@param parameters the initial seed for the optimisation
 */
Optimiser::Optimiser(std::string name, vector<Parameter> parameters)
{
    m_name = name;
    m_initial_parameters = parameters;

    #ifdef TARGET_IS_TRAINING
        m_microsec_starttime = boost::posix_time::microsec_clock::local_time();
    #endif

    srand(static_cast<unsigned int> (1e6*getRealTime()*getRealTime()*getRealTime()));
}

/*! @brief Destructor for the abstract optimiser */
Optimiser::~Optimiser()
{
}

/*! @brief A dummy implementation of a the multi-objective optimiser interface
 * 	@param fitness a vector of fitnesses, one entry for each of the objectives. The higher the fitness the better the parameters.
 */
void Optimiser::setParametersResult(const vector<float>& fitness)
{
	if (not fitness.empty())
		setParametersResult(fitness[0]);
}

/*! @brief Returns the optimiser's name
    @return the optimiser's name
*/
string& Optimiser::getName()
{
    return m_name;
}

/*! @brief Stream insertion operator for optimiser. Store the entire optimiser in the given stream
    @param o the output stream to save the optimiser to
   	@param optimiser the optimiser to save
*/
ostream& operator<<(ostream& o, const Optimiser& optimiser)
{
    optimiser.toStream(o);
    return o;
}

/*! @brief Stream insertion operator for optimiser. Store the entire optimiser in the given stream
    @param o the output stream to save the optimiser to
    @param optimiser the optimiser to save
 */
ostream& operator<<(ostream& o, const Optimiser* optimiser)
{
    if (optimiser != 0)
        o << *optimiser;
    return o;
}

/*! @brief Stream extraction operator for optimiser. Load the entire optimiser from the given stream
    @param i the input stream to load the optimiser from
    @param optimiser the optimiser to load
 */
istream& operator>>(istream& i, Optimiser& optimiser)
{
    optimiser.fromStream(i);
    return i;
}

/*! @brief Stream extraction operator for optimiser. Load the entire optimiser from the given stream
    @param i the input stream to load the optimiser from
    @param optimiser the optimiser to load
 */
istream& operator>>(istream& i, Optimiser* optimiser)
{
    if (optimiser != 0)
        i >> *optimiser;
    return i;
}

/*! @brief Saves the optimiser to a file called "m_name.log" */
void Optimiser::save()
{
    saveAs(m_name);
}

/*! @brief Saves the optimiser to a file called "name.log" */
void Optimiser::saveAs(string name)
{
    ofstream file((DATA_DIR + string("Optimisation/") + name + ".log").c_str());
    if (file.is_open())
    {
        file << this;
        file.close();
    }
    else
        debug << "Optimiser::saveAs(): Failed to open file " << name + ".log" << endl;
}

/*! @brief Loads the optimiser from a file called "m_name.log". If no file is found, the optimiser continues to use the current configuration. */
void Optimiser::load()
{
    string filepath = DATA_DIR + string("Optimisation/") + m_name + ".log";
    ifstream file(filepath.c_str());
    if (file.is_open())
    {
        file >> this;
        file.close();
    }
}

/*! @brief Returns a normal random variable from the normal distribution with mean and sigma
 * 	@param mean the mean of the normal distribution
 * 	@param sigma the standard deviation of the normal distribution
 * 	@return a float from the normal distribution
 */
float Optimiser::normalDistribution(float mean, float sigma)
{
    static unsigned int seed = 1e6*getRealTime()*getRealTime()*getRealTime();          // I am hoping that at least one of the three calls is different for each process
    static boost::mt19937 generator(seed);                       // you need to seed it here with an unsigned int!
    static boost::normal_distribution<float> distribution(0,1);
    static boost::variate_generator<boost::mt19937, boost::normal_distribution<float> > standardnorm(generator, distribution);
    
    float z = standardnorm();       // take a random variable from the standard normal distribution
    float x = mean + z*sigma;       // then scale it to belong to the specified normal distribution
    
    return x;
}

/*! @brief Returns a random number from a uniform distribution
 *	@param min the minimum value of the uniform range
 *	@param max the maximum value of the uniform range
 *	@return the random number
 */
float Optimiser::uniformDistribution(float min, float max)
{
	// We can't use boost's uniform distribution because it is buggy.
	return (max - min)*rand()/RAND_MAX + min;
}

double Optimiser::getRealTime()
{
    #ifdef TARGET_IS_TRAINING
        double timeinmilliseconds;
        boost::posix_time::ptime timenow;
        timenow = boost::posix_time::microsec_clock::local_time();
        timeinmilliseconds = (timenow - m_microsec_starttime).total_nanoseconds()/1e6;

        return timeinmilliseconds;
        //return (boost::posix_time::microsec_clock::universal_time() - boost::posix_time::from_time_t(0)).total_nanoseconds()/1e6;
    #else
        return Platform->getRealTime();
    #endif
}
