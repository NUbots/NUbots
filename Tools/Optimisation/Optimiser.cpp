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
}

/*! @brief Destructor for the abstract optimiser */
Optimiser::~Optimiser()
{
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

