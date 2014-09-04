/*! @file FourierApproximator.cpp
    @brief Class to implement fourier approximator/network.
        Works by taking a weighted linear combination of a number of cosine functions for each output.
        Learning done by a gradient descent rule.
    Number of basis functions = num_outputs*(k+1)^num_inputs if coupled
                              = num_outputs*num_inputs*(k+1) otherwise


    @author Jake Fountain

 Copyright (c) 2013 Jake Fountain

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
#include "FourierApproximator.h"

FourierApproximator::FourierApproximator(bool fully_coupled, float learning_rate):ApproximatorInterface()
{
    this->fully_coupled = fully_coupled;
    this->learning_rate = learning_rate;
}

/*! @brief Saves approximator to config folder.
*/
void FourierApproximator::saveApproximator(std::string agentName)
{
    std::ofstream save_file;
    std::stringstream file_name;
    file_name<<save_location<<agentName;
    save_file.open(file_name.str().c_str(),std::fstream::out);

    save_file << num_inputs << "\n";
    save_file << num_outputs << "\n";
    save_file << learning_rate << "\n";
    if(fully_coupled) {save_file<< 1 << "\n";} else {save_file<< 0 << "\n";}

    for(int i = 0; i<num_outputs;i++){
        save_file << value_action_functions[i].getSaveData();
        save_file << "\n$ \n";
    }

    save_file.close();
}
/*! @brief loads approximator from nubot folder
*/
void FourierApproximator::loadApproximator(std::string agentName)
{

    std::ifstream save_file;
    std::stringstream file_name;
    file_name<<save_location<<agentName;
    save_file.open(file_name.str().c_str(),std::fstream::in);
    if(!save_file.good()) {
        throw std::string("FourierApproximator::loadApproximator - file not found: ") + file_name.str();
    }
    save_file>>num_inputs;
    save_file>>num_outputs;
    save_file>>learning_rate;
    int fc;
    save_file >> fc;
    fully_coupled = false;
    if (fc == 1) fully_coupled = true;

    value_action_functions.clear();
    std::string data = "a";
    std::string marker = "$";
    for(int i = 0;i<num_outputs;i++){
        save_file >> data;
        std::stringstream function_data;
        while (data!=marker){
            function_data << data << " ";
            save_file >> data;

            if(!save_file.good()) {
                throw std::string("FourierApproximator::loadApproximator - file corrupt ") + file_name.str();
              //std::cout<<"FourierApproximator::loadApproximator - file corrupt " << file_name.str()<<std::endl;
            }
        }
        FourierFunction f;
        f.loadSaveData(function_data.str());
        value_action_functions.push_back(f);
    }

    save_file.close();
}


/*! @brief Initialises approximator: call iff approx not loaded.
*/
void FourierApproximator::initialiseApproximator(int numberOfInputs, int numberOfOutputs, int numberOfHiddens, float max_parameter_range)
{
    num_inputs = numberOfInputs;
    num_outputs = numberOfOutputs;
    value_action_functions.clear();
    for(int i = 0; i<num_outputs;i++){
        FourierFunction f;
        f.initialiseFunction(numberOfHiddens,num_inputs,fully_coupled, 0.01, max_parameter_range);
        value_action_functions.push_back(f);
    }

}
/*! @brief Does learning for each output fourier function
*/
void FourierApproximator::doLearningEpisode(std::vector<std::vector<float> > const& observations, std::vector< std::vector<float> > const& values, float stepSize, int iterations)
{

    for(int obs = 0; obs<observations.size();obs++){
        for(int i = 0; i<num_outputs;i++){
            value_action_functions[i].learn(observations[obs],values[obs][i],iterations);
        }
    }

}
/*! @brief Evaluates each output function
*/
std::vector<float> FourierApproximator::getValues(std::vector<float> const& observations)
{
    std::vector<float> result;
    for(int i = 0; i<num_outputs;i++){
        result.push_back(value_action_functions[i].evaluate(observations));
    }
    return result;

}
