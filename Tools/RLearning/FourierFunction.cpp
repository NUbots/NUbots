/*! @file FourierFunction.cpp
    @brief Class to model individual fourier function F:R^m -> R.
    Number of basis functions = (k+1)^m if coupled
                              = m*(k+1) otherwise
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
#include "FourierFunction.h"

FourierFunction::FourierFunction(){
 PI= atan(1)*4;
}
/*! @brief Initialises fourier function. Must be called before using a new function. If function loaded, this method should NOT be called as it will reset parameters and learnt function.
    @param  int order_k_ = order of fourier approximator, determines the highest frequency wave in the function pool.
            int num_inputs_m_ = number of inputs to the function
            bool fully_coupled_ = fully coupled true gives a stronger function at the cost of time and space complexity:
                                                                                number of functions when fully_coupled = (k+1)^m
                                                                                number of functions otherwise = m*(k+1), but approximation will be worse
            float learning_rate_alpha_ = stepsize which the function takes when learning
            float max_period_ = the largest of the expected ranges of the input data. Determines the lowest frequncy wave in the pool.
*/
void FourierFunction::initialiseFunction(int order_k_, int num_inputs_m_, bool fully_coupled_, float learning_rate_alpha_, float max_period_)
{
    order_k = order_k_;
    num_inputs_m = num_inputs_m_;
    fully_coupled = fully_coupled_;
    generateConstants();
    number_of_basis_functions_n = basis_constants_c.size();
    learning_rate_alpha = learning_rate_alpha_;
    weights_w = std::vector<float>(number_of_basis_functions_n,0);
    max_period = max_period_;
}
/*! @brief Returns string of info about function which can then be saved to disk and reloaded using loadSaveData(..)
*/
std::string FourierFunction::getSaveData()
{
    std::stringstream save_data;
    save_data<< order_k << "\n";
    save_data<< num_inputs_m << "\n";
    save_data <<max_period<<"\n";
    if(fully_coupled) {save_data<< 1 << "\n";} else {save_data<< 0 << "\n";}
    save_data<< learning_rate_alpha<< "\n";

    for (unsigned int i = 0; i<weights_w.size();i++){
        save_data<< weights_w[i] << " ";
        if (i%10 == 0 and i!=0) save_data<<"\n";
    }

    return save_data.str();

}
/*! @brief Loads function from std::string of saved data.
*/
void FourierFunction::loadSaveData(std::string save_data_)
{
    std::stringstream save_data(save_data_.c_str());
    save_data >> order_k;
    save_data >> num_inputs_m;
    save_data >> max_period;

    int fc;
    save_data >> fc;
    fully_coupled = false;
    if (fc == 1) fully_coupled = true;

    save_data >> learning_rate_alpha;

    generateConstants();
    number_of_basis_functions_n = basis_constants_c.size();
    weights_w = std::vector<float>(number_of_basis_functions_n,0);

    for (unsigned int i = 0; i<weights_w.size();i++){
        save_data >> weights_w[i];
    }


}
/*! @brief Evaluates the function at the point input in R^m
*/
float FourierFunction::evaluate(std::vector<float> const& input)
{
    float result = 0;
    for(int i = 0; i<number_of_basis_functions_n;i++){
        result+= weights_w[i]*cos(PI*dotProd(basis_constants_c[i],input)/max_period);

    }
    return result;
}
/*! @brief Moves the function closer the desired value at the given sample points using gradient descent update rule.
    @param input = sample point
           value = desired value
           iterations = number of learning iterations to take
*/
void FourierFunction::learn(std::vector<float> input, float value, int iterations)
{
    for(int i = 0; i<iterations; i++){
        float delta = value - evaluate(input);
        //calculate the phi std::vector and its norm for learning:
        std::vector<float> phi;
        for(int j = 0; j<number_of_basis_functions_n;j++){
            phi.push_back(cos(PI*dotProd(basis_constants_c[j],input)/max_period));
        }
        float norm_squared = 0;
        for(unsigned int j = 0; j<phi.size();j++){
            norm_squared+=phi[j]*phi[j];
        }

        if(norm_squared!=0)
            for (int j = 0; j< number_of_basis_functions_n; j++){
                weights_w[j]+=learning_rate_alpha*phi[j]*delta/norm_squared;
            }
    }
}



/*! @brief Calculates the dot product of two vectors
*/
float FourierFunction::dotProd(std::vector<float> x, std::vector<float> y)
{
    float result = 0;
    for (unsigned int i = 0; i<x.size(); i++){
        result += x[i]*y[i];
    }
    return result;
}
/*! @brief Generates the constant coefficients for the inputs to the function.
*/
void FourierFunction::generateConstants()
{
    basis_constants_c.clear();

    if(fully_coupled){
        std::vector<float> constant(num_inputs_m,0);//Create initial vector of zeros
        int num = (int)pow((double)order_k+1,(double)num_inputs_m);
        for(int i = 0; i<num;i++){
            basis_constants_c.push_back(constant);
            getNextConstant(constant);//Iterates as a base k+1 counter
        }
    }else{
        //Uncoupled: simply generate linearly independant constant vectors
        for(int i = 0; i<num_inputs_m;i++){
            for(int j = 0; j<=order_k;j++){
                std::vector<float> constant(num_inputs_m,0);
                constant[i] = j;
                basis_constants_c.push_back(constant);
            }
        }
    }
}
/*! @brief Iterates a length n vector as if it is a base k+1 number, with each entry representing one digit.
*/
void FourierFunction::getNextConstant(std::vector<float> &c)
{

    //Increments through number of length m in base k+1
    c[0]=((int)c[0]+1)%(order_k+1);//k is included by convention
    unsigned int i = 0;
    while((int)c[i]==0){
        i++;
        if(i>=c.size()) break;
        c[i]=((int)c[i]+1)%(order_k+1);
    }
}
