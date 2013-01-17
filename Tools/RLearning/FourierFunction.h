#ifndef FOURIERFUNCTION_H
#define FOURIERFUNCTION_H

#include <sstream>
#include <fstream>
#include <vector>
#include <cmath>
#include <cstdlib>

#include <map>
#include <string>
#include <vector>
#include <iostream>
using namespace std;
class FourierFunction
{
public:
    FourierFunction();
    void initialiseFunction(int order_k_, int num_inputs_m_, bool fully_coupled_, float learning_rate_alpha_, float max_period_);
    float evaluate(vector<float> const& input);

    void learn(vector<float> input, float value, int iterations = 1);

    string getSaveData();
    void loadSaveData(string save_data);
    float dotProd(vector<float> x, vector<float> y);


private:
    int order_k;
    int num_inputs_m;
    int number_of_basis_functions_n;//=(k+1)^m for fully coupled
    float max_period;


    bool fully_coupled;

    float learning_rate_alpha;

    vector<float> weights_w;
    vector<vector<float> > basis_constants_c;

    float PI;
    void generateConstants();
    void getNextConstant(vector<float> &c);
};

#endif // FOURIERFUNCTION_H
