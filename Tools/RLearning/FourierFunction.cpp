#include "FourierFunction.h"

FourierFunction::FourierFunction(){
 PI= atan(1)*4;
}

void FourierFunction::initialiseFunction(int order_k_, int num_inputs_m_, bool fully_coupled_, float learning_rate_alpha_, float max_period_)
{
    order_k = order_k_;
    num_inputs_m = num_inputs_m_;
    fully_coupled = fully_coupled_;
    generateConstants();
    number_of_basis_functions_n = basis_constants_c.size();
    learning_rate_alpha = learning_rate_alpha_;
    weights_w = vector<float>(number_of_basis_functions_n,0);
    max_period = max_period_;
}

string FourierFunction::getSaveData()
{
    stringstream save_data;
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

void FourierFunction::loadSaveData(string save_data_)
{
    stringstream save_data(save_data_.c_str());
    save_data >> order_k;
    cout<<"Loaded order = "<< order_k<<endl;
    save_data >> num_inputs_m;
    cout<<"Loaded input size"<< num_inputs_m<<endl;
    save_data >> max_period;

    int fc;
    save_data >> fc;
    fully_coupled = false;
    if (fc == 1) fully_coupled = true;

    save_data >> learning_rate_alpha;

    generateConstants();
    number_of_basis_functions_n = basis_constants_c.size();
    weights_w = vector<float>(number_of_basis_functions_n,0);

    for (unsigned int i = 0; i<weights_w.size();i++){
        save_data >> weights_w[i];
    }


}

float FourierFunction::evaluate(vector<float> const& input)
{
    float result = 0;
    for(int i = 0; i<number_of_basis_functions_n;i++){
        result+= weights_w[i]*cos(2*PI*dotProd(basis_constants_c[i],input)/max_period);

    }
    return result;
}

void FourierFunction::learn(vector<float> input, float value, int iterations)
{
    for(int i = 0; i<iterations; i++){
        float delta = value - evaluate(input);
        //calculate the phi vector and its norm for learning:
        vector<float> phi;
        for(int j = 0; j<number_of_basis_functions_n;j++){
            phi.push_back(cos(2*PI*dotProd(basis_constants_c[j],input)/max_period));
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




float FourierFunction::dotProd(vector<float> x, vector<float> y)
{
    float result = 0;
    for (unsigned int i = 0; i<x.size(); i++){
        result += x[i]*y[i];
    }
    return result;
}

void FourierFunction::generateConstants()
{
    basis_constants_c.clear();

    if(fully_coupled){
        vector<float> constant(num_inputs_m,0);//Create initial vector of zeros
        int num = (int)pow((double)order_k+1,(double)num_inputs_m);
        for(int i = 0; i<num;i++){
            basis_constants_c.push_back(constant);
            getNextConstant(constant);
           /*
            cout<<"Constant Added And updated to ";
            for(unsigned int j = 0; j<constant.size();j++){
                cout<<constant[j]<<" ";
            }
            cout<<endl;
            */
        }
    }else{
        //Uncoupled: simply generate linearly independant constant vectors
        for(int i = 0; i<num_inputs_m;i++){
            for(int j = 0; j<=order_k;j++){
                vector<float> constant(num_inputs_m,0);
                constant[i] = j;
                basis_constants_c.push_back(constant);
            }
        }
    }
}

void FourierFunction::getNextConstant(vector<float> &c)
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
