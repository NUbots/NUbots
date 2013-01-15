#include "FourierApproximator.h"

FourierApproximator::FourierApproximator(bool fully_coupled, float learning_rate)
{
    this->fully_coupled = fully_coupled;
    this->learning_rate = learning_rate;
}


void FourierApproximator::saveApproximator(string agentName)
{
    ofstream save_file;
    stringstream file_name;
    file_name<<"nubot/"<<agentName;
    save_file.open(file_name.str().c_str(),fstream::out);

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

void FourierApproximator::loadApproximator(string agentName)
{

    ifstream save_file;
    stringstream file_name;
    file_name<<"nubot/"<<agentName;
    save_file.open(file_name.str().c_str(),fstream::in);
    if(!save_file.good()) {
        throw string("FourierApproximator::loadApproximator - file not found: ") + file_name.str();
    }
    save_file>>num_inputs;
    save_file>>num_outputs;
    save_file>>learning_rate;
    int fc;
    save_file >> fc;
    fully_coupled = false;
    if (fc == 1) fully_coupled = true;

    value_action_functions.clear();
    string data = "a";
    string marker = "$";
    for(int i = 0;i<num_outputs;i++){
        save_file >> data;
        stringstream function_data;
        while (data!=marker){
            function_data << data << " ";
            save_file >> data;

            if (!save_file.good()){
                cout<<"File Corrupt."<<endl;
                break;
            }
        }
        FourierFunction f;
        f.loadSaveData(function_data.str());
        value_action_functions.push_back(f);
    }

    save_file.close();
}



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

void FourierApproximator::doLearningEpisode(vector<vector<float> > const& observations, vector< vector<float> > const& values, float stepSize, int iterations)
{

    for(int obs = 0; obs<observations.size();obs++){
        for(int i = 0; i<num_outputs;i++){
            value_action_functions[i].learn(observations[obs],values[obs][i],iterations);
        }
    }

}

vector<float> FourierApproximator::getValues(vector<float> const& observations)
{
    vector<float> result;
    for(int i = 0; i<num_outputs;i++){
        result.push_back(value_action_functions[i].evaluate(observations));
    }
    return result;

}
