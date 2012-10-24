#include "MRLAgent.h"

#include <math.h>

MRLAgent::MRLAgent():RLAgent()
{
    DictionaryApproximator DA();
    FunctionApproximator = DA;
}



/*! @brief
        Initialises agent by initialising function approximator
*/
void MRLAgent::initialiseAgent(int numberOfInputs, int numberOfOutputs, int numberOfHiddens){
    //MRLFunctionApprox FuncApprox(numberOfInputs, numberOfOutputs, numberOfHiddens);
    //FunctionApproximator = FuncApprox;
    FunctionApproximator.initialiseApproximator(numberOfInputs, numberOfOutputs, numberOfHiddens);
    num_inputs = numberOfInputs;
    num_outputs = numberOfOutputs;
    num_hidden = numberOfHiddens;
    setParameters();
}



/*! @brief
        Main feature of the MRL agent. The novelty of the latest observation is calculated by taking the Euclidean norm
    metric of the previous observation vectors with the latest.The motivation reward is then calculated by taking the Wundt function
    of the novelty.*/
float MRLAgent::giveMotivationReward(){
    float novelty=0;
    float memory_constant = 0.9;
    int count = 0;//used to normalise the novelty
    for (int i=0; i<observations.size()-1;i++){
        for (int j=0; j<observations[i].size();j++){

            novelty+= memory_constant*(observations[i][j]-observations[observations.size()-1][j])*(observations[i][j]-observations[observations.size()-1][j]);
            memory_constant*=memory_constant;

            //novelty = sum of squared differences of current percept from all previous percepts, with time distant percepts decreasing in importance exponentially
        }
        count++;
    }
    float motivation = wundtFunction(novelty/count);
    giveReward(motivation);
}



/*! @brief
        The wundt function is a linear combination of two sigmoids. It is similar to a decapitated gaussian distribution.
*/
float MLRAgent::wundtFunction(float N){
    float N1 = 1;//Location of max positive gradient
    float N2 = 2;//Location of max negative gradient

    float M1 = 1;//Maximum motivation
    float M2 = 0;//Minimum motivation
    float M3 = M2-M1;//maximum negative reward

    float p1 = 1;//Max pos gradient
    float p2 = 1;//Max negative gradient

    //Positive reward
    float F1 = M1/(1+exp(-p1*(2*N-N1)));
    //Negative Reward
    float F2 = M3/(1+exp(-p2*(2*N-N2)));

    return F1-F2;

}

