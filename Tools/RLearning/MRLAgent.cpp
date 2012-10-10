#include "MRLAgent.h"
#include <math.h>
MRLAgent::MRLAgent():RLAgent()
{
    FunctionApproximator();

}

void MRLAgent::initialiseAgent(int numberOfInputs, int numberOfOutputs, int numberOfHiddens){
    //MRLFunctionApprox FuncApprox(numberOfInputs, numberOfOutputs, numberOfHiddens);
    //FunctionApproximator = FuncApprox;
    FunctionApproximator.initialiseApproximator(numberOfInputs, numberOfOutputs, numberOfHiddens);
}

float MRLAgent::giveMotivationReward(){
    float novelty=0;
    for (int i=0; i<observations.size()-1;i++){
        for (int j=0; j<observations[i].size();j++){
            novelty+= exp(-i*0.01)*(observations[i][j]-observations[observations.size()-1][j])*(observations[i][j]-observations[observations.size()-1][j]);
            //novelty = sum of squared differences of current percept from all previous percepts, with time distant percepts decreasing in importance exponentially
        }
    }
    float motivation = wundtFunction(novelty);
    giveReward(motivation);
}

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

