#ifndef SELFORGANISINGMAP_H
#define SELFORGANISINGMAP_H

#endif // SELFORGANISINGMAP_H


#include <vector>
#include <iostream>
#include <string>
#include <cmath>
#include <stack>
#include <algorithm>

class SelfOrganisingMap{

public:

    SelfOrganisingMap(int percept_size, int number_of_neurons, int neighbourhood_population){
        PERCEPT_SIZE = percept_size;

    }

    ~SelfOrganisingMap();

    //Returns the index of the neuron with weights closest to last percept.
    /*May not be needed.
    int getLastWinningIndex(){
        return winningIndex;
    }*/

    /*Main method for class: allows input of percept and returns the output of each neuron
    Output of neuron is:  0 if not updated
                          1 if updated but not winner
                          2 if winner
    */
    std::vector<int> propagate(std::vector<double> percept){
        if (percept.size()!= PERCEPT_SIZE){
            std::cout << "Improper percept size." << std::endl;
            return NULL;
        }

        std::vector<int> winners = findWinners(percept);
        for (int i = 0; i<winners.size();i++){
            update(winners[i], percept);
        }

        std::vector<int> output = new std::vector<int>[NUM_NODES];
        for (int i = 0; i<NUM_NODES; i++){
            output[i] = 0;
        }

        output[winners[0]] += 1;//Best neuron has value 2 rather than 1, to distinguish it in habituating stage
        for (int i = 0; i<winners.size(); i++){
            output[winners[i]] += 1;
        }
        return output;
    }

private:
    std::vector<std::vector<double> > weights;
    int winningIndex;
    //Number of neurons to update inside the neighbourhood of the winning neuron.
    static int NEIGHBOURHOOD_POP;
    //Total number of nodes:
    static int NUM_NODES;
    //Expected percept size:
    static int PERCEPT_SIZE;
    //Persistant distances from percept to nodes. Saves mulitple computations.
    //std::vector<double> distances;

    //Metric to measure topology of neurons: takes two weight vectors and measures their seperation.
    double d(std::vector<double> x,std::vector<double> y){
        double sum_squares = 0;
        try{
            for (int i = 0; i<x.size();i++){
                sum_squares += (x[i]-y[i])*(x[i]-y[i]);
            }
        }catch(Exception e){
            std::cout<< "Distance between vectors cannot be measured."<<std::endl;
            return -1;
        }
        return sqrt(sum_squares);

    }

    //Compares tuples of distances and weight indices.
    bool sortFunction(std::vector<double> x, std::vector<double> y){
        return x[0]<y[0];
    }

    //Update rule for weights: takes in single neuron's weights pointer and environment percept input.
    void update(int index,std::vector<double> percept){
        for (int i = 0; i<percept.size(); i++){
            weights[index][i] += learning_rate*(percept[i]-weights[index][i]);
        }
    }

    //Returns indices of winning neurons in winning neighbourhood
    std::vector<int> findWinners(std::vector<double> percept){
        std::vector<std::vector<double> > indexedDistances = new std::vector<std::vector<double> >[NUM_NODES,2];//indexedDistances=[[d1,index1],...,[dn,indexn]]
        for (int i = 0; i<indexedDistances.size();i++){
            indexedDistances[i][0] = d(percept,weights[i]);
            indexedDistances[i][1] = i;//Store the index of each node associated with distance.
        }

        sort(indexedDistances.begin(),indexedDistances.end(),sortFunction);


        std::vector<int> winners = new std::vector<int>[NEIGHBOURHOOD_POP];
        for (int i = 0; i<winners.size();i++){
            winners[i] = (int)indexedDistances[i][2];
        }
        return winners;
    }



};
