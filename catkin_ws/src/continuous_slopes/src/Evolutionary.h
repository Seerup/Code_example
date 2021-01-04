
#ifndef EVOLUTIONARY_H

#include <vector>
#include <iostream>
#include <fstream>

#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32MultiArray.h"

#include <cstdlib>
#include <ctime>
#include <iostream>

#include "LinearAlgebra.h"
#include "CPG.h"

class Evolutionary : public LinearAlgebra
{
    public:
        Evolutionary(int amount_of_AI, int amount_of_hidden_layers);
        Evolutionary(string weightsFilePath, int amount_of_hidden_layers);

        int getID(int placement);
        int getPopulationSize();
        int pickPartner(int ownID, int topVal);

        float getFitness(int placement);

        void setFitness(int ID, float fitness_val);
        void sortPopulation();
        void evolve();
        void printAI(int placement);
        void saveWeightsAI(string path, int id);
        void saveStatistics(string path);
        void loadWeightsAI(string path, int id);
        void clearPopulation();

        double calculateActivationSigmoid(double val);
        double calculateActivationTanH(double val);
        
        std::vector<std::vector<double>> getWeightsMatrix(int placement, int layer);
        int getAmountOfWeightsMatrices(int placement);
        std::vector<std::vector<double>> calculateOutput(int id, std::vector<std::vector<double>> input, int amount_of_hidden_layers);

        std::vector<double> calculateInfluenceFromInput(int AIID);
         

    private:

        struct AI
        {
            int ID;
            float fitness;         
            std::vector<std::vector<std::vector<double>>> weightsList;
        };

        std::vector<AI> population;
        std::vector<double> initVectorWeights;
        std::vector<std::vector<double>> weightsMatrix;
        std::vector<std::vector<double>> weightsMatrix6by6;

        int dimensionX = 1;
        int dimensionY = 12;
        int weightsXDimension = 6;
        int weightsYDimension = 12;

        float tempFitness = 0;
        float mean = 0;
        float variance = 0;
        float stdDeviation = 0;
        float bestAiDeviation = 0;
        float mutation = 0;
        float deviationFactor = 0;
        
    /*
    std::vector<std::vector<double>> hiddenLayer1;
    std::vector<std::vector<double>> hiddenLayer2;
    std::vector<std::vector<double>> hiddenLayer3;
    std::vector<std::vector<double>> hiddenLayer4;
    std::vector<std::vector<double>> hiddenLayer5;
    */
    std::vector<std::vector<double>> resultMatrix;
};


#endif