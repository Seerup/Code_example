#include "Evolutionary.h"


Evolutionary::Evolutionary(int amount_of_AI, int amount_of_hidden_layers)
{   
    
    vector<double> initColumn17(17, 0); 
    vector<double> initColumn16(16, 0); 
    vector<double> initColumn15(15, 0);
    vector<double> initColumn14(14, 0);  
    vector<double> initColumn13(13, 0); 
    vector<double> initColumn12(12, 0); 
    vector<double> initColumn11(11, 0);
    vector<double> initColumn10(10, 0);
    vector<double> initColumn9(9, 0);
    vector<double> initColumn8(8, 0);
    vector<double> initColumn7(7, 0);
    vector<double> initColumn6(6, 0);
    
    if (amount_of_hidden_layers == 2)
    {   
        // Makes the amount of rows, the initVector
        /*
        vector<vector<double>> weightsMatrix20by15(20, initColumn15); 
        vector<vector<double>> weightsMatrix15by10(15, initColumn10);
        vector<vector<double>> weightsMatrix10by6(10, initColumn6);
        */
        // Try to make it as close to AI4 as possible
        vector<vector<double>> weightsMatrix15by6(15, initColumn6); 
        vector<vector<double>> weightsMatrix6by6(6, initColumn6); 
        //vector<vector<double>> weightsMatrix6by6(6, initColumn6); 

        for(int k = 0; k < amount_of_AI; k++)
        {
            population.push_back(AI());
            //std::cout << "Making object: " << k;
            population[k].ID = k;
            population[k].fitness = 0;
            // Regular function 
            /*
            population[k].weightsList.push_back(weightsMatrix20by15);
            population[k].weightsList.push_back(weightsMatrix15by10);
            population[k].weightsList.push_back(weightsMatrix10by6); 
            */
            // Like AI4
            population[k].weightsList.push_back(weightsMatrix15by6);
            population[k].weightsList.push_back(weightsMatrix6by6);
            population[k].weightsList.push_back(weightsMatrix6by6); 

            for (int i = 0; i < population[k].weightsList.size(); i++)
            {
                for (int j = 0; j < population[k].weightsList[i].size(); j++)
                {
                    for (int l = 0; l < population[k].weightsList[i][j].size(); l++)
                    {
                        population[k].weightsList[i][j][l] = ((double) rand() / (RAND_MAX)) - 0.5;
                        //population[k].weightsList[i][j][l] = 0;
                    }
                }
            }
        }
    }   
    else if (amount_of_hidden_layers == 5)
    {
        // Makes the amount of rows, the initVector
        vector<vector<double>> weightsMatrix20by17(20, initColumn17);
        vector<vector<double>> weightsMatrix15by15(15, initColumn15); 
        vector<vector<double>> weightsMatrix15by14(15, initColumn14);
        vector<vector<double>> weightsMatrix14by12(14, initColumn12);
        vector<vector<double>> weightsMatrix12by10(12, initColumn10);
        vector<vector<double>> weightsMatrix10by8(10, initColumn8);
        vector<vector<double>> weightsMatrix8by6(8, initColumn6);
        
        for(int k = 0; k < amount_of_AI; k++)
        {
            population.push_back(AI());
            std::cout << "\r" << "Making object: " << k;
            population[k].ID = k;
            population[k].fitness = 0;
            population[k].weightsList.push_back(weightsMatrix15by15);
            population[k].weightsList.push_back(weightsMatrix15by15);
            population[k].weightsList.push_back(weightsMatrix15by14);
            population[k].weightsList.push_back(weightsMatrix14by12);
            population[k].weightsList.push_back(weightsMatrix12by10);
            population[k].weightsList.push_back(weightsMatrix10by8);
            population[k].weightsList.push_back(weightsMatrix8by6);

            for (int i = 0; i < population[k].weightsList.size(); i++)
            {
                for (int j = 0; j < population[k].weightsList[i].size(); j++)
                {
                    for (int l = 0; l < population[k].weightsList[i][j].size(); l++)
                    {
                        population[k].weightsList[i][j][l] = ((double) rand() / (RAND_MAX)) - 0.5;
                        //population[k].weightsList[i][j][l] = 0;
                    }  
                }
            }
        }
    }
    else if (amount_of_hidden_layers == 10)
    {
        // Makes the amount of rows, the initVector
        vector<vector<double>> weightsMatrix15by15(15, initColumn15);
        vector<vector<double>> weightsMatrix15by14(15, initColumn14);
        vector<vector<double>> weightsMatrix14by13(14, initColumn13);
        vector<vector<double>> weightsMatrix13by12(13, initColumn12);
        vector<vector<double>> weightsMatrix12by11(12, initColumn11);
        vector<vector<double>> weightsMatrix11by10(11, initColumn10);
        vector<vector<double>> weightsMatrix10by9(10, initColumn9);
        vector<vector<double>> weightsMatrix9by8(9, initColumn8);
        vector<vector<double>> weightsMatrix8by7(8, initColumn7);
        vector<vector<double>> weightsMatrix7by6(7, initColumn6);
        
        for(int k = 0; k < amount_of_AI; k++)
        {
            population.push_back(AI());
            std::cout << "\r" << "Making object: " << k;
            population[k].ID = k;
            population[k].fitness = 0;
            population[k].weightsList.push_back(weightsMatrix15by15);
            population[k].weightsList.push_back(weightsMatrix15by15);
            population[k].weightsList.push_back(weightsMatrix15by14);
            population[k].weightsList.push_back(weightsMatrix14by13);
            population[k].weightsList.push_back(weightsMatrix13by12);
            population[k].weightsList.push_back(weightsMatrix12by11);
            population[k].weightsList.push_back(weightsMatrix11by10);
            population[k].weightsList.push_back(weightsMatrix10by9);
            population[k].weightsList.push_back(weightsMatrix9by8);
            population[k].weightsList.push_back(weightsMatrix8by7);
            population[k].weightsList.push_back(weightsMatrix7by6);


            for (int i = 0; i < population[k].weightsList.size(); i++)
            {
                for (int j = 0; j < population[k].weightsList[i].size(); j++)
                {
                    for (int l = 0; l < population[k].weightsList[i][j].size(); l++)
                    {
                        population[k].weightsList[i][j][l] = ((double) rand() / (RAND_MAX)) - 0.5;
                        //population[k].weightsList[i][j][l] = 0;
                    }  
                }
            }
        }
    }
    else
    {
        std::cout << "No matching function for Evolutionary AI of that number! " << std::endl;
    }
    std::vector<double> initVectorOutput(6, 0);

    resultMatrix.push_back(initVectorOutput);
    std::cout << " Done" << std::endl;
}

Evolutionary::Evolutionary(string weightsFilePath, int amount_of_hidden_layers)
{
    population.push_back(AI());
    population[0].ID = 1;
    population[0].fitness = 0;
	vector<double> tempWeigthsVector;
    vector<vector<double>> tempLayerVector;

   
    std::cout << "Loading AI Weights from path:" << std::endl << weightsFilePath << std::endl;

     std::ifstream inputFile(weightsFilePath.c_str());

     if(!inputFile.is_open()) 
        throw std::runtime_error("Could not open file");

  
	std::string line;
    getline(inputFile, line); //read and erase the first line of the csv file 
	// Iterate through each line and split the content using delimeter
	while(getline(inputFile, line))
	{
        //tempWeigthsVector.push_back(std::stod(line));
        if(line.size() !=0)
        {
            line.pop_back();
            tempWeigthsVector.clear();
            stringstream ss(line);

            while( ss.good() )
            {
                string substr;
                getline( ss, substr, ',' );
                tempWeigthsVector.push_back(std::stod(substr));

            }
            tempLayerVector.push_back(tempWeigthsVector);
            //std::cout << tempWeigthsVector.size() << endl;
        }
        else
        {   
            population[0].weightsList.push_back(tempLayerVector);
            tempLayerVector.clear();
            getline(inputFile, line);
        }
	}
	// Close the File
	inputFile.close();
    std::vector<double> initVectorOutput(6, 0);
    resultMatrix.push_back(initVectorOutput);
    //printAI(0);
}

int Evolutionary::getID(int placement)
{
    return population[placement].ID;
}

float Evolutionary::getFitness(int placement)
{
    return population[placement].fitness;
}

void Evolutionary::setFitness(int ID, float fitness_val)
{
    population[ID].fitness = fitness_val;
}

double Evolutionary::calculateActivationSigmoid(double val)
{
    return (1 / (1+(exp(-val))));
}

double Evolutionary::calculateActivationTanH(double val)
{
    return (((2 / (1 + (exp(-2 * val)))) - 1) );
}


// Returns the weightsmatrix for a given placement for a given layer
std::vector<std::vector<double>> Evolutionary::getWeightsMatrix(int placement, int layer)
{
    return population[placement].weightsList[layer];
}

int Evolutionary::getAmountOfWeightsMatrices(int placement)
{
    return population[placement].weightsList.size();
}

int Evolutionary::getPopulationSize()
{
    return population.size();
}

void Evolutionary::sortPopulation()
{
    for(int i = 0; i < population.size() - 1;)
	{
        //std::cout << i << "  i = " << population[i].fitness << "  i + 1: " << population[i + 1].fitness; 
		if(population[i].fitness < population[i + 1].fitness)
		{
			std::swap(population[i], population[i + 1]);
			if(i != 0)
                i--;
            //std::cout << "  Swapped" << std::endl;
		}
        else
        {
            //std::cout << "  Not swapped" << std::endl;
            i++;
        }
	}
}

// Uses the casino algorithm to pick a partner with a higher probability of picking a better partner
int Evolutionary::pickPartner(int ownID, int topVal)
{
    int r1 = 0;
    int r2 = 0;
    bool breakVal = true;

    while (breakVal) 
    {
        // Pick a random value.
        r1 = (((int) rand() % topVal) + 1);
        // Pick a second random value.
        r2 = (((int) rand() % topVal) + 1);
        // Does it qualify? If so, we’re done!
        // std::cout << r1 << ", " << r2 << std::endl;
        if (r2 < r1 && (topVal - r1 != ownID)) 
        {
            breakVal = false;
        }
    }
    return (topVal - r1);
}

void Evolutionary::evolve()
{
    //std::cout << "Begun evolving" << std::endl;
    int halfPopulationSize = population.size() / 2;
    if(halfPopulationSize < 2)
    {
        std::cout << "can't evovle, too few to mate with" << std::endl;
    }
    float parent1Gene = 0;
    float parent2Gene = 0;
    mutation = 0;
    deviationFactor = 0;
    int parent2Index = 0;
    float partition = 0;
    
    //--------fitness Statistics:-------
    tempFitness = 0;
    mean = 0;
    variance = 0;
    stdDeviation = 0;
    bestAiDeviation = 0;
    for(int i = 0; i < halfPopulationSize; i++)
    {
        tempFitness += population[i].fitness;
    }
    mean = tempFitness/halfPopulationSize;
    
    for(int i = 0; i < halfPopulationSize; i++)
    {
        variance += pow(population[i].fitness - mean,2);
    }
    variance /= halfPopulationSize;

    stdDeviation = sqrt(variance);

    bestAiDeviation = (population[0].fitness - mean) / stdDeviation;

    if(3-bestAiDeviation > 0)
    {
        deviationFactor = 3 - bestAiDeviation;
    }
    std::cout << "Mean: " << mean << std::endl;
    std::cout << "Variance: " << variance << std::endl;
    std::cout << "Std Deviation: " << stdDeviation << std::endl;
    std::cout << "Best AI deviation : " << bestAiDeviation << std::endl;
    
    //--------fitness Statistics (END):-------

    for(int i = 0; i < halfPopulationSize; i++)
    {
        // ----- weights Input to Hidden layer 1 ----- //
        //std::cout << "Going through population" << std::endl;
        // Clear half the population

        parent2Index = pickPartner(i, halfPopulationSize);

        //std::cout << "parent1: " << i << " , parent2: " << parent2Index << std::endl;
        for (int l = 0; l < population[i].weightsList.size(); l++)
        {
            for (int j = 0; j < population[i].weightsList[l].size(); j++)
            {
                for (int k = 0; k < population[i].weightsList[l][j].size(); k++)
                {
                    //std::cout << "First dimension: " << i << " second dimension: " << j << std::endl;
                    population[halfPopulationSize + i].weightsList[l][j][k] = 0;
                    parent1Gene = population[i].weightsList[l][j][k];
                    // Add mutation:
 
                    parent2Gene = population[parent2Index].weightsList[l][j][k];
                    mutation = (((double) rand() / (RAND_MAX) - 0.5) / 5);
                    partition = (double) rand() / (RAND_MAX);
                    
                    population[halfPopulationSize + i].weightsList[l][j][k] = (parent1Gene * partition) + (parent2Gene * (1 - partition)) + mutation;// + (mutation*deviationFactor);
                } 
            }
        }
    }
    //std::cout << "Mutation : " << mutation << std::endl;
    std::cout << "DeviationFactor : " << deviationFactor << std::endl;
    //std::cout << "Mutation effect: " << mutation + (mutation*deviationFactor) << std::endl;
    //std::cout << "Evolution done" << std::endl;
}

void Evolutionary::printAI(int placement)
{
    std::cout << "Input to hidden" << endl;
    for (int i = 0; i < population[placement].weightsList.size(); i++)
    {
        std::cout << "layer:" << i << " to: " << i+1 << std::endl;

        for (int j = 0; j < population[placement].weightsList[i].size(); j++)
        {
            for (int k = 0; k < population[placement].weightsList[i][j].size(); k++)
            {
                std::cout << population[placement].weightsList[i][j][k] << ",";
                /*if(j != population[0].weightsList[i][j].size()-1)
                    weightsfile << ", ";*/
            }
            std::cout << std::endl;
        }
        std::cout << std::endl;
    }
}

void Evolutionary::saveWeightsAI(string path, int id)
{
    //std::cout << "Beginning saving best AI" << std::endl;
    std::ofstream weightsfile;
    //std::cout << "Save path: " << path << std::endl;
    weightsfile.open(path);

    for (int i = 0; i < population[id].weightsList.size(); i++)
    {
        weightsfile << "layer:" << i << " to: " << i+1 << "\n";

        for (int j = 0; j < population[id].weightsList[i].size(); j++)
        {
            for (int k = 0; k < population[id].weightsList[i][j].size(); k++)
            {
                weightsfile << population[id].weightsList[i][j][k] << ",";
                /*if(j != population[0].weightsList[i][j].size()-1)
                    weightsfile << ", ";*/
            }
            weightsfile << "\n";
        }
        weightsfile << "\n";
    }
    weightsfile.close();
    //std::cout << "Done saving best AI" << std::endl;
    
}

void Evolutionary::saveStatistics(string path)
{
    std::ofstream statisticsFile;
    statisticsFile.open(path, std::ios_base::app);
    statisticsFile << mean << "," << variance << "," << stdDeviation << "," << bestAiDeviation << "," << mutation << "," << deviationFactor << "," << mutation + (mutation*deviationFactor) << "\n";
    statisticsFile.close();
}

void Evolutionary::loadWeightsAI(string path, int id)
{
    population.push_back(AI());
    population[id].ID = id;
    population[id].fitness = 0;

	vector<double> tempWeigthsVector;
    vector<vector<double>> tempLayerVector;
   
    //std::cout << "Loading AI Weights from path: " << path << std::endl;

     std::ifstream inputFile(path.c_str());
     //std::ifstream inputFile("home/seerup/TestData/Test_Thu_May_14_20:50:23_2020/LastGen/lastGen_ID_0_Training_200gens_100AIs_10Layers.csv");

     if(!inputFile.is_open()) 
        throw std::runtime_error("Could not open file");

  
	std::string line;
    getline(inputFile, line); //read and erase the first line of the csv file 
	// Iterate through each line and split the content using delimeter
	while(getline(inputFile, line))
	{
        //tempWeigthsVector.push_back(std::stod(line));
        if(line.size() !=0)
        {
            line.pop_back();
            tempWeigthsVector.clear();
            stringstream ss(line);

            while( ss.good() )
            {
                string substr;
                getline( ss, substr, ',' );
                tempWeigthsVector.push_back(std::stod(substr));

            }
            tempLayerVector.push_back(tempWeigthsVector);
            //std::cout << tempWeigthsVector.size() << endl;
        }
        else
        {   
            population[id].weightsList.push_back(tempLayerVector);
            tempLayerVector.clear();
            getline(inputFile, line);
        }
	}
	// Close the File
	inputFile.close();
}

void Evolutionary::clearPopulation()
{
    population.clear();
}

std::vector<std::vector<double>> Evolutionary::calculateOutput(int id, std::vector<std::vector<double>> input, int amount_of_hidden_layers)
{
    //cout << "Print input size: " << input[0].size() << "ID: " << id << endl;
    for (int i = 0; i < 6; i++)
    {
        resultMatrix[0][i] = 0; 
       // cout << "setting up resultmatrix" << endl;
    }
    //std::cout << "getting weights matrix" << endl;
    resultMatrix = matrixMultiplication(input, getWeightsMatrix(id,0));

    for (int i = 0; i < resultMatrix.size(); i++)
    {
        resultMatrix[0][i] = calculateActivationTanH(resultMatrix[0][i]);
    }
    

    for (int j = 1; j < population[id].weightsList.size(); j++)
    {
        resultMatrix = matrixMultiplication(resultMatrix, getWeightsMatrix(id,j));

        for (int i = 0; i < resultMatrix.size(); i++)
        {
            resultMatrix[0][i] = calculateActivationTanH(resultMatrix[0][i]);
        }
        //cout << resultMatrix[0].size() << endl;
        /*for (int k = 0; k < resultMatrix[0].size(); k++)
        {
            cout << resultMatrix[0][k];
        }
        cout << endl;*/
    }
        
    return resultMatrix;
}

// Calculate the influence on all outputs for a given input and return it as a percentage value
// The weights are calculated as a sum of squares
std::vector<double> Evolutionary::calculateInfluenceFromInput(int AIID)
{
    std::vector<double> output;
    
    int amountOfInputs = population[AIID].weightsList[0].size();

    for (int i = 0; i < amountOfInputs; i++)
    {
        output.push_back(0.0);
    }
    
    double influenceWeights = 0;    // Weights from a given input to all outputs
    double totalWeights = 0;        // Total amount of weights

    for (int i = 0; i < 1; i++) //population[AIID].weightsList.size()
    {   
        for (int j = 0; j < population[AIID].weightsList[i].size(); j++)
        {
            for (int k = 0; k < population[AIID].weightsList[i][j].size(); k++)
            {
                //totalWeights += pow(population[AIID].weightsList[i][j][k],2);

                if (i==0)
                {
                    output[j] += pow(population[AIID].weightsList[i][j][k],2);
                }
                else
                {
                    for (int l = 0; l < output.size(); l++)
                    {
                        output[l] += pow(population[AIID].weightsList[i][j][k],2);
                    } 
                }
                /*
                cout << "b:" << population[AIID].weightsList[i][j][k] << ", ";
                cout << "a:" << sqrt(population[AIID].weightsList[i][j][k]) << " , ";
                cout << "n:" << totalWeights << ", ";
                cout << "r:" << population[AIID].weightsList[i][j][k] << ", ";
                cout << "i:" << influenceWeights << ", ";
                */
            }
        }
    }
    for (int i = 0; i < output.size(); i++)
    {
        totalWeights += output[i];
    }
    for (int i = 0; i < output.size(); i++)
    {
        output[i] = (output[i]/totalWeights) * 100;
    }

    return output;
}



