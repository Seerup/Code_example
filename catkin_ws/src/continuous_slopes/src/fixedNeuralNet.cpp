
// ----- Includes ----- //
#include "ros/ros.h"
#include "continuous_slopes/exo_joint_conf.h"
#include "continuous_slopes/exo_output.h"
#include "Exo_interface.h"
#include "Evolutionary.h"
#include <chrono>				//Timing the program.
#include <ctime> 
#include <sstream>
#include <array>				
#include <math.h>				
#include <iostream>				// Get input from terminal
#include <fstream>				// Write data to files
#include <stdlib.h>				// Use srand and rand
#include <string>				
#include <vector>

// ----- Namespaces ----- //
using namespace std;

// print debug messages: 
bool debug = true;
void debugCout(string text)
{
	if(debug)
		cout << "debug - " << text << endl;
}

int main(int argc, char **argv) 
{
	debugCout("Initializing program");
	// Transfer function calculation:
	srand((unsigned) time(0));
	int simState = 5;
	
	//----------ROS Setup----------//
    ros::init(argc, argv, "exo_interface");
    ros::NodeHandle nh;
    ros::Rate loop_rate(15);
	Exo_interface Exo(&nh);

	//----------Variables----------// 
	int totalAI = 1;
	int amountOfHiddenLayers = 0;
	int trainingReps = 1;
	int totalGen = 1;
	int simTimeMax = 20;

	double fitnessFunc = 0;
	double fitnessVal = 0;
	double lengthWalked = 0;

	bool saveTestData = false;
	bool loadPopulation = false;
	bool restarting = false;
	bool success = false;
	bool restartAI = false;

	ofstream fitnessfile;
	ofstream SetupFile;

	float simulationTime = 0;
	float simulationTimeOld = 0;

	int simStopCondition = 0;
	int Gen = 0;
	string cinInput = "";

	//Folder structure: testFolderPath/NetworkTypeFolder/TrainingFolder/...
	string testFolderPath = "TestData";
	string networkTypeFolder = "";
	string trainingFolder = "";
	string lastGenFolder = "/LastGen";
	string loadFolder = "" ;
	string logEntry;

	string bestAIFileName = "";
	string lastGenFileName = "";
	string fitnessFileName = "";
	string trainingDataFileName = "";
	string LoadFile = "";
	string TrainingType = "";
	string tempCIN = "";
	string supportType = "";
	int intCIN;
	
	std::cout << "----------------------------------------" << std::endl;
	std::cout << "-----Exoskelleton NeuralNet Trainer-----" << std::endl;
	std::cout << "----------------------------------------" << std::endl;

	std::cout << "Which type of support are used?" << std::endl;
	cin >> supportType;

	std::cout << "Do you want to save Training data? (yes/no): " << std::endl;
	while(true)
	{
		cin >> cinInput;
		if(cinInput == "yes")
		{
			saveTestData = true;
			break;
		}
		else if(cinInput == "no")
		{
			saveTestData = false;
			break;
		}
		else
			std::cout << "answer not valid, Try again. (yes/no)" << std::endl;
	}

	std::cout << "Please Select number of hidden layers (2,5 or 10): " ;
	while(true)
	{
		cin >> intCIN;
		if(intCIN == 2)
		{
			amountOfHiddenLayers = 2;
			networkTypeFolder = "/2LayerNet";
			break;	
		}
		else if(intCIN == 5)
		{
			amountOfHiddenLayers = 5;
			networkTypeFolder = "/5LayerNet";
			break;
		}
		else if(intCIN == 10)
		{
			amountOfHiddenLayers = 10;
			networkTypeFolder = "/10LayerNet";
			break;
		}
		else
			std::cout << "answer not valid, Try again. (2,5 or 10): ";
	}

	cout<< "Do you want to load an entire population? (yes/no): " ;
	while(true)
	{
		cin >> cinInput;
		if(cinInput == "yes")
		{
			loadPopulation = true;
			cout << "Please specify Training folder (xxxxxxxx):";
			cin >> tempCIN;
			loadFolder = testFolderPath + networkTypeFolder + "/" + tempCIN + lastGenFolder;	
			break;
		}
		else if(cinInput == "no")
		{
			loadPopulation = false;
			break;
		}
		else
			std::cout << "answer not valid, Try again. (yes/no)" << std::endl;
	}

	cout << "Enter description of the training to the log file: (Without Spaces)" << std::endl;
	//std::getline(cin, logEntry);
	
	cout << "Set number of AI pr. generation (must be 6 or greater): " ;
	cin >> totalAI; 

	cout << "Set number of generations: " ;
	cin >> totalGen; 

	debugCout("Setting up the Evolutionary AI object: ");
	Evolutionary AIs(totalAI, amountOfHiddenLayers);
	debugCout("Done");

	if(loadPopulation)
	{
		AIs.clearPopulation();
		for (size_t i = 0; i < totalAI; i++) 
		{
			LoadFile = loadFolder + "/lastGen_ID_" + to_string(i) + ".csv"; // for older trainings remember Training type!!
			AIs.loadWeightsAI(LoadFile, i);
		}
	}

	if(saveTestData)
	{
		//----- Time stamp for the training folder: -----/
		time_t rawtime; 
		struct tm * timeinfo;
		time ( &rawtime );
		timeinfo = localtime ( &rawtime );
		string timeStamp = to_string(timeinfo->tm_mday  + 1) + "-" + to_string(timeinfo->tm_mon + 1) + "-" + to_string(timeinfo->tm_year + 1900) + "_" + to_string(timeinfo->tm_hour) + ":" + to_string(timeinfo->tm_min);
		//std::cout << timeinfo->tm_hour << ":" << timeinfo->tm_min << std::endl;
		trainingFolder = timeStamp;
		//std::cout << trainingFolder << std::endl;
		//std::cout << "date: " << (timeinfo->tm_mday  + 1)  << '-' << timeinfo->tm_mon + 1 << '-' << (timeinfo->tm_year + 1900) << std::endl;
		

		// Creating timestamped folder and sub folder for last gen:
		string mkdir_cmd ="mkdir " + testFolderPath + networkTypeFolder + "/" + trainingFolder;
		system(mkdir_cmd.c_str()); 
		mkdir_cmd += lastGenFolder;
		system(mkdir_cmd.c_str());

		//-----Output to a file: -----/
		debugCout("Setting up outputfiles");
		TrainingType = "Training_" + to_string(totalGen) + "gens_" + to_string(totalAI) + "AIs";
		fitnessFileName = testFolderPath + networkTypeFolder + "/" + trainingFolder + "/fitnessFile_" + TrainingType + ".csv";
		
		bestAIFileName = testFolderPath + networkTypeFolder + "/" + trainingFolder + "/bestAIWeightsGen";
		lastGenFileName = testFolderPath + networkTypeFolder + "/" + trainingFolder + lastGenFolder + "/lastGen_ID_";
		
		trainingDataFileName = testFolderPath + networkTypeFolder + "/" + trainingFolder + "/TraningSetup_" + to_string(totalGen) + "gens_" + to_string(totalAI) + "AIs.txt";

		SetupFile.open(trainingDataFileName.c_str(), std::ios_base::app);
		SetupFile << "---------------------------------------- \n";
		SetupFile << "-----Exoskelleton NeuralNet Trainer----- \n";
		SetupFile << "--------------Data Log------------------ \n \n";
		SetupFile << "Training Description: " << supportType << "\n";
		if(loadPopulation){
			SetupFile << "Population Loaded from: " << LoadFile << "\n"; 
		}
		SetupFile << "Generations: " << to_string(totalGen) << "\n";
		SetupFile << "Number of AI: " << to_string(totalAI) << "\n";
		SetupFile << "Network Type: " << to_string(amountOfHiddenLayers) <<  " Hidden Layers \n";
		SetupFile << "Training started: " << timeStamp << "\n";
		SetupFile.close();
	}

	debugCout("initializing complet Starting Training Loop");

	if(saveTestData)
	{
		std::cout << "Fitness Path: " << fitnessFileName << std::endl;
		fitnessfile.open(fitnessFileName.c_str(), std::ios_base::app);
		fitnessfile << "Fitness functions of each AI though the different generations. ,";
		for (size_t i = 1; i <= totalAI; i++)
		{
			fitnessfile << i << ","; 
		}
		fitnessfile << "\n";	
		fitnessfile.close();
	}
	auto startGen = std::chrono::steady_clock::now();
	for (; Gen < totalGen;)
	{
		if(saveTestData)
		{
			fitnessfile.open(fitnessFileName.c_str(), std::ios_base::app);
			fitnessfile << "Gen " << Gen + 1 << ",";
			fitnessfile.close();
		}
		
		auto start = std::chrono::steady_clock::now();
		cout << "Generation: " << Gen + 1 << " / " << totalGen << endl;
		int curorg=0;

		for(; curorg < AIs.getPopulationSize();)
		{
			Exo.clearInput();
			cout << "Organism: " << curorg + 1  << " / " << AIs.getPopulationSize() << " "; // << endl;
			//AIs.printAI(curorg);
			fitnessFunc = 0;
			lengthWalked = 0;

			for (int rep = 0; rep < trainingReps;)
			{
				restartAI = false;
				while(ros::ok() && restartAI == false)
				{
					simState = Exo.getSimState();

					ros::spinOnce();
					loop_rate.sleep();
					
					//cout << "simState: " << Exo.getSimState() << endl;

					simulationTimeOld = simulationTime;
					simulationTime = Exo.getSimTime();
					if(simulationTime < simulationTimeOld)
						restarting = false;
					
					if(Exo.getHipDistance() >= 0 && simState == 0)
					{
						simStopCondition = 0;
						Exo.startSimulation();
					}
					Exo.setOutputMatrix(AIs.calculateOutput(curorg, Exo.getInputMatrix(), amountOfHiddenLayers));
					//Exo.setOutputMatrixVel(AIs.calculateOutput(curorg, Exo.getInputMatrix(), amountOfHiddenLayers));
					

					// ----- Check stopConditions ----- //
					if(restarting == false)
					{
						simStopCondition = Exo.checkStopCondition(simTimeMax);
					}
					else
					{
						simStopCondition = 0;
					}

					// ----- Stop according to stop condition ----- // 
					if (simStopCondition != 0)
					{
						restartAI = true;
						lengthWalked += Exo.getHipXCoord();

						if(simStopCondition == 1) // "Reset 1 - Too close to floor"
						{
							fitnessFunc += (Exo.getHipXCoord() - (abs(Exo.getHipXCoord()) * 0.3));	
						}
						else if(simStopCondition == 2) //"Reset 2 - Timeout"
						{
							fitnessFunc += (Exo.getHipXCoord() + (abs(Exo.getHipXCoord()) * 0.1));
						}
						else if (simStopCondition == 3) //"Reset 3 - Didn't move"
						{
							fitnessFunc += Exo.getHipXCoord()- (abs(Exo.getHipXCoord()) * 0.0);
						}
						else if (simStopCondition == 4) //"Reset 4 - Flying!"
						{
							fitnessFunc += (Exo.getHipXCoord() - (abs(Exo.getHipXCoord()) * 0.1));
						}/*  */
						else if(simStopCondition == 5) //"Reset 5 - Cancan"
						{
							fitnessFunc += (Exo.getHipXCoord() - (abs(Exo.getHipXCoord()) * 0.3));
						}
						else if(simStopCondition == 6) //"Reset 6 - Sitting down!"/*  */
						{
							fitnessFunc += (Exo.getHipXCoord() - (abs(Exo.getHipXCoord()) * 0.2));
						}
						/*else if(simStopCondition == 7) //"Reset 7 - Wrong step!"
						{
							fitnessFunc += (Exo.getHipXCoord() - (abs(Exo.getHipXCoord()) * 0.1));
						}*/

						if (rep >= trainingReps - 1)
						{
							fitnessVal = (fitnessFunc / trainingReps);
							AIs.setFitness(curorg, fitnessVal);
							cout << "	Length walked: " << lengthWalked / trainingReps << " Fitness func: " << fitnessVal << endl;
							if(saveTestData)
							{
								fitnessfile.open(fitnessFileName.c_str(), std::ios_base::app);
								fitnessfile << fitnessVal << ",";
								fitnessfile.close();
							}
						}
						Exo.stopSimulation();
						restarting = true;
					}
				}
				rep++;
			}
			curorg++;
		}	
		Gen++;
		AIs.sortPopulation();
		/*for (int i = 0; i < AIs.getPopulationSize(); i++)
		{
			cout << AIs.getFitness(i) << endl;
		}*/
		
		// Timing the progresses
		auto end = std::chrono::steady_clock::now();
		double seconds = std::chrono::duration<double>(end-start).count();
		int remainder = seconds*(totalGen - Gen);
		int minutes = remainder / 60;
		int hours = minutes / 60;
		std::cout << "Estimated time remaining: "<< int(hours) << " hours " << int(minutes%60) << " minutes " << int(remainder%60) << " seconds." << std::endl;
		if(saveTestData)
		{
			
			fitnessfile.open(fitnessFileName.c_str(), std::ios_base::app);
			fitnessfile << "\n";
			fitnessfile.close();
			
			string tempPath = bestAIFileName + to_string(Gen) + "_" + TrainingType + ".csv" ;
			AIs.saveWeightsAI(tempPath, 0);
			//std::cout << tempPath << std::endl;
		}

		// ----- Evolve step ----- //
		AIs.evolve();
		cout << "Best fitness: " << AIs.getFitness(0) << endl;

		
	} //gen for loop
	AIs.sortPopulation();
	auto endGen = std::chrono::steady_clock::now();
	double seconds = std::chrono::duration<double>(endGen-startGen).count();
		
	int remainder = seconds;
	int minutes = remainder / 60;
	int hours = minutes / 60;

	std::cout << "Training took: "<< int(hours) << " hours " << int(minutes%60) << " minutes " << int(remainder%60) << " seconds." << std::endl;

	cout << "print the influences of the different inputs of the winning AI:" << endl;

	std::vector<double> inputsInPercent = AIs.calculateInfluenceFromInput(0);
	double totalPercent = 0;

	
	for (int i = 0; i < inputsInPercent.size(); i++)
	{
		cout << "Input:" << i << " - " << inputsInPercent[i]  << " % " << std::endl;
	}

	for (int i = 0; i < inputsInPercent.size(); i++)
	{
		totalPercent += inputsInPercent[i];
	}
	cout << "Total: " << totalPercent << std::endl;
	
	if(saveTestData)
	{
		//fitnessfile.open(fitnessFileName.c_str(), std::ios_base::app);
		//fitnessfile << "Training took:,Hours,Minutes,Seconds \n";
		//fitnessfile << "," << int(hours) << "," << int(minutes%60) << "," << int(remainder%60);
		//fitnessfile.close();
		for (size_t i = 0; i < totalAI; i++)
		{
			string tempPath = lastGenFileName + to_string(i) + ".csv" ;
			AIs.saveWeightsAI(tempPath, i);
		}
		SetupFile.open(trainingDataFileName.c_str(), std::ios_base::app);
		SetupFile << "Training took: " << int(hours) << ":" << int(minutes%60) << ":" << int(remainder%60);

		
		SetupFile << "\n" << "Input influence: \n";
		for (int i = 0; i < inputsInPercent.size(); i++)
		{
			SetupFile << "Input: " << i << ", " << inputsInPercent[i] << "\n";
		}
		SetupFile << "\n";
		SetupFile.close();
	}

	Exo.stopSimulation();


	cout << "Ending program" << endl;
	return 0; 
}
