// ----- Includes ----- //
#include "ros/ros.h"
//#include "std_msgs/Float32.h"
#include "continuous_slopes/exo_joint_conf.h"
#include "continuous_slopes/exo_output.h"
#include "Exo_interface.h"
#include "Evolutionary.h"
#include <chrono>				//Timing the program.
#include<ctime> 
#include <sstream>
#include <array>				
#include <math.h>				
#include <iostream>				// Get output from terminal
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
    ros::Rate loop_rate(80);
	
	Exo_interface Exo(&nh); //exo ROS object

	std::vector<std::vector<double>> zeroVector = {{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},{}};


	//----------Variables----------// 
	int totalAI = 1;
	int amountOfHiddenLayers = 10;
	int trainingReps = 5;
	int totalGen = 1;
	int simTimeMax = 20;
	int testReps = 10;
	int simStopCondition = 0;

	double fitnessFunc = 0;
	double lengthWalked = 0;

	float simulationTime = 0;
	float simulationTimeOld = 0;

	bool saveTestData = true;
	bool restarting = false;
	bool success = false;
	bool restartAI = false;

	bool firstNet = false;
	
	//Folder structure: testFolderPath/NetworkTypeFolder/TrainingFolder/...
	string testFolderPath = "TestData";
	string networkTypeFolder = "";
	string trainingFolder = "/";
	string lastGenFolder = "/LastGen";

	string LoadFileName = "";

	string cinInput = "";
	int intCIN;

	ofstream fitnessfile;
	
	std::cout << "------------------------------------------------------" << std::endl;
	std::cout << "-------------Exoskelleton NeuralNet Tester------------" << std::endl;
	std::cout << "------------------------------------------------------" << std::endl;

	std::cout << "Do you want to save test data? (yes/no) " << std::endl;
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
	//Getting training network type
	cout << "Please Select number of hidden layers (2,5 or 10): " ;
	while(true)
	{
		cin >> intCIN;
		if(intCIN == 2)
		{
			networkTypeFolder = "/2LayerNet";
			break;	
		}
		else if(intCIN == 5)
		{
			networkTypeFolder = "/5LayerNet";
			break;
		}
		else if(intCIN == 10)
		{
			networkTypeFolder = "/10LayerNet";
		}
		else
			std::cout << "answer not valid, Try again. (2,5 or 10): ";
	}

	//Getting which AI training to test
	std::cout << "Please enter training folder name: (xxxxxxxxxx)"  << std::endl;
	cin >> cinInput;
	trainingFolder += cinInput;

	//Getting specific AI to test
	std::cout << "Please enter AI weightsFile(xxxxxxxx.csv) from folder: " << std::endl;  
	std::cout << testFolderPath + networkTypeFolder + trainingFolder << std::endl;
	cin >> cinInput;
	LoadFileName = testFolderPath + networkTypeFolder + trainingFolder + "/" + cinInput;

	std::cout << "Please set number of test runs: " ;
	cin >> trainingReps;

	debugCout("Setting up the Evolutionary AI object: ");
	Evolutionary AIs(LoadFileName, amountOfHiddenLayers);

	debugCout("initializing complet");
	if(saveTestData)
	{
		//----- Time stamp for the test folder: -----/
		time_t rawtime; 
		struct tm * timeinfo;
		time ( &rawtime );
		timeinfo = localtime ( &rawtime );
		string timeStamp = asctime (timeinfo);
		string testFolderPath = "TestData/Test_";
		for (size_t i = 0; i < timeStamp.size()-1; i++)
		{
			if(timeStamp[i] == ' ')
				testFolderPath += '_';
			else
				testFolderPath += timeStamp[i];
		}
		// Creating timestamped folder:
		string mkdir_cmd ="mkdir " + testFolderPath;
		system(mkdir_cmd.c_str()); 

		//-----Output to a file: -----/
		debugCout("Setting up outputfiles");
		string fitnessFileName = testFolderPath;
		string TrainingType = "AI_testing_" + to_string(testReps) + "reps_";
		fitnessFileName +=  "/fitnessFile_" + TrainingType + ".csv";
		debugCout(fitnessFileName);
		fitnessfile.open(fitnessFileName.c_str());
		string bestAIFileName = testFolderPath + "/bestAIWeightsGen";
	}

	debugCout("Inital setup Done!");

	
	//vector<vector<double>> inputVector;

	debugCout("Starting Training-loop");
	if(saveTestData)
	{
		fitnessfile << "Best AI from a training: " << LoadFileName << '\n';
		fitnessfile << "Testrun,Length Walked,Fitness Function \n";
	}

	auto teststart = std::chrono::steady_clock::now();
	for (int rep = 0; rep < trainingReps;)
	{
		if(saveTestData)
			fitnessfile << rep+1 << ",";
		
		std::cout << "Testrun " << rep + 1 << "/" << trainingReps << " " << endl;
		Exo.clearInput();
		auto repStart = std::chrono::steady_clock::now();
		restartAI = false;
		//debugCout("Starting while-ROS");
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

			//debugCout("set output");
			if (firstNet)
			{
				firstNet = false;
				Exo.setOutputMatrix(zeroVector);
			}
			Exo.setOutputMatrix(AIs.calculateOutput(0, Exo.getInputMatrix(), amountOfHiddenLayers));

			// ----- Check stopConditions ----- //
			//debugCout("Checking Stop conditions");
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
				lengthWalked = Exo.getHipXCoord();
				
				if(simStopCondition == 1)
				{
					fitnessFunc += (Exo.getHipXCoord() - (abs(Exo.getHipXCoord()) * 0.2));	
				}
				else if(simStopCondition == 2)
				{
					fitnessFunc += (Exo.getHipXCoord() + (abs(Exo.getHipXCoord()) * 0.1));
				}
				else if (simStopCondition == 3)
				{
					fitnessFunc += Exo.getHipXCoord()- (abs(Exo.getHipXCoord()) * 0.1);
				}
				else if (simStopCondition == 4)
				{
					fitnessFunc += (Exo.getHipXCoord() - (abs(Exo.getHipXCoord()) * 0.2));
				}
				cout << "	Length walked: " << lengthWalked << " Fitness func: " << fitnessFunc << endl;
				AIs.setFitness(0, fitnessFunc);
				if(saveTestData)
					fitnessfile << lengthWalked << "," << fitnessFunc << '\n';

				fitnessFunc = 0;
				Exo.stopSimulation();
				restarting = true;
			}
		}
		// Timing the progresses
		auto repEnd = std::chrono::steady_clock::now();
		double seconds = std::chrono::duration<double>(repEnd-repStart).count();
		int remainder = seconds*(trainingReps - rep);
		int minutes = remainder / 60;
		int hours = minutes / 60;
		std::cout << "Estimated time remaining: "<< int(hours) << " hours " << int(minutes%60) << " minutes " << int(remainder%60) << " seconds." << std::endl;
		rep++;
	}
	auto testEnd = std::chrono::steady_clock::now();
	double testSeconds = std::chrono::duration<double>(testEnd-teststart).count();
	int testRemainder = testSeconds;
	int testMinutes = testRemainder / 60;
	int testHours = testMinutes / 60;
	std::cout << "Test took: "<< int(testHours) << " hours " << int(testMinutes%60) << " minutes " << int(testRemainder%60) << " seconds." << std::endl;
	
	if(saveTestData)
	{
		fitnessfile << "Test took:," << "Hours," << " Minutes," << "Seconds. \n";
		fitnessfile << "Time:," << int(testHours) << "," <<  int(testMinutes%60) << "," << int(testRemainder%60);
		fitnessfile.close();
	}
	
	Exo.stopSimulation();
	cout << "Ending program" << endl;
	return 0; 
}
