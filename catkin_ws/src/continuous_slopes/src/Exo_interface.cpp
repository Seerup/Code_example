#include "Exo_interface.h"

Exo_interface::Exo_interface(ros::NodeHandle *nh) : nh_(*nh)
{
	jointAngles = nh_.subscribe("/exoskeletonAngles", 1, &Exo_interface::callback_angle, this);
/*	
	hipOrientationSub = nh_.subscribe("/hipOrientationToFloor", 1, &Exo_interface::callback_array_orientation, this);
	hipPositionSub = nh_.subscribe("/hipPositionToFloor", 1, &Exo_interface::callback_array_position, this);
	centerOfMass = nh_.subscribe("/centerOfMass", 1, &Exo_interface::callback_centerOfMass, this);
	solePositions = nh_.subscribe("/solePositions", 1, &Exo_interface::callback_solePositions, this);
	angularIMUVelocity = nh_.subscribe("/angularIMUVelocity", 1, &Exo_interface::callback_IMUVelocity, this);
*/
	jointPositions = nh_.advertise<continuous_slopes::exo_joint_conf>("exoskeletonJointPosition", 1);

	simTime = nh_.subscribe("/simulationTime", 1, &Exo_interface::callback_SimTime, this);
	startSim = nh_.advertise<std_msgs::Bool>("/startSimulation", 1);
	stopSim = nh_.advertise<std_msgs::Bool>("/stopSimulation", 1);
	simState = nh_.subscribe("/simulationState", 1, &Exo_interface::callback_simState, this);

	neuralNets = nh_.advertise<std_msgs::Float32MultiArray>("neuralNetwork",3);

	if (INPUT_TRANSFER_FUNCTION == "straight")
	{
		upper_limit_input = 1;
		lower_limit_input = -1;
	}

	transfer_function_extent = upper_limit_input - lower_limit_input;

	Hip_slope = (transfer_function_extent) / (Hip_flex - Hip_ext);
	Knee_slope = (transfer_function_extent) / (Knee_ext - Knee_flex);
	Ancle_slope = (transfer_function_extent) / (Ancle_ext - Ancle_flex);

	HipRollPitchYaw_slope = (transfer_function_extent) / (HipHigh - HipLow);

	Hip_b_val = lower_limit_input - (Hip_slope * Hip_ext);
	Knee_b_val = lower_limit_input - (Knee_slope * Knee_flex);
	Ancle_b_val = lower_limit_input - (Ancle_slope * Ancle_flex);

	HipRollPitchYaw_b_val = lower_limit_input - (HipRollPitchYaw_slope * HipLow);

	std::vector<double> initVectorInput(15, 0.5);
	std::vector<double> initVectorOutput(6, 0);


    for (int i = 0; i < 1; i++)
    {
        inputMatrix.push_back(initVectorInput);
        
    }
	for	(int j = 0; j < 6; j++)
	{
		outputMatrix.push_back(initVectorOutput);
	}

	neuralNet.layout.dim.push_back(std_msgs::MultiArrayDimension());
    neuralNet.layout.dim.push_back(std_msgs::MultiArrayDimension());

	//std::cout << "debug" << std::endl;
    neuralNet.layout.data_offset = 0;
}

void Exo_interface::setJointPosition(double sendArray[6])
{
	joint_msg.joint0 = sendArray[0];
	joint_msg.joint1 = sendArray[1];
	joint_msg.joint2 = sendArray[2];
	joint_msg.joint3 = sendArray[3];
	joint_msg.joint4 = sendArray[4];
	joint_msg.joint5 = sendArray[5];

	jointPositions.publish(joint_msg);
}

void Exo_interface::startSimulation()
{
	reset_msg_false.data = false;
	reset_msg_true.data = true;

	joint_msg.joint0 = 0;
	joint_msg.joint1 = 0;
	joint_msg.joint2 = 0;
	joint_msg.joint3 = 0;
	joint_msg.joint4 = 0;
	joint_msg.joint5 = 0;

	jointPositions.publish(joint_msg);

	startSim.publish(reset_msg_true);
	//stopSim.publish(reset_msg_false);
	stopSemaphore = false;
}

void Exo_interface::stopSimulation()
{
	reset_msg_false.data = false;
	reset_msg_true.data = true;

	earlierVal = 1000;

	/*joint_msg.data.clear();
	for (int i = 0; i < 6; i++)
	{
		joint_msg.data.push_back(0);
	}*/
	joint_msg.joint0 = 0;
	joint_msg.joint1 = 0;
	joint_msg.joint2 = 0;
	joint_msg.joint3 = 0;
	joint_msg.joint4 = 0;
	joint_msg.joint5 = 0;

	jointPositions.publish(joint_msg);


	stopSim.publish(reset_msg_true);
	//startSim.publish(reset_msg_false);
	stopSemaphore = true;
}

int Exo_interface::getSimState()
{
	return simStateValue;
}

double Exo_interface::getSimTime()
{
	return simulationTime;
}

int Exo_interface::checkStopCondition(int timeOutVal)
{// Check if simulation should stop
	double simTime = getSimTime();
	int returnVal = 0;
	//std::cout << "SimState: " << simStateValue << " simTime: " << simTime << std::endl;
	if(stopSemaphore == false)
	{
		if((getHipZCoord() < 0.6) && (simStateValue == 1) && (simTime > 1) && (simTime < timeOutVal))
		{
			std::cout << "Reset 1 - Too close to floor" << std::endl;
			//std::cout << "simState: " << getSimState() << std::endl;
			returnVal = 1;
			oldSimTime = 0;
		}
		else if(simTime > timeOutVal && (simStateValue == 1))
		{
			std::cout << "Reset 2 - Timeout" << std::endl;
			returnVal = 2;
			oldSimTime = 0;
		}
		else if(simTime > 2)
		{
			//std::cout << "simTime: " << simTime << " oldSimTime: " << oldSimTime << "gap: " <<  simTime - oldSimTime << std::endl;
			
			/*
			if (!checkRightFootOnFloor() && !checkLeftFootOnFloor())
			{
				std::cout << "Reset 4 - Flying!" << std::endl;
				returnVal = 4;
				oldSimTime = 0;
			}
			else if(getRightSolePositionZ() > getHipZCoord() || getLeftSolePositionZ() > getHipZCoord())
			{
				std::cout << "Reset 5 - Cancan" << std::endl;
				returnVal = 5;
			}
			else if((getHipXCoord() + 0.3) < getRightSolePositionX() && (getHipXCoord() + 0.3 < getLeftSolePositionX()))
			{
				std::cout << "Reset 6 - Sitting down!" << std::endl;
				returnVal = 6;
			}
			else if(!checkStep())
			{
				std::cout << "Reset 7 - Wrong step" << std::endl;
				returnVal = 7;
				stepStart = true;
			}*/
			if(simTime - oldSimTime > 2)
			{
				oldSimTime = simTime; 
				if (abs(getHipXCoord() - earlierVal) < 0.05)
				{
					//std::cout << "Distance: " << abs(getHipXCoord() - earlierVal) << std::endl;
					std::cout << "Reset 3 - Didn't move" << std::endl;
					returnVal = 3;
					oldSimTime = 0;
				}
				earlierVal = getHipXCoord();
			}
		}
	}
	return returnVal;
}

bool Exo_interface::checkStep()
{
	//std::cout << checkRightFootOnFloor() << ";" << checkLeftFootOnFloor() << std::endl;
	if(stepStart)
	{
		if(!checkRightFootOnFloor())
		{
			goodStep = true;
			stepStart = false;
			rightFootRaised = true;
		}
		else if(!checkLeftFootOnFloor())
		{
			goodStep = true;
			stepStart = false;
			leftFootRaised = true;
		}
	}
	if(rightFootRaised && goodStep)
	{
		if(checkRightFootOnFloor())
		{
			rightFootRaised = false;
			if(getRightSolePositionX() >= getLeftSolePositionX())
			{
				goodRightStep = true;
				goodStep = true;
			}
			else if(getRightSolePositionX() < getLeftSolePositionX())
			{
				goodRightStep = false;
				goodStep = false;
			}
		}
	}
	else if(leftFootRaised && goodStep)
	{
		if(checkLeftFootOnFloor())
		{
			leftFootRaised = false;
			if(getLeftSolePositionX() >= getRightSolePositionX())
			{
				goodLeftStep = true;
				goodStep = true;
			}
			else if(getLeftSolePositionX() < getRightSolePositionX())
			{
				goodLeftStep = false;
				goodStep = false;
			}
		}
	}
	else if(goodRightStep && goodStep)
	{
		if(!checkRightFootOnFloor())
		{
			goodRightStep = false;
			goodStep = false;
		}
		else if(!checkLeftFootOnFloor())
		{
			goodRightStep = false;
			leftFootRaised = true;
		}
	}
	else if(goodLeftStep && goodStep)
	{
		if(!checkLeftFootOnFloor())
		{
			goodLeftStep = false;
			goodStep = false;
		}
		else if(!checkRightFootOnFloor())
		{
			goodLeftStep = false;
			rightFootRaised = true;
		}
	}
	return goodStep;
}


double Exo_interface::getHipRollAngle()
{
	return hipOrientation[0];
}

double Exo_interface::getHipPitchAngle()
{
	return hipOrientation[1];
}

double Exo_interface::getHipYawAngle()
{
	return hipOrientation[2];
}

double Exo_interface::getHipDistance()
{
	return hipOrientation[3];
}

double Exo_interface::getHipXCoord()
{
	return hipPosition[0];
}

double Exo_interface::getHipYCoord()
{
	return hipPosition[1];
}

double Exo_interface::getHipZCoord()
{
	return hipPosition[2];
}

double Exo_interface::getRightHipAngle()
{
	return jointConfiguration[0];
}

double Exo_interface::getRightKneeAngle()
{
	return jointConfiguration[1];
}

double Exo_interface::getRightAncleAngle()
{
	return jointConfiguration[2];
}

double Exo_interface::getLeftHipAngle()
{
	return jointConfiguration[3];
}

double Exo_interface::getLeftKneeAngle()
{
	return jointConfiguration[4];
}

double Exo_interface::getLeftAncleAngle()
{
	return jointConfiguration[5];
}

double Exo_interface::getRightHeelForce()
{
	return soleContactArray[0];
}

double Exo_interface::getRightToeForce()
{
	return soleContactArray[1];
}

double Exo_interface::getLeftHeelForce()
{
	return soleContactArray[2];
}

double Exo_interface::getLeftToeForce()
{
	return soleContactArray[3];
}

double Exo_interface::getCenterOfMassX()
{
	return centerOfMassArray[0];
}

double Exo_interface::getCenterOfMassY()
{
	return centerOfMassArray[1];
}

double Exo_interface::getCenterOfMassZ()
{
	return centerOfMassArray[2];
}

double Exo_interface::getRightSolePositionX()
{
	return solePositionArray[0];
}

double Exo_interface::getRightSolePositionY()
{
	return solePositionArray[1];
}

double Exo_interface::getRightSolePositionZ()
{
	return solePositionArray[2];
}

double Exo_interface::getLeftSolePositionX()
{
	return solePositionArray[3];
}

double Exo_interface::getLeftSolePositionY()
{
	return solePositionArray[4];
}

double Exo_interface::getLeftSolePositionZ()
{
	return solePositionArray[5];
}


double Exo_interface::getRightFrontLegX()
{
	Cx = cos(hipOrientation[0]);
	Cy = cos(hipOrientation[1]);
	Cz = cos(hipOrientation[2]);

	Sx = sin(hipOrientation[0]);
	Sy = sin(hipOrientation[1]);
	Sz = sin(hipOrientation[2]);

	HipX = hipPosition[0];

	return ((Cy * Cz * Bx) + (Sx * Sy * Cz - Cx * Sz * (-By)) + (Cx * Sy * Cz + Sx * Sz * Bz) + HipX);
}

double Exo_interface::getRightFrontLegY()
{
	Cx = cos(hipOrientation[0]);
	Cy = cos(hipOrientation[1]);
	Cz = cos(hipOrientation[2]);

	Sx = sin(hipOrientation[0]);
	Sy = sin(hipOrientation[1]);
	Sz = sin(hipOrientation[2]);

	HipY = hipPosition[1];

	return ((Cy * Sz * Bx) + (Sx * Sy * Sz + Cx * Cz * (-By)) + (Cx * Sy * Sz - Sx * Cz * Bz) + HipY);
}

double Exo_interface::getLeftFrontLegX()
{
	Cx = cos(hipOrientation[0]);
	Cy = cos(hipOrientation[1]);
	Cz = cos(hipOrientation[2]);

	Sx = sin(hipOrientation[0]);
	Sy = sin(hipOrientation[1]);
	Sz = sin(hipOrientation[2]);

	HipX = hipPosition[0];

	return ((Cy * Cz * Bx) + (Sx * Sy * Cz - Cx * Sz * By) + (Cx * Sy * Cz + Sx * Sz * Bz) + HipX);
}

double Exo_interface::getLeftFrontLegY()
{
	Cx = cos(hipOrientation[0]);
	Cy = cos(hipOrientation[1]);
	Cz = cos(hipOrientation[2]);

	Sx = sin(hipOrientation[0]);
	Sy = sin(hipOrientation[1]);
	Sz = sin(hipOrientation[2]);

	HipY = hipPosition[1];

	return ((Cy * Sz * Bx) + (Sx * Sy * Sz + Cx * Cz * By) + (Cx * Sy * Sz - Sx * Cz * Bz) + HipY);
}


double Exo_interface::getRightFootProximityDistance()
{// Get function for the right foot proximity sensor of the exo skeleton if value is 0 then the sensor does not see anything in the vicinity.
	return soleProximitySensor[0];
}

double Exo_interface::getLeftFootProximityDistance()
{
	return soleProximitySensor[1];
}

int Exo_interface::checkCenterOfMass()
{	
	double balancePoint = 0;
	double rightFootX = getRightSolePositionX();
	double leftFootX = getLeftSolePositionX();
	double hipX = getHipXCoord();

	if(rightFootX > leftFootX)
	{
		balancePoint = rightFootX - leftFootX;
	}
	else
	{
		balancePoint = leftFootX - rightFootX;
	}

	if(hipX > balancePoint)
	{
		return 1;
	}
	else if(hipX = balancePoint)
	{
		return 0;
	}
	else if(hipX < balancePoint)
	{
		return -1;	
	}
	
}


double Exo_interface::calc_val_input_Right_Hip()
{
	return Hip_slope * jointConfiguration[0] + Hip_b_val;
}

double Exo_interface::calc_val_input_Right_Knee()
{
	return (- (Knee_slope * jointConfiguration[1] + Knee_b_val));
}

double Exo_interface::calc_val_input_Right_Ancle()
{
	return Ancle_slope * jointConfiguration[2] + Ancle_b_val;
}


double Exo_interface::calc_val_input_Left_Hip()
{
	return Hip_slope * jointConfiguration[3] + Hip_b_val;
}

double Exo_interface::calc_val_input_Left_Knee()
{
	return (-(Knee_slope * jointConfiguration[4] + Knee_b_val));
}

double Exo_interface::calc_val_input_Left_Ancle()
{
	return Ancle_slope * jointConfiguration[5] + Ancle_b_val;
}


double Exo_interface::calc_val_input_Right_Heel_Sensor()
{
	if (soleContactArray[0] < 5)
	{
		return 0;
	}
	else
	{
		return 1;
	}
}

double Exo_interface::calc_val_input_Right_Toe_Sensor()
{
	//cout << "foot value: " << input << endl;
	if (soleContactArray[1] < 5)
	{
		return 0;
	}
	else
	{
		return 1;
	}
}

double Exo_interface::calc_val_input_Left_Heel_Sensor()
{
	//cout << "foot value: " << input << endl;
	if (soleContactArray[2] < 5)
	{
		return 0;
	}
	else
	{
		return 1;
	}
}

double Exo_interface::calc_val_input_Left_Toe_Sensor()
{
	//cout << "foot value: " << input << endl;
	if (soleContactArray[3] < 5)
	{
		return 0;
	}
	else
	{
		return 1;
	}
}


bool Exo_interface::checkRightFootOnFloor()
{
	if(soleDistanceToFloor[0] < 0.05)
		{
			return 1;
		}
		else
		{
			return 0;
		}
}

bool Exo_interface::checkLeftFootOnFloor()
{
	if(soleDistanceToFloor[1] < 0.05)
		{
			return 1;
		}
		else
		{
			return 0;
		}
}


bool Exo_interface::calc_val_Right_ToeX()
{
	
	if(toeForceArray[0]<-1)
		return true;
	else
		return false;
}

bool Exo_interface::calc_val_Left_ToeX()
{
	if(toeForceArray[3]<-1)
			return true;
		else
			return false;
}


double Exo_interface::calc_val_input_HipRoll()
{
	return HipRollPitchYaw_slope * hipOrientation[0] + HipRollPitchYaw_b_val;
}

double Exo_interface::calc_val_input_HipPitch()
{
	return HipRollPitchYaw_slope * hipOrientation[1] + HipRollPitchYaw_b_val;
}

double Exo_interface::calc_val_input_HipYaw()
{
	return HipRollPitchYaw_slope * hipOrientation[2] + HipRollPitchYaw_b_val;
}


double Exo_interface::calc_val_input_HipX()
{
	return hipPosition[0];		//Needs to be corrected!
}

double Exo_interface::calc_val_input_HipY()
{
	return hipPosition[1];		//Needs to be corrected!
}

double Exo_interface::calc_val_input_HipZ()	
{
	return hipPosition[2];		//Needs to be corrected!
}


double Exo_interface::calc_val_input_HipAngularVelocity_phi()
{
	double output = 0;
	double vel = angularIMUVelocityArray[0];
	
	if (abs(vel) > highestPhi)
	{
		highestPhi = abs(vel);
	}
	if (vel != 0)
	{
		output = (vel / highestPhi);
	}
	
	return output;
}

double Exo_interface::calc_val_input_HipAngularVelocity_theta()
{
	double output;
	double vel = angularIMUVelocityArray[1];
	
	if (abs(vel) > highestTheta)
	{
		highestTheta = abs(vel);
	}
	if (vel != 0)
	{
		output = (vel / highestTheta);
	}
	return output;
}

double Exo_interface::calc_val_input_HipAngularVelocity_psi()
{
	double output = 0;
	double vel = angularIMUVelocityArray[2];
	
	if (abs(vel) > highestPsi)
	{
		highestPsi = abs(vel);
	}
	if (vel != 0)
	{
		output = (vel / highestPsi);
	}
	
	return output;
}


double Exo_interface::calc_cal_input_left_proximity_sensor()
{
	if (soleProximitySensor[1] == 0)
	{
		return 1;
	}
	else
	{
		return soleProximitySensor[1];
	}
}

double Exo_interface::calc_cal_input_right_proximity_sensor()
{
	if (soleProximitySensor[0] == 0)
	{
		return 1;
	}
	else
	{
		return soleProximitySensor[0];
	}
}


double Exo_interface::calc_val_output_Hip(double input)
{
	double result = (input - Hip_b_val) / Hip_slope;
	if (result > Hip_ext && result < Hip_flex)
	{
		return (input - Hip_b_val) / Hip_slope;
	}
	else if (result < Hip_ext)
	{
		return Hip_ext;
	}
	else
	{
		return Hip_flex;
	}
}

double Exo_interface::calc_val_output_Knee(double input)
{
	double result = (input - Knee_b_val) / Knee_slope;
	if (result > Knee_flex && result < Knee_ext)
	{
		return (input - Knee_b_val) / Knee_slope;
	}
	else if (result < Knee_flex)
	{
		return Knee_flex;
	}
	else
	{
		return Knee_ext;
	}
}

double Exo_interface::calc_val_output_Ancle(double input)
{
	double result = (input - Ancle_b_val) / Ancle_slope;
	if (result > Ancle_flex && result < Ancle_ext)
	{
		return (input - Ancle_b_val) / Ancle_slope;
	}
	else if (result < Ancle_flex)
	{
		return Ancle_flex;
	}
	else
	{
		return Ancle_ext;
	}
}


double Exo_interface::calc_vel_output(double input)
{
	return (input * (73/8));
}

std::vector<std::vector<double>> Exo_interface::getInputMatrix()
{
	// input 0  - Right hip
	// input 1  - Right knee
	// input 2  - Right ankle
	// input 3  - Left hip
	// input 4  - Left knee
	// input 5  - Left ankle

	// input 6  - Right foot heel sensor
	// input 7  - Right foot toe sensor
	// input 8  - Left foot heel sensor
	// input 9  - Left foot toe sensor
	
	// input 10 - Hip roll angle
	// input 11 - Hip pitch angle
	// input 12 - Hip yaw angle
	
	// input 13 - Hip X
	// input 14 - Hip Y
	// input 15 - Hip Z
	
	// input 16 - COM
	
	// input 17 - IMU roll velocity (phi)
	// input 18 - IMU pitch velocity (theta)
	// input 19 - IMU yaw velocity (psi)
	
	// input 20 - proximity sensor right
	// input 21 - proximity sensor left
	
	inputMatrix[0][0] = calc_val_input_Right_Hip();
	inputMatrix[0][1] = calc_val_input_Right_Knee();
	inputMatrix[0][2] = calc_val_input_Right_Ancle();

	inputMatrix[0][3] = calc_val_input_Left_Hip();
	inputMatrix[0][4] = calc_val_input_Left_Knee();
	inputMatrix[0][5] = calc_val_input_Left_Ancle();

	inputMatrix[0][6] = calc_val_input_Right_Heel_Sensor();
	inputMatrix[0][7] = calc_val_input_Right_Toe_Sensor();
	inputMatrix[0][8] = calc_val_input_Left_Heel_Sensor();
	inputMatrix[0][9] = calc_val_input_Left_Toe_Sensor();

	inputMatrix[0][10] = calc_val_input_HipAngularVelocity_phi();
	inputMatrix[0][11] = calc_val_input_HipAngularVelocity_theta();
	inputMatrix[0][12] = calc_val_input_HipAngularVelocity_psi();

	inputMatrix[0][13] = calc_cal_input_right_proximity_sensor();
	inputMatrix[0][14] = calc_cal_input_left_proximity_sensor();



	//inputMatrix[0][11] = checkCenterOfMass();
	

	/*
	inputMatrix[0][0] = calc_val_input_Right_Hip();
	inputMatrix[0][1] = calc_val_input_Right_Knee();
	inputMatrix[0][2] = calc_val_input_Right_Ancle();

	inputMatrix[0][3] = calc_val_input_Left_Hip();
	inputMatrix[0][4] = calc_val_input_Left_Knee();
	inputMatrix[0][5] = calc_val_input_Left_Ancle();

	inputMatrix[0][6] = calc_val_input_Right_Heel_Sensor();
	inputMatrix[0][7] = calc_val_input_Right_Toe_Sensor();
	inputMatrix[0][8] = calc_val_input_Left_Heel_Sensor();
	inputMatrix[0][9] = calc_val_input_Left_Toe_Sensor();

	inputMatrix[0][10] = calc_val_input_HipRoll();
	inputMatrix[0][11] = calc_val_input_HipPitch();
	inputMatrix[0][12] = calc_val_input_HipYaw();

	inputMatrix[0][13] = 0; //calc_val_input_HipX();
	inputMatrix[0][14] = 0; //calc_val_input_HipY();
	inputMatrix[0][15] = 0; //calc_val_input_HipZ();
	
	inputMatrix[0][16] = checkCenterOfMass();

	inputMatrix[0][17] = calc_val_input_HipAngularVelocity_phi();
	inputMatrix[0][18] = calc_val_input_HipAngularVelocity_theta();
	inputMatrix[0][19] = calc_val_input_HipAngularVelocity_psi();

	inputMatrix[0][20] = calc_cal_input_right_proximity_sensor();
	inputMatrix[0][21] = calc_cal_input_left_proximity_sensor();
	*/
	//std::cout << "sedning inputmatrix with size: " << inputMatrix[0].size() <<  std::endl; 

	return inputMatrix;
}


void Exo_interface::setOutputMatrix(std::vector<std::vector<double>> output)
{
	joint_msg.joint0 = calc_val_output_Hip(output[0][0]);
	joint_msg.joint1 = calc_val_output_Knee(output[0][1]);
	joint_msg.joint2 = calc_val_output_Ancle(output[0][2]);

	joint_msg.joint3 = calc_val_output_Hip(output[0][3]);
	joint_msg.joint4 = calc_val_output_Knee(output[0][4]);
	joint_msg.joint5 = calc_val_output_Ancle(output[0][5]);

	jointPositions.publish(joint_msg);
}

void Exo_interface::setOutputMatrix(double output[6])
{
	joint_msg.joint0 = calc_val_output_Hip(output[0]);
	joint_msg.joint1 = calc_val_output_Knee(output[1]);
	joint_msg.joint2 = calc_val_output_Ancle(output[2]);

	joint_msg.joint3 = calc_val_output_Hip(output[3]);
	joint_msg.joint4 = calc_val_output_Knee(output[4]);
	joint_msg.joint5 = calc_val_output_Ancle(output[5]);

	jointPositions.publish(joint_msg);
}

void Exo_interface::setOutputMatrixVel(std::vector<std::vector<double>> output)
{
	joint_msg.joint0 = calc_vel_output(output[0][0]);
	joint_msg.joint1 = calc_vel_output(output[0][1]);
	joint_msg.joint2 = calc_vel_output(output[0][2]);

	joint_msg.joint3 = calc_vel_output(output[0][3]);
	joint_msg.joint4 = calc_vel_output(output[0][4]);
	joint_msg.joint5 = calc_vel_output(output[0][5]);

	jointPositions.publish(joint_msg);
}

void Exo_interface::setOutputMatrixVel(double output[6])
{
	joint_msg.joint0 = calc_vel_output(output[0]);
	joint_msg.joint1 = calc_vel_output(output[1]);
	joint_msg.joint2 = calc_vel_output(output[2]);

	joint_msg.joint3 = calc_vel_output(output[3]);
	joint_msg.joint4 = calc_vel_output(output[4]);
	joint_msg.joint5 = calc_vel_output(output[5]);

	jointPositions.publish(joint_msg);
}

void Exo_interface::publishNeuralNet(std::vector<std::vector<double>> output, std::string identifierTerm, int netnum)
{
	neuralNet.data.clear();

	int height = output.size();
	int width = output[0].size();

	std::vector<double> vec(output.size() * output[0].size());

    neuralNet.layout.dim[0].label = identifierTerm;
    neuralNet.layout.dim[0].size = height;
    neuralNet.layout.dim[0].stride = height*width;
    //neuralNet.layout.dim[1].label = netnum;
    neuralNet.layout.dim[1].size = width;
	//std::cout << "netnum" << netnum << std::endl;
    neuralNet.layout.dim[1].stride = netnum;
	
/*
    for (int i=0; i<height; i++)
        for (int j=0; j<width; j++)
            vec[i*width + j] = output[i][j];
    neuralNet.data = vec;

	neuralNets.publish(neuralNet);*/
}

void Exo_interface::clearInput()
{
	jointConfiguration[0] = 0;
	jointConfiguration[1] = 0;
	jointConfiguration[2] = 0;
	jointConfiguration[3] = 0;
	jointConfiguration[4] = 0;
	jointConfiguration[5] = 0;

	hipPosition[0] = 0;
	hipPosition[1] = 0;
	hipPosition[2] = 0;

	hipOrientation[0] = 0;
	hipOrientation[1] = 0;
	hipOrientation[2] = 0;
	hipOrientation[3] = 0;

	angularIMUVelocityArray[0] = 0;
	angularIMUVelocityArray[1] = 0;
	angularIMUVelocityArray[2] = 0;

	soleContactArray[0] = 0;
	soleContactArray[1] = 0;
	soleContactArray[2] = 0;
	soleContactArray[3] = 0;

	solePositionArray[0] = 0;
	solePositionArray[1] = 0;
	solePositionArray[2] = 0;

	solePositionArray[3] = 0;
	solePositionArray[4] = 0;
	solePositionArray[5] = 0;
}


//-----------------------Private functions-----------------------:
void Exo_interface::callback_angle(const continuous_slopes::exo_output &msg)
{
	/*exoJointAngles.data = msg.data;
	for (int i = 0; i < msg.data.size(); i++)
	{
		//std::cout << "test: " << msg.data[i] << std::endl;
		jointConfiguration[i] = msg.data[i];
	}*/
	jointConfiguration[0] = msg.joint0;
	jointConfiguration[1] = msg.joint1;
	jointConfiguration[2] = msg.joint2;
	jointConfiguration[3] = msg.joint3;
	jointConfiguration[4] = msg.joint4;
	jointConfiguration[5] = msg.joint5;

	hipPosition[0] = msg.hipX;
	hipPosition[1] = msg.hipY;
	hipPosition[2] = msg.hipZ;

	hipOrientation[0] = msg.hipRoll;
	hipOrientation[1] = msg.hipPitch;
	hipOrientation[2] = msg.hipYaw;
	hipOrientation[3] = msg.hipDistToFloor;

	angularIMUVelocityArray[0] = msg.angularX;
	angularIMUVelocityArray[1] = msg.angularY;
	angularIMUVelocityArray[2] = msg.angularZ;

	soleContactArray[0] = msg.rightHeel;
	soleContactArray[1] = msg.rightToeY;
	soleContactArray[2] = msg.leftHeel;
	soleContactArray[3] = msg.leftToeY;

	solePositionArray[0] = msg.rightFootX;
	solePositionArray[1] = msg.rightFootY;
	solePositionArray[2] = msg.rightFootZ;

	solePositionArray[3] = msg.leftFootX;
	solePositionArray[4] = msg.leftFootY;
	solePositionArray[5] = msg.leftFootZ;

	toeForceArray[0] = msg.rightToeX;
	toeForceArray[1] = msg.rightToeY;
	toeForceArray[2] = msg.rightToeZ;

	toeForceArray[3] = msg.leftToeX;
	toeForceArray[4] = msg.leftToeY;
	toeForceArray[5] = msg.leftToeZ;
	
	soleProximitySensor[0] = msg.rightProximityDistance;
	soleProximitySensor[1] = msg.leftProximityDistance;

	soleDistanceToFloor[0] = msg.rightFootDist;
	soleDistanceToFloor[1] = msg.leftFootDist;


}

void Exo_interface::callback_simState(const std_msgs::Int32 &msg)
{

	simStateValue = msg.data;
	//std::cout << "SimState: " << simStateValue << std::endl;
}

void Exo_interface::callback_SimTime(const std_msgs::Float32 &msg)
{
	simulationTime = msg.data;
	//std::cout << "SimTime: " << simulationTime << std::endl;
}



