#ifndef EXO_INTERFACE_H_

#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float32MultiArray.h"

#include "continuous_slopes/exo_joint_conf.h"
#include "continuous_slopes/exo_output.h"

#include <string>
#include <array>
#include <iostream>
#include <math.h>
#include <vector>

#define PI 3.14159265

class Exo_interface
{
	public:

		Exo_interface(ros::NodeHandle* nh);
		
		void setJointPosition(double sendArray[6]);

		// ----- Simulation functions ----- //
		void startSimulation();
		void stopSimulation();

		int getSimState();
		double getSimTime();

		int checkStopCondition(int timeOutVal);
		bool checkStep();

		// ----- Get functions ----- //
		double getHipRollAngle();
		double getHipPitchAngle();
		double getHipYawAngle();
		double getHipDistance();
		double getHipXCoord();
		double getHipYCoord();
		double getHipZCoord();

		double getRightHipAngle();
		double getRightKneeAngle();
		double getRightAncleAngle();

		double getLeftHipAngle();
		double getLeftKneeAngle();
		double getLeftAncleAngle();
		
		double getRightHeelForce();
		double getRightToeForce();

		double getLeftHeelForce();
		double getLeftToeForce();

		double getRightSolePositionX();
		double getRightSolePositionY();
		double getRightSolePositionZ();

		double getLeftSolePositionX();
		double getLeftSolePositionY();
		double getLeftSolePositionZ();

		double getCenterOfMassX();
		double getCenterOfMassY();
		double getCenterOfMassZ();
		
		double getRightFrontLegX();
		double getRightFrontLegY();

		double getLeftFrontLegX();
		double getLeftFrontLegY();

		double getRightFootProximityDistance();
		double getLeftFootProximityDistance();

		// ----- Calculation functions ----- //
		double calc_val_input_Right_Hip();
		double calc_val_input_Right_Knee();
		double calc_val_input_Right_Ancle();

		double calc_val_input_Left_Hip();
		double calc_val_input_Left_Knee();
		double calc_val_input_Left_Ancle();

		double calc_val_input_Left_Heel_Sensor();
		double calc_val_input_Left_Toe_Sensor();

		double calc_val_input_Right_Heel_Sensor();
		double calc_val_input_Right_Toe_Sensor();

		bool checkRightFootOnFloor();
		bool checkLeftFootOnFloor();
		
		bool calc_val_Right_ToeX();
		bool calc_val_Left_ToeX();
		
		double calc_val_input_HipRoll();
		double calc_val_input_HipPitch();
		double calc_val_input_HipYaw();

		double calc_val_input_HipX();
		double calc_val_input_HipY();
		double calc_val_input_HipZ();

		double calc_val_input_HipAngularVelocity_phi();
		double calc_val_input_HipAngularVelocity_theta();
		double calc_val_input_HipAngularVelocity_psi();

		double calc_cal_input_right_proximity_sensor();
		double calc_cal_input_left_proximity_sensor();

		double calc_val_output_Hip(double input);
		double calc_val_output_Knee(double input);
		double calc_val_output_Ancle(double input);

		double calc_vel_output(double input);

		int checkCenterOfMass();

		// ----- Input / Output matrix functions ----- //
		std::vector<std::vector<double>> getInputMatrix();
		void setOutputMatrix(std::vector<std::vector<double>> output);
		void setOutputMatrix(double output[6]);

		void setOutputMatrixVel(std::vector<std::vector<double>> output);
		void setOutputMatrixVel(double output[6]);

		void clearInput();

		// ----- Neural network functions ----- //
		void publishNeuralNet(std::vector<std::vector<double>> output, std::string identifierTerm, int netnum);	

		
	private:
	    ros::NodeHandle nh_;
	    ros::Subscriber jointAngles;
        ros::Subscriber hipOrientationSub;
		ros::Subscriber hipPositionSub;
	    ros::Subscriber simState;
		ros::Subscriber centerOfMass;
		ros::Subscriber solePositions;
		ros::Subscriber simTime;
		ros::Subscriber angularIMUVelocity;
	    ros::Publisher jointPositions;
	    ros::Publisher startSim;
	    ros::Publisher stopSim;

		ros::Publisher neuralNets;

	    bool waitingForSimState = false;

		double jointConfiguration[10] = {};
		double hipOrientation[4] = {};
		double hipPosition[3] = {};
		double centerOfMassArray[3] = {};
		double solePositionArray[6] = {};
		double angularIMUVelocityArray[3] = {};
		double soleContactArray[4] = {};
		double soleDistanceToFloor[2] = {};
		double toeForceArray[6] ={};
		double soleProximitySensor[2] = {};

		bool goodStep = true;
		bool stepStart = true;
		bool rightFootRaised = false;
		bool leftFootRaised = false;
		bool goodRightStep = false;
		bool goodLeftStep = false;

		double simulationTime = 0;

	    std_msgs::Bool reset_msg_true;
		std_msgs::Bool reset_msg_false;
		//std_msgs::Float32MultiArray joint_msg;

		continuous_slopes::exo_joint_conf joint_msg;

		std_msgs::Float32MultiArray neuralNet;

	    int simStateValue = 5;
		
		double Cx = 0;
		double Cy = 0;
		double Cz = 0;

		double  Sx = 0;
		double  Sy = 0;
		double  Sz = 0;

		double  HipX = 0;
		double  HipY = 0;
		double  HipZ = 0;

		double LegX = 0;
		double LegY = 0;
		double LegZ = 0;

		double Bx = 1.004;
		double By = 0.304;
		double Bz = -1.018;

		double x1 = 0;
		double y1 = 0;

		double x2 = 0;
		double y2 = 0;

		double x3 = 0;
		double y3 = 0;

		double x, y = 0;

		double earlierVal = 10000;
		double oldSimTime = 0;

		bool feetReset = false;
		// Exo neural control

		std::string INPUT_TRANSFER_FUNCTION = "straight";

		double upper_limit_input = 0;
		double lower_limit_input = 0;
		double transfer_function_extent = 0;

		double Hip_slope = 0;
		double Knee_slope = 0;
		double Ancle_slope = 0;

		double HipRollPitchYaw_slope = 0;

		double Hip_b_val = 0;
		double Knee_b_val = 0;
		double Ancle_b_val = 0;

		double HipRollPitchYaw_b_val = 0;

		// Joint limits:

		// Hip (135º)	105º (flex.) 30º (ext.)
		// The extend direction is calculated as a negative

		double Hip_ext = (-105 * PI / 180);
		double Hip_flex = (30  * PI / 180);

		// Knee (110º)  105º (flex.)  5º (ext.)

		double Knee_flex = (-105 * PI / 180);
		double Knee_ext = (0   * PI / 180);

		// Ankle (60º)   30º (flex.) 30º (ext.)

		double Ancle_flex = (-30 * PI / 180);
		double Ancle_ext = (30 * PI / 180);

		double HipLow = -PI;
		double HipHigh = PI;

	    void initialize_Float32_publisher(std::string topicName);
	    void initialize_angle_subscriber(std::string topicName);
	    void initialize_array_subscriber(std::string topicName);
		
		void callback_angle(const continuous_slopes::exo_output& msg);
	    void callback_simState(const std_msgs::Int32& msg);
		void callback_SimTime(const std_msgs::Float32& msg);

		double outputArray[1][6];
		int dimensionX = 1;
		int dimensionY = 12;

		std::vector<std::vector<double>> inputMatrix;
		std::vector<std::vector<double>> outputMatrix;

		bool stopSemaphore = false;

		double highestPhi = 0;
		double highestTheta = 0;
		double highestPsi = 0;
};

#endif