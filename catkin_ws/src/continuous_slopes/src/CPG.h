#ifndef CPG_H_

#include <stdlib.h>				// Use srand and rand
#include <iostream>
#include <fstream>
#include <math.h>
#include <vector>

//#define PI (355/113)

class CPG
{
	public:
		CPG();
		std::vector<float> get_CPG_Output();

	private:		
		float h1, h2 = 0;
		float h1_old, h2_old = 0;
		float h1_output, h2_output = 0;
		float w11 = 1.5;
		float w22 = 1.5;
		float w12 = -0.4;
		float w21 = 0.4;
		float bias1 = 0.01;
		float bias2 = 0.01;
		std::vector<float> output;
};
		/*

		float calcSineWaveOutput(float time)
		{
			return (Amplitude * sin(2 * PI/180 * frequency * time));	
		}

		float error = 0;
		float h2_old = 0;
		float h0 = 1;
		float h0Ref = 1, h1Ref = 0;
		float h0_old_Ref = 0, h1_old_Ref = 1;
		float h0Ref_output = 0;
		
		
		std::ofstream dataFile;

		// Slow learner
		float As = 0.9; //Retention factor
		float Bs = 0.4; //Learning rate
		float Cs = 0.005;
		float xs = 0; // Output from learner slow learner
		float xs_old = 0;

		// Fast learner
		float Af = 0.6; 
		float Bf = 0.8;
		float Cf = 0.015;
		float xf = 0;
		float xf_old = 0;
		float xn = 0;

		// Errors for the intregation term
		float errorMinus1 = 0;
		float errorMinus2 = 0;
		float errorMinus3 = 0;
		float errorMinus4 = 0;
		float errorMinus5 = 0;

		float Amplitude = 1;
		float frequency = 10;

		bool up = true;			// Semaphor for the oscilation generator
		float alpha = 1;		// Constants for the learning of neurons
		float mu = 0.1; 		//learning rate
		float phi = 0;
		float phiRef = PI/4.82;

		float weights[3][3] = {{   0, 1, 0.5} , 
							{  -1, 0,   0},
							{ 0.5, 0,   0}};

		float weightsRef[2][2] = {{cos(phiRef), sin(phiRef)} ,
								  {-sin(phiRef), cos(phiRef)}};
}


	/*
		if(up == true)
		{
			value++;
			if (value == 40)
			{
				up = false;
			}
		}
		else
		{
			value--;
			if(value == -40)
			{
				up = true;
			}
		}

		// For fast and slow learner
		
	/*	h0_old = h0;
		h1_old = h1;

		h0 = (h0_old * weights[0][0]) + (h1_old * weights[1][0]);
		h1 = (h1_old * weights[1][1]) + (h0_old * weights[0][1]);

		h0Ref = calcSineWaveOutput(count);
		ROS_INFO("calcSineWaveOutput %f", calcSineWaveOutput(count));	
		ROS_INFO("ref %f", h0Ref);	
		
		error = h0Ref - h0;
		
		xf = Af * xf_old + Bf * error + Cf * (errorMinus1 + errorMinus2 + errorMinus3 + errorMinus4 + errorMinus5);
		xs = As * xs_old + Bs * error + Cs * (errorMinus1 + errorMinus2 + errorMinus3 + errorMinus4 + errorMinus5);

		errorMinus5 = errorMinus4;
		errorMinus4 = errorMinus3;
		errorMinus3 = errorMinus2;
		errorMinus2 = errorMinus1;
		errorMinus1 = error;

		xn = xs + xf;

		xs_old = xs;
		xf_old = xf;

		phi = xn;

		weights[0][0] = cos(phi);
		weights[0][1] = sin(phi);
		weights[1][0] = -sin(phi);
		weights[1][1] = cos(phi);

		h0_output = h0 / (1 + abs(h0));
		h1_output = h1 / (1 + abs(h1));
		h0Ref_output = h0Ref / (1 + abs(h0Ref));

		dataFile << h0 << "," << h1 << "," << h0Ref << "," << error << "\n";

		ROS_INFO("H0: %f, H1 %f, ref %f, phi %f", h0, h1, h0Ref, phi);
	*/

#endif