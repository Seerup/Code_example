#include "LinearAlgebra.h"

LinearAlgebra::LinearAlgebra()
{
    
}

// Multiply two matrices
vector<vector<double>> LinearAlgebra::matrixMultiplication(vector<vector<double>> matrixA, vector<vector<double>> matrixB)
{
    int matrixA_rows = matrixA.size();
	int matrixA_cols = matrixA[0].size();
	int matrixB_rows = matrixB.size(); //sizeof(matrixB)/sizeof(matrixB[0]);
	int matrixB_cols = matrixB[0].size(); //sizeof(matrixB[0])/sizeof(matrixB[0][0]);

	vector<double> initColumnResult(matrixB_cols, 0);

    vector<vector<double>> result(matrixA_rows, initColumnResult); // Makes the amount of rows, the initVector

	//cout << "result dimensions: " << result_rows << " x " << result_cols << endl;
	
	if(matrixA_cols != matrixB_rows)
	{
		std::cout << "Multiplication is not possible, mismatch of dimensions" << endl;
		cout << "matrixA dimensions: " << matrixA_rows << " x " << matrixA_cols << endl;
		cout << "matrixB dimensions: " << matrixB_rows << " x " << matrixB_cols << endl;
	}
	else
	{
		//std::cout << "Multiplication possible " << endl;
		for(int i = 0; i < matrixA_rows; i++)
		{
			for(int j = 0; j < matrixB_cols; j++)
			{
				result[i][j] = 0;
				for(int k = 0; k < matrixA_cols; k++)
				{
					result[i][j] += matrixA[i][k] * matrixB[k][j];
				}
				//cout << result[i][j] << endl;
			}
		}	
	}
	return result;
}

void LinearAlgebra::printMatrix(vector<vector<double>> aVector)
{
    for(int i = 0; i < aVector.size(); i++)
    {
        cout << "{" ;
        for(int j = 0; j < aVector[0].size(); j++)
        {
            cout << aVector[i][j];
            if(j != aVector[0].size()-1)
                cout << ", ";
        }
        cout << "}" << endl;
    }
    cout << endl;
}


