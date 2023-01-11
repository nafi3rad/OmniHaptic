/*****************************************************************************

Copyright (c) 2004 SensAble Technologies, Inc. All rights reserved.

OpenHaptics(TM) toolkit. The material embodied in this software and use of
this software is subject to the terms and conditions of the clickthrough
Development License Agreement.

For questions, comments or bug reports, go to forums at: 
    http://dsc.sensable.com

Module Name:

  CommandJointTorque.cpp

Description:
  This example demonstrates commanding joint torques to the Phantom device. 
*******************************************************************************/
#ifdef  _WIN64
#pragma warning (disable:4996)
#endif

#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <fstream>
#include <iostream>
#include <queue>
#include <cstdlib>
#include <nlopt.h>
//#include <stdio.h>
//#include <iostream>
#include <tchar.h>
#include <math.h>
//#include <stdlib.h>
using namespace std;


#if defined(WIN32)
# include <windows.h>
# include <conio.h>
#else
#include <time.h>
# include "conio.h"
# include <string.h>
#define FALSE 0
#define TRUE 1
#endif

#include <HD/hd.h>
#include <HDU/hduError.h>
#include <HDU/hduVector.h>
//#include <HDU/hduMatrix.h>
//#include <Eigen/Dense>
//#include <Eigen/QR>
//using namespace Eigen;

static const hduVector3Dd maxGimbalTorque(188.0,188.0,48.0); //mNm
static const hduVector3Dd nominalBaseTorque(400.0,350.0,200.0); //mNm
static bool TorqueMode = true;

HDCallbackCode HDCALLBACK jointTorqueCallback(void *data);// declare functions that you wanna define later but u used them before that
HDSchedulerHandle hGravityWell = HD_INVALID_HANDLE;

void mainLoop(void);
bool initDemo(void);


//—————— NLopt optimizer code
typedef struct {
	double a00, a01, a10, a11, b0, b1;
} my_constraint_data;

typedef struct {
	double Fe[2];
} my_func_data;

double myfunc(unsigned n, const double *x, double *grad, void *data)
{
	my_func_data *d = (my_func_data *)data;
	if (grad) {
		grad[0] = -2.0 * x[0]; // TODO? ronde objective func be ronde x0
		grad[1] = -2.0 * x[1]; // TODO? ronde objective func be ronde x1
	}
	double r = d->Fe[0] * d->Fe[0] + d->Fe[1] * d->Fe[1] - x[0] * x[0] - x[1] * x[1];
	return r * r;
}

double myconstraint1(unsigned n, const double *x, double *grad, void *data)
{
	my_constraint_data *d = (my_constraint_data *)data;
	if (grad) {
		grad[0] = d->a00; // ronde constraint be ronde x0
		grad[1] = d->a01; // ronde constraint be ronde x1
	}
	double r = d->a00*x[0] + d->a01*x[1] - d->b0;
	return r; // constraint is assumed to be r <= 0
}

double myconstraint2(unsigned n, const double *x, double *grad, void *data)
{
	my_constraint_data *d = (my_constraint_data *)data;
	if (grad) {
		grad[0] = d->a10; // ronde constraint be ronde x0
		grad[1] = d->a11; // ronde constraint be ronde x1
	}
	double r = d->a10*x[0] + d->a11*x[1] - d->b1;
	return r; // constraint is assumed to be r <= 0
}

void solve() {
	double lb[2] = { -1.0e9, 0 };
	nlopt_opt opt;
	opt = nlopt_create(NLOPT_LD_MMA, 2);
	nlopt_set_lower_bounds(opt, lb);
	nlopt_set_min_objective(opt, myfunc, NULL);
	my_constraint_data data;
	nlopt_add_inequality_constraint(opt, myconstraint1, &data, 1e-8);
	nlopt_add_inequality_constraint(opt, myconstraint2, &data, 1e-8);

}

//—————


void PrintHelp()
{
    static const char help[] = {\
"CommandJointTorque Help\n\
---\n\
T: Command Base & Gimbal Torques \n\
   Torsional Springs at each Joint\n\
   Torsion Spring Constant at Joints = 1000 mN.m/radian\n\
   Torsion Spring Constant at Gimbals = 500 mN.m/radian\n\
   Max Base Torque Commanded by this Demo = {200.0,350.0,200.0}mNm\n\
   Max Gimbal Torque Commanded by this Demo = {188.0,188.0,48.0}mNm\n\
F: Command Base force & Gimbal Torque\n\
   Torsional Springs at Gimbals & a Extension Spring at Position(0,0,0)\n\
   Extension Spring Constant at (0,0,0) = 0.075 N/mm\n\
   Torsion Spring Constant at Gimbals = 500 mN.m/radian\n\
P: Prints device state\n\
C: Continuously prints device state\n\
H: Prints help menu\n\
Q: Quits the program\n\
---"};
    
    printf("\n%s\n", help); //print string(s) help %f: float
}

class CMatrix
{
private:
	int m_rows;
	int m_cols;
	char m_name[128];
	CMatrix();
public:
	double **m_pData;
	CMatrix(const char *name, int rows, int cols) :
		m_rows(rows), m_cols(cols)
	{
		strcpy_s(m_name, name);
		m_pData = new double*[m_rows];
		for (int i = 0; i < m_rows; i++)
			m_pData[i] = new double[m_cols];
		for (int i = 0; i < m_rows; i++)
		{
			for (int j = 0; j < m_cols; j++)
			{
				m_pData[i][j] = 0.0;
			}
		}
	}
	CMatrix(const CMatrix &other)
	{
		strcpy_s(m_name, other.m_name);
		m_rows = other.m_rows;
		m_cols = other.m_cols;
		m_pData = new double*[m_rows];
		for (int i = 0; i < m_rows; i++)
			m_pData[i] = new double[m_cols];
		for (int i = 0; i < m_rows; i++)
		{
			for (int j = 0; j < m_cols; j++)
			{
				m_pData[i][j] = other.m_pData[i][j];
			}
		}
	}
	~CMatrix()
	{
		for (int i = 0; i < m_rows; i++)
			delete[] m_pData[i];
		delete[] m_pData;
		m_rows = m_cols = 0;
	}
	void SetName(const char *name)
	{
		strcpy_s(m_name, name);
	}
	const char* GetName() const
	{
		return m_name;
	}
	void GetInput()
	{
		std::cin >> *this;
	}
	void FillSimulatedInput()
	{
		static int factor1 = 1, factor2 = 2;
		std::cout << "\n\nEnter Input For Matrix : " << m_name << " Rows: "
			<< m_rows << " Cols: " << m_cols << "\n";
		for (int i = 0; i < m_rows; i++)
		{
			for (int j = 0; j < m_cols; j++)
			{
				std::cout << "Input For Row: " << i + 1 << " Col: " << j
					+ 1 << " = ";
				int data = ((i + 1) * factor1) + (j + 1) * factor2;
				m_pData[i][j] = data / 10.2;
				std::cout << m_pData[i][j] << "\n";
				factor1 += (rand() % 4);
				factor2 += (rand() % 3);
			}
			std::cout << "\n";
		}
		std::cout << "\n";
	}
	double Determinant()
	{
		double det = 0;
		double **pd = m_pData;
		switch (m_rows)
		{
		case 2:
		{
			det = pd[0][0] * pd[1][1] - pd[0][1] * pd[1][0];
			return det;
		}
		break;
		case 3:
		{
			/***
			a b c
			d e f
			g h i

			a b c a b c
			d e f d e f
			g h i g h i

			// det (A) = aei + bfg + cdh - afh - bdi - ceg.
			***/
			double a = pd[0][0];
			double b = pd[0][1];
			double c = pd[0][2];
			double d = pd[1][0];
			double e = pd[1][1];
			double f = pd[1][2];
			double g = pd[2][0];
			double h = pd[2][1];
			double i = pd[2][2];
			double det = (a * e * i + b * f * g + c * d * h);
			det = det - a * f * h;
			det = det - b * d * i;
			det = det - c * e * g;
			return det;
		}
		break;
		case 4:
		{
			CMatrix *temp[4];
			for (int i = 0; i < 4; i++)
				temp[i] = new CMatrix("", 3, 3);
			for (int k = 0; k < 4; k++)
			{
				for (int i = 1; i < 4; i++)
				{
					int j1 = 0;
					for (int j = 0; j < 4; j++)
					{
						if (k == j)
							continue;
						temp[k]->m_pData[i - 1][j1++]
							= this->m_pData[i][j];
					}
				}
			}
			double det = this->m_pData[0][0] * temp[0]->Determinant()
				- this->m_pData[0][1] * temp[1]->Determinant()
				+ this->m_pData[0][2] * temp[2]->Determinant()
				- this->m_pData[0][3] * temp[3]->Determinant();
			return det;
		}
		break;
		case 5:
		{
			CMatrix *temp[5];
			for (int i = 0; i < 5; i++)
				temp[i] = new CMatrix("", 4, 4);
			for (int k = 0; k < 5; k++)
			{
				for (int i = 1; i < 5; i++)
				{
					int j1 = 0;
					for (int j = 0; j < 5; j++)
					{
						if (k == j)
							continue;
						temp[k]->m_pData[i - 1][j1++]
							= this->m_pData[i][j];
					}
				}
			}
			double det = this->m_pData[0][0] * temp[0]->Determinant()
				- this->m_pData[0][1] * temp[1]->Determinant()
				+ this->m_pData[0][2] * temp[2]->Determinant()
				- this->m_pData[0][3] * temp[3]->Determinant()
				+ this->m_pData[0][4] * temp[4]->Determinant();
			return det;
		}
		case 6:
		case 7:
		case 8:
		case 9:
		case 10:
		case 11:
		case 12:
		default:
		{
			int DIM = m_rows;
			CMatrix **temp = new CMatrix*[DIM];
			for (int i = 0; i < DIM; i++)
				temp[i] = new CMatrix("", DIM - 1, DIM - 1);
			for (int k = 0; k < DIM; k++)
			{
				for (int i = 1; i < DIM; i++)
				{
					int j1 = 0;
					for (int j = 0; j < DIM; j++)
					{
						if (k == j)
							continue;
						temp[k]->m_pData[i - 1][j1++]
							= this->m_pData[i][j];
					}
				}
			}
			double det = 0;
			for (int k = 0; k < DIM; k++)
			{
				if ((k % 2) == 0)
					det = det + (this->m_pData[0][k]
					* temp[k]->Determinant());
				else
					det = det - (this->m_pData[0][k]
					* temp[k]->Determinant());
			}
			for (int i = 0; i < DIM; i++)
				delete temp[i];
			delete[] temp;
			return det;
		}
		break;
		}
	}
	CMatrix& operator =(const CMatrix &other)
	{
		if (this->m_rows != other.m_rows || this->m_cols != other.m_cols)
		{
			std::cout
				<< "WARNING: Assignment is taking place with by changing the number of rows and columns of the matrix";
		}
		for (int i = 0; i < m_rows; i++)
			delete[] m_pData[i];
		delete[] m_pData;
		m_rows = m_cols = 0;
		strcpy_s(m_name, other.m_name);
		m_rows = other.m_rows;
		m_cols = other.m_cols;
		m_pData = new double*[m_rows];
		for (int i = 0; i < m_rows; i++)
			m_pData[i] = new double[m_cols];
		for (int i = 0; i < m_rows; i++)
		{
			for (int j = 0; j < m_cols; j++)
			{
				m_pData[i][j] = other.m_pData[i][j];
			}
		}
		return *this;
	}
	CMatrix CoFactor()
	{
		CMatrix cofactor("COF", m_rows, m_cols);
		if (m_rows != m_cols)
			return cofactor;
		if (m_rows < 2)
			return cofactor;
		else if (m_rows == 2)
		{
			cofactor.m_pData[0][0] = m_pData[1][1];
			cofactor.m_pData[0][1] = -m_pData[1][0];
			cofactor.m_pData[1][0] = -m_pData[0][1];
			cofactor.m_pData[1][1] = m_pData[0][0];
			return cofactor;
		}
		else if (m_rows >= 3)
		{
			int DIM = m_rows;
			CMatrix ***temp = new CMatrix**[DIM];
			for (int i = 0; i < DIM; i++)
				temp[i] = new CMatrix*[DIM];
			for (int i = 0; i < DIM; i++)
				for (int j = 0; j < DIM; j++)
					temp[i][j] = new CMatrix("", DIM - 1, DIM - 1);
			for (int k1 = 0; k1 < DIM; k1++)
			{
				for (int k2 = 0; k2 < DIM; k2++)
				{
					int i1 = 0;
					for (int i = 0; i < DIM; i++)
					{
						int j1 = 0;
						for (int j = 0; j < DIM; j++)
						{
							if (k1 == i || k2 == j)
								continue;
							temp[k1][k2]->m_pData[i1][j1++]
								= this->m_pData[i][j];
						}
						if (k1 != i)
							i1++;
					}
				}
			}
			bool flagPositive = true;
			for (int k1 = 0; k1 < DIM; k1++)
			{
				flagPositive = ((k1 % 2) == 0);
				for (int k2 = 0; k2 < DIM; k2++)
				{
					if (flagPositive == true)
					{
						cofactor.m_pData[k1][k2]
							= temp[k1][k2]->Determinant();
						flagPositive = false;
					}
					else
					{
						cofactor.m_pData[k1][k2]
							= -temp[k1][k2]->Determinant();
						flagPositive = true;
					}
				}
			}
			for (int i = 0; i < DIM; i++)
				for (int j = 0; j < DIM; j++)
					delete temp[i][j];
			for (int i = 0; i < DIM; i++)
				delete[] temp[i];
			delete[] temp;
		}
		return cofactor;
	}
	CMatrix Adjoint()
	{
		CMatrix cofactor("COF", m_rows, m_cols);
		CMatrix adj("ADJ", m_rows, m_cols);
		if (m_rows != m_cols)
			return adj;
		cofactor = this->CoFactor();
		// adjoint is transpose of a cofactor of a matrix
		for (int i = 0; i < m_rows; i++)
		{
			for (int j = 0; j < m_cols; j++)
			{
				adj.m_pData[j][i] = cofactor.m_pData[i][j];
			}
		}
		return adj;
	}
	CMatrix Transpose()
	{
		CMatrix trans("TR", m_cols, m_rows);
		for (int i = 0; i < m_rows; i++)
		{
			for (int j = 0; j < m_cols; j++)
			{
				trans.m_pData[j][i] = m_pData[i][j];
			}
		}
		return trans;
	}
	CMatrix Inverse()
	{
		CMatrix cofactor("COF", m_rows, m_cols);
		CMatrix inv("INV", m_rows, m_cols);
		if (m_rows != m_cols)
			return inv;
		// to find out Determinant
		double det = Determinant();
		cofactor = this->CoFactor();
		// inv = transpose of cofactor / Determinant
		for (int i = 0; i < m_rows; i++)
		{
			for (int j = 0; j < m_cols; j++)
			{
				inv.m_pData[j][i] = cofactor.m_pData[i][j] / det;
			}
		}
		return inv;
	}
	CMatrix operator +(const CMatrix &other)
	{
		if (this->m_rows != other.m_rows || this->m_cols != other.m_cols)
		{
			std::cout
				<< "Addition could not take place because number of rows and columns are different between the two matrices";
			return *this;
		}
		CMatrix result("", m_rows, m_cols);
		for (int i = 0; i < m_rows; i++)
		{
			for (int j = 0; j < m_cols; j++)
			{
				result.m_pData[i][j] = this->m_pData[i][j]
					+ other.m_pData[i][j];
			}
		}
		return result;
	}
	CMatrix operator -(const CMatrix &other)
	{
		if (this->m_rows != other.m_rows || this->m_cols != other.m_cols)
		{
			std::cout
				<< "Subtraction could not take place because number of rows and columns are different between the two matrices";
			return *this;
		}
		CMatrix result("", m_rows, m_cols);
		for (int i = 0; i < m_rows; i++)
		{
			for (int j = 0; j < m_cols; j++)
			{
				result.m_pData[i][j] = this->m_pData[i][j]
					- other.m_pData[i][j];
			}
		}
		return result;
	}
	CMatrix operator *(const CMatrix &other)
	{
		if (this->m_cols != other.m_rows)
		{
			std::cout
				<< "Multiplication could not take place because number of columns of 1st Matrix and number of rows in 2nd Matrix are different";
			return *this;
		}
		CMatrix result("", this->m_rows, other.m_cols);
		for (int i = 0; i < this->m_rows; i++)
		{
			for (int j = 0; j < other.m_cols; j++)
			{
				for (int k = 0; k < this->m_cols; k++)
				{
					result.m_pData[i][j] += this->m_pData[i][k]
						* other.m_pData[k][j];
				}
			}
		}
		return result;
	}
	bool operator ==(const CMatrix &other)
	{
		if (this->m_rows != other.m_rows || this->m_cols != other.m_cols)
		{
			std::cout
				<< "Comparision could not take place because number of rows and columns are different between the two matrices";
			return false;
		}
		CMatrix result("", m_rows, m_cols);
		bool bEqual = true;
		for (int i = 0; i < m_rows; i++)
		{
			for (int j = 0; j < m_cols; j++)
			{
				if (this->m_pData[i][j] != other.m_pData[i][j])
					bEqual = false;
			}
		}
		return bEqual;
	}
	friend std::istream& operator >>(std::istream &is, CMatrix &m);
	friend std::ostream& operator <<(std::ostream &os, const CMatrix &m);
};
std::istream& operator >>(std::istream &is, CMatrix &m)
{
	std::cout << "\n\nEnter Input For Matrix : " << m.m_name << " Rows: "
		<< m.m_rows << " Cols: " << m.m_cols << "\n";
	for (int i = 0; i < m.m_rows; i++)
	{
		for (int j = 0; j < m.m_cols; j++)
		{
			std::cout << "Input For Row: " << i + 1 << " Col: " << j + 1
				<< " = ";
			is >> m.m_pData[i][j];
		}
		std::cout << "\n";
	}
	std::cout << "\n";
	return is;
}
std::ostream& operator <<(std::ostream &os, const CMatrix &m)
{

	os << "\n\nMatrix : " << m.m_name << " Rows: " << m.m_rows << " Cols: "
		<< m.m_cols << "\n\n";
	for (int i = 0; i < m.m_rows; i++)
	{
		os << " | ";
		for (int j = 0; j < m.m_cols; j++)
		{
			char buf[32];
			double data = m.m_pData[i][j];
			if (m.m_pData[i][j] > -0.00001 && m.m_pData[i][j] < 0.00001)
				data = 0;
			sprintf_s(buf, "%10.2lf ", data);
			os << buf;
		}
		os << "|\n";
	}
	os << "\n\n";
	return os;
}
/* Synchronization structure. */
typedef struct//defining a new type
{
    HDdouble forceValues[3];
    HDdouble jointTorqueValues[3];   
    HDdouble gimbalTorqueValues[3];
	HDdouble positionValues[3];
	HDdouble velocityValues[3];

} DeviceStateStruct;

typedef struct
{
	HDdouble energy;
	HDdouble observedEnergy;
	HDdouble alpha;
	HDdouble oldAlpha;
	hduVector3Dd oldVelocity;
	HDint counter;
	HDdouble oldP;
	HDdouble oldkhat;
	HDdouble Emaxp;
	HDdouble oldx;
	hduVector3Dd oldFpc;
	HDdouble oldFc;
	HDdouble doldFc;
	HDdouble olddx;
	queue<hduVector3Dd> force_history;
} EnergyStruct;

ofstream csvEnergy;
ofstream csvTorque;
ofstream csvPosition;
ofstream csvVelocity;
ofstream csvSampling;
ofstream csvAlpha;
ofstream csvKhat;
ofstream csvControlForce;
/*****************************************************************************
 Callback that retrieves state.
*****************************************************************************/
HDCallbackCode HDCALLBACK GetDeviceStateCallback(void *pUserData)
{
    DeviceStateStruct *pState = (DeviceStateStruct *) pUserData;//cast type:change the type    float f = (float) 2;

    hdGetDoublev(HD_CURRENT_FORCE, pState->forceValues); // a->b is exactly (*a).b
    hdGetDoublev(HD_CURRENT_JOINT_TORQUE, pState->jointTorqueValues);
    hdGetDoublev(HD_CURRENT_GIMBAL_TORQUE, pState->gimbalTorqueValues);
	hdGetDoublev(HD_CURRENT_POSITION, pState->positionValues);
	hdGetDoublev(HD_CURRENT_VELOCITY, pState->velocityValues);

    return HD_CALLBACK_DONE;
}

/*****************************************************************************
 Callback that retrieves state.
*****************************************************************************/
void PrintDeviceState(HDboolean bContinuous)
{
    int i;
    DeviceStateStruct state;
	//ofstream csvTorque;
	//ofstream csvPosition;
	//ofstream csvVelocity;
	//csvTorque.open("torque.csv");
	//csvPosition.open("position.csv");
	//csvVelocity.open("Velocity.csv");
    memset(&state, 0, sizeof(DeviceStateStruct));//making all the elements equal to zero, some other values maybe stored in this address

    do//do while structure, runs at least once the DO part
    {
        hdScheduleSynchronous(GetDeviceStateCallback, &state,
            HD_DEFAULT_SCHEDULER_PRIORITY);//sync: wait for this to finish then start the next line

        printf("\n");


        if (TorqueMode)
        {
        //printf("Current Base Torque Values (mNm):");
        for (i = 0; i < 3; i++)// it has 3 elements
        {
            printf("%f,", state.jointTorqueValues[i]);
			//csvTorque << state.jointTorqueValues[i] << ",";
        }
        //printf("\n");

        //printf("Current Gimbal Torque Values (mNm):");
        //for (i = 0; i < 3; i++)
        //{
        //    printf("%f,", state.gimbalTorqueValues[i]);
        //}
        //printf("\n");
		for (i = 0; i < 3; i++)
		{
			printf("%f,", state.positionValues[i]);
			//if (i == 2){
				//csvPosition << state.positionValues[i] << endl;
			//}
			//else
			//{
				//csvPosition << state.positionValues[i] << ",";
			//}
			
		}
		//
		for (i = 0; i < 3; i++)
		{
			printf("%f,", state.velocityValues[i]);
		}
		//
        }
        else
        {
        //printf("Current Base Force Values (N):");
        for (i = 0; i < 3; i++)
        {
            printf("%f,", state.forceValues[i]);
			if (i == 2){
				//csvTorque << state.forceValues[i] << endl;
				cout << state.forceValues[i] << endl;
			}
			else
			{
				//csvTorque << state.forceValues[i] << ",";
				cout << state.forceValues[i] << ",";
			}
        }
		//energy = oldEnergy + 0.001 * state.forceValues[0] * state.velocityValues[0] * -1.0;
		//oldEnergy = energy;
		//csvEnergy << energy << endl;

        //printf("\n");

        //printf("Current Base Torque Values (mNm):");
        //for (i = 0; i < 3; i++)
        //{
            //printf("%f,", state.jointTorqueValues[i]);
        //}
        //printf("\n");

        //printf("Current Gimbal Torque Values (mNm):");
        //for (i = 0; i < 3; i++)
        //{
            //printf("%f,", state.gimbalTorqueValues[i]);
        //}
        //printf("\n");
		for (i = 0; i < 3; i++)
		{
			printf("%f,", state.positionValues[i]);
			if (i == 2){
				//csvPosition << state.positionValues[i] << endl;
				cout << state.positionValues[i] << endl;
			}
			else
			{
				//csvPosition << state.positionValues[i] << ",";
				cout << state.positionValues[i] << ",";
			}
		}
		//
		for (i = 0; i < 3; i++)
		{
			printf("%f,", state.velocityValues[i]);
			if (i == 2){
				//csvVelocity << state.velocityValues[i] << endl;
				cout << state.velocityValues[i] << endl;
			}
			else
			{
				//csvVelocity << state.velocityValues[i] << ",";
				cout << state.velocityValues[i] << ",";
			}
		}
		//
        }

        if (bContinuous)
        {
       		Sleep(1);
        }

    } while (!_kbhit() && bContinuous);
	//csvTorque.close();
	//csvPosition.close();
	//csvVelocity.close();
}

/*******************************************************************************
 Main function.
 Initializes the device, starts the schedule, creates a schedule callback
 to handle gravity well forces, waits for the user to press a button, exits
 the application.
*******************************************************************************/
int main(int argc, char* argv[])
{    
    HDErrorInfo error;
    /* Initialize the device, must be done before attempting to call any hd 
       functions. Passing in HD_DEFAULT_DEVICE causes the default device to be 
       initialized. */
    HHD hHD = hdInitDevice(HD_DEFAULT_DEVICE);
    if (HD_DEVICE_ERROR(error = hdGetError())) 
    {
        hduPrintError(stderr, &error, "Failed to initialize haptic device");
        fprintf(stderr, "\nPress any key to quit.\n");//functional print f(stdout, stderr are two type of output, fprintf
        getch();// get one char from keyboard
        return -1;
    }

    printf("Command Joint Torque Demo!\n");
    printf("Found device model: %s.\n\n", hdGetString(HD_DEVICE_MODEL_TYPE));

    if (!initDemo())
    {
        printf("Demo Initialization failed\n");
        printf("Press any key to exit\n");
        getch();
        
    }
	//ofstream csvEnergy;
	csvEnergy.open("energy.csv");
	csvTorque.open("torque.csv");
	csvPosition.open("position.csv");
	csvVelocity.open("Velocity.csv");
	csvSampling.open("sampling.csv");
	csvAlpha.open("Alpha.csv"); 
	csvKhat.open("khat.csv");
	csvControlForce.open("ControlForce.csv");
	EnergyStruct e;
	e.counter = 0;
	e.energy = 0.0;
	e.observedEnergy = 0.0;
	e.oldAlpha = 0.0;
	e.oldVelocity[0] = 0.0;
	e.alpha = 0.0;
	e.oldkhat = 0.0;
	e.oldP = 0.1;
	e.Emaxp = 0.0;
	e.oldFc = 0.0;
	e.oldFpc[0] = 0.0;
	e.oldFpc[1] = 0.0;
	e.oldFpc[2] = 0.0;
	e.oldx = 0.0;
	HDdouble maxStif;
	HDdouble maxDamp;


    /* Schedule the main callback that will render forces to the device. */
    hGravityWell = hdScheduleAsynchronous(
        jointTorqueCallback, &e, 
        HD_MAX_SCHEDULER_PRIORITY);
	
    hdEnable(HD_FORCE_OUTPUT);
	hdSetSchedulerRate(1000);
    hdStartScheduler();
	//csvEnergy << e.energy << endl;
    /* Check for errors and abort if so. */
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        hduPrintError(stderr, &error, "Failed to start scheduler");
        fprintf(stderr, "\nPress any key to quit.\n");
        return -1;
    }

    PrintHelp();

    /* Start the main application loop */
    mainLoop();
	hdGetDoublev(HD_NOMINAL_MAX_STIFFNESS, &maxStif);
	cout << maxStif << endl;
	hdGetDoublev(HD_NOMINAL_MAX_DAMPING, &maxDamp);
	cout << maxDamp << endl;
    /* For cleanup, unschedule callback and stop the scheduler. */
    hdStopScheduler();
    hdUnschedule(hGravityWell);
	csvEnergy.close();
	csvTorque.close();
	csvPosition.close();
	csvVelocity.close();
	csvSampling.close();
	csvAlpha.close();
	csvKhat.close();
	csvControlForce.close();
    /* Disable the device. */
    hdDisableDevice(hHD);

    return 0;
}
/******************************************************************************
 The main loop of execution.  Detects and interprets keypresses.  Monitors and 
 initiates error recovery if necessary.
******************************************************************************/
void mainLoop()
{
    int keypress;
    int nMotorIndex = 0;

    while (TRUE)
    {
        if (_kbhit())
        {
            keypress = getch();
            keypress = toupper(keypress);
            
            switch (keypress)//check yhe val
            {
                case 'F': TorqueMode = false; break;
                case 'T': TorqueMode = true; break;
                case 'P': PrintDeviceState(FALSE); break;
                case 'C': PrintDeviceState(TRUE); break;
                case 'H': PrintHelp(); break;
                case 'Q': return;
                default: PrintHelp(); break;//none of them
            }
        }

        /* Check if the scheduled callback has stopped running */
        if (!hdWaitForCompletion(hGravityWell, HD_WAIT_CHECK_STATUS))
        {
            fprintf(stderr, "\nThe main scheduler callback has exited\n");
            fprintf(stderr, "\nPress any key to quit.\n");
            getch();
            return;
        }
    }
}

//void multiply(HDdouble *A, HDdouble *B, HDdouble *result, int n, int m, int k) {
//	for (int i = 0; i < n; i++) {
//		for (int j = 0; j < k; j++) {
//			// Compute result[i * k + j]
//			result[i * k + j] = 0;
//			for (int t = 0; t < m; t++)
//				result[i * k + j] += A[i * m + t] * B[t * k + j];
//		}
//	}
//}
//
//
//void multiply(HDdouble *A, HDdouble *result, HDdouble x, int n, int m) {
//	for (int i = 0; i < n; i++) {
//		for (int j = 0; j < m; j++) {
//			result[i * m + j] = A[i * m + j] * x;
//		}
//	}
//}
//
//void transpose(HDdouble *A, HDdouble *result, int n, int m) {
//	for (int i = 0; i < n; i++) {
//		for (int j = 0; j < m; j++) {
//			result[j * n + i] = A[i * m + j];
//		}
//	}
//}

/*******************************************************************************
 Servo callback.  
 Called every servo loop tick.  Simulates a gravity well, which sucks the device 
 towards its center whenever the device is within a certain range.
*******************************************************************************/
HDCallbackCode HDCALLBACK jointTorqueCallback(void *data)
{
	EnergyStruct* ep = (EnergyStruct*) data;
	const HDdouble kStiffnessx = 0.1;// 0.1;// 0.075; /* N/mm */positive k is passive
	const HDdouble kStiffnessy = 0.1;
	HDdouble khat; //estimation of the k
	const HDdouble constkhat = 0.08;// ; //estimation of the k
	const HDdouble bDampingx = -0.002;// -0.003;//-0.002 75;positive b is passive
	const HDdouble bDampingy = -0.002;// -0.003;//-0.002 75;positive b is passive
    const HDdouble kStylusTorqueConstant = 500; /* torque spring constant (mN.m/radian)*/
    const HDdouble kJointTorqueConstant = 12000; /* torque spring constant (mN.m/radian)*/
 
    const HDdouble kForceInfluence = 50.0; /* mm */
	const HDdouble kTorqueInfluence = 0.0;//3.14; /* radians */
	const HDdouble SampleTime = 0.001;

    /* This is the position of the gravity well in cartesian
       (i.e. x,y,z) space. */
    static const hduVector3Dd wellPos(0,0,0);
    static const hduVector3Dd stylusVirtualFulcrum(0.0, 0.0, 0.0); // In radians
    static const hduVector3Dd jointVirtualFulcrum(0.0, 0.0, 0.0); // In radians
    
    HDErrorInfo error;
    hduVector3Dd position;
	hduVector3Dd velocity;
	hduVector3Dd vp;

	//double energy = 0;
	HDdouble sampleTime;
	HDdouble ePassive;
	HDdouble oldEnergy;
	HDint currentRate;
	hduVector3Dd force;
	hduVector3Dd controlForce;
	hduVector3Dd Fpc;
	hduVector3Dd Forcedx;
	hduVector3Dd sensorForce;
    hduVector3Dd positionTwell;
    hduVector3Dd gimbalAngles;
    hduVector3Dd gimbalTorque;
    hduVector3Dd gimbalAngleOfTwist;
    hduVector3Dd jointAngles;
    hduVector3Dd jointTorque;
    hduVector3Dd jointAngleOfTwist;
    HHD hHD = hdGetCurrentDevice();
	HDdouble l;
	HDdouble p;
	HDdouble z;
	const HDdouble lambda=1.0;

	const HDdouble irr = 0.1;
	HDdouble printEnergy;

    /* Begin haptics frame.  ( In general, all state-related haptics calls
       should be made within a frame. ) */
    hdBeginFrame(hHD);

    /* Get the current position of the device. */
    hdGetDoublev(HD_CURRENT_POSITION, position);
    hdGetDoublev(HD_CURRENT_GIMBAL_ANGLES,gimbalAngles );
    hdGetDoublev(HD_CURRENT_JOINT_ANGLES,jointAngles );
	hdGetDoublev(HD_CURRENT_VELOCITY, velocity);
	//hdGetDoublev(HD_CURRENT_FORCE, sensorForce);
	hdGetIntegerv(HD_INSTANTANEOUS_UPDATE_RATE, &currentRate);

    memset(force, 0, sizeof(hduVector3Dd));
	memset(controlForce, 0, sizeof(hduVector3Dd));
	memset(Forcedx, 0, sizeof(hduVector3Dd));
	/*filter velocity*/
	velocity = irr * velocity + (1 - irr) * (ep->oldVelocity);

    /* >  positionTwell = wellPos-position  < 
       Create a vector from the device position towards the gravity 
       well's center. */
    hduVecSubtract(positionTwell, wellPos, position);
    
    hduVecSubtract(gimbalAngleOfTwist, stylusVirtualFulcrum, gimbalAngles);

    hduVecSubtract(jointAngleOfTwist, jointVirtualFulcrum, jointAngles);

    /* If the device position is within some distance of the gravity well's 
       center, apply a spring force towards gravity well's center.  The force
       calculation differs from a traditional gravitational body in that the
       closer the device is to the center, the less force the well exerts;
       the device behaves as if a spring were connected between itself and
       the well's center. */
	if (positionTwell[0]<0.0 || positionTwell[1]<0.0)
    {
        /* >  F = k * x  < y
           F: Force in Newtons (N)
           k: Stiffness of the well (N/mm)
           x: Vector from the device endpoint position to the center 
           of the well. */
        //hduVecScale(force, positionTwell, kStiffness);
		//force[0] = kStiffness*positionTwell[0];
		controlForce[0] = 0.0;
		controlForce[0] = 0.0;
		if (positionTwell[0]<0.0)
		{controlForce[0] = -bDampingx * velocity[0] + kStiffnessx*positionTwell[0] ;}
		if (positionTwell[1]<0.0)
		{controlForce[1] = -bDampingy * velocity[1] + kStiffnessy*positionTwell[1] ;}
		controlForce[2] = 0;

    }
	else{
		controlForce[0] = 0;
		controlForce[1] = 0;
		controlForce[2] = 0;
	}
	/*Adding delay*/ 
	//if the movement be fast, 11 is good enough

	//ep->force_history.push(controlForce);
	//if (ep->force_history.size() == 21){
	//	force = ep->force_history.front();
	//	ep->force_history.pop();
	//}
	
	force = controlForce;


	Forcedx = -controlForce ;
	//cout << force[0] << endl;
	
	//sampleTime = 1.0 / currentRate;//SampleTime;
	sampleTime = 0.001;
	//ep->energy += sampleTime * force[0] * velocity[0] * -1.0;
	

	/*using -velocity instead of deltaPos/T */
	//ePassive = -1*khat*positionTwell[0] * velocity[0];//wrong : it should be summation

	//oldEnergy = ep->observedEnergy+ 
			//ep->oldAlpha * ep->oldVelocity[0] * ep->oldVelocity[0];
	 /*computing observed energy*/
	if (ep->counter>0)
	{
		//oldEnergy = ep->observedEnergy +
		//	ep->oldAlpha * dotProduct(ep->oldVelocity , velocity ) * sampleTime;

		//ep->observedEnergy += -dotProduct(force , velocity) * sampleTime +
		//	ep->oldAlpha * dotProduct(ep->oldVelocity, velocity) * sampleTime;

		/*Optimization*/
		oldEnergy = ep->observedEnergy +
			dotProduct(ep->oldFpc, velocity) * sampleTime;

		ep->observedEnergy += -dotProduct(force, velocity) * sampleTime +
			dotProduct(ep->oldFpc, velocity) * sampleTime;
	}
	else
	{
		oldEnergy = 0.0;

		ep->observedEnergy = -dotProduct(force, velocity) * sampleTime;
	}

	//Emax
	//if (ep->observedEnergy > oldEnergy)
	//{ 
	//	if (ep->observedEnergy > ep->Emaxp)
	//	{
	//		ep->Emaxp = ep->observedEnergy;
	//	}
	//}
	//else
	//{
	//	ep->Emaxp = 0.0001;
	//}
	
	/*RLS*/
	//if (oldEnergy < ep->observedEnergy)
	//{
	//	z = 0.5*positionTwell[0] * positionTwell[0];
	//	l = ep->oldP*z / (lambda + z*ep->oldP*z);
	//	p = (1 / lambda)*(ep->oldP - l*z*ep->oldP);
	//	khat = ep->oldkhat + l*(ep->observedEnergy - z*ep->oldkhat);
	//}
	//else
	//{
	//	khat = ep->oldkhat;
	//}
	////khat = 0.1;
	////updating old variables;
	//ep->oldP = p;
	//ep->oldkhat = khat;

	//ePassive = 0.5*khat*positionTwell[0] * positionTwell[0];


	/*computing damping variable in TDPA*/
	//if (abs(velocity[0]) < 10){
	//	velocity[0] = 0.0;
	//}
	/*optimization*/
	velocity = velocity*0.001;
	
	CMatrix am("A", 2, 4);
	am.m_pData[0][0] = velocity[0];
	am.m_pData[0][1] = velocity[1];
	am.m_pData[0][2] = 0.0;
	am.m_pData[0][3] = 0.0;

	am.m_pData[1][0] = 0.0;
	am.m_pData[1][1] = 0.0;
	am.m_pData[1][2] = velocity[0];
	am.m_pData[1][3] = velocity[1];

	CMatrix bm("B", 4, 1);
	bm.m_pData[0][0] = velocity[0] * velocity[0];
	bm.m_pData[1][0] = velocity[0] * velocity[1];
	bm.m_pData[2][0] = velocity[0] * velocity[1];
	bm.m_pData[3][0] = velocity[1] * velocity[1];

	CMatrix cm("C", 5, 1);
	cm.m_pData[0][0] =0.0;
	cm.m_pData[1][0] = 0.0;
	cm.m_pData[2][0] = 0.0;
	cm.m_pData[3][0] = 0.0;
	cm.m_pData[4][0] = -ep->observedEnergy*0.001/SampleTime;
	//std::cin >> a;
	//a.FillSimulatedInput();
	CMatrix ATA = (am.Transpose()*am);
	CMatrix dm("D", 5, 5);
	dm.m_pData[0][0] = ATA.m_pData[0][0];
	dm.m_pData[0][1] = ATA.m_pData[0][1];
	dm.m_pData[0][2] = ATA.m_pData[0][2];
	dm.m_pData[0][3] = ATA.m_pData[0][3];
	dm.m_pData[0][4] = -bm.m_pData[0][0];

	dm.m_pData[1][0] = ATA.m_pData[1][0];
	dm.m_pData[1][1] = ATA.m_pData[1][1];
	dm.m_pData[1][2] = ATA.m_pData[1][2];
	dm.m_pData[1][3] = ATA.m_pData[1][3];
	dm.m_pData[1][4] = -bm.m_pData[1][0];

	dm.m_pData[2][0] = ATA.m_pData[2][0];
	dm.m_pData[2][1] = ATA.m_pData[2][1];
	dm.m_pData[2][2] = ATA.m_pData[2][2];
	dm.m_pData[2][3] = ATA.m_pData[2][3];
	dm.m_pData[2][4] =- bm.m_pData[2][0];

	dm.m_pData[3][0] = ATA.m_pData[3][0];
	dm.m_pData[3][1] = ATA.m_pData[3][1];
	dm.m_pData[3][2] = ATA.m_pData[3][2];
	dm.m_pData[3][3] = ATA.m_pData[3][3];
	dm.m_pData[3][4] = -bm.m_pData[3][0];

	dm.m_pData[4][0] = bm.m_pData[0][0];
	dm.m_pData[4][1] = bm.m_pData[1][0];
	dm.m_pData[4][2] = bm.m_pData[2][0];
	dm.m_pData[4][3] = bm.m_pData[3][0];
	dm.m_pData[4][4] = 0.0;

	CMatrix DTD = dm.Transpose()*dm;
	double uu = 0.00001;
	CMatrix un("I", 5, 5);
	un.m_pData[0][0] = uu;
	un.m_pData[0][1] = 0.0;
	un.m_pData[0][2] = 0.0;
	un.m_pData[0][3] = 0.0;
	un.m_pData[0][4] = 0.0;

	un.m_pData[1][0] = 0.0;
	un.m_pData[1][1] = uu;
	un.m_pData[1][2] = 0.0;
	un.m_pData[1][3] = 0.0;
	un.m_pData[1][4] = 0.0;

	un.m_pData[2][0] = 0.0;
	un.m_pData[2][1] = 0.0;
	un.m_pData[2][2] = uu;
	un.m_pData[2][3] = 0.0;
	un.m_pData[2][4] = 0.0;

	un.m_pData[3][0] = 0.0;
	un.m_pData[3][1] = 0.0;
	un.m_pData[3][2] = 0.0;
	un.m_pData[3][3] = uu;
	un.m_pData[3][4] = 0.0;

	un.m_pData[4][0] = 0.0;
	un.m_pData[4][1] = 0.0;
	un.m_pData[4][2] = 0.0;
	un.m_pData[4][3] = 0.0;
	un.m_pData[4][4] = uu;
	CMatrix DTDsc = DTD + un;
	CMatrix sudo_inverse=DTDsc.Inverse()*dm.Transpose();
	CMatrix velocity2d("v2d", 2, 1);
	velocity2d.m_pData[0][0] = velocity[0];
	velocity2d.m_pData[1][0] = velocity[1];
	CMatrix Xm = sudo_inverse*cm;
	CMatrix alm("Alpha",2,2);

	//alm= sudo_inverse*velocity2d;
	//CMatrix Force_matrix = alm*velocity2d;
	/*Matrix2d BB;
	BB<< 0.0, 0.0,
		0.0, 0.0;*/
	//Matrix < double, 2, 4 > A ;
	//A << velocity[0], velocity[1], 0.0, 0.0,
	//	0.0, 0.0, velocity[0], velocity[1];

	//Matrix < double, 4, 1 > B;
	//B<<velocity[0] * velocity[0] ,
	//	velocity[0] * velocity[1],
	//	velocity[0] * velocity[1] ,
	//	velocity[1] * velocity[1] ;

	//Matrix < double, 5, 1 > C;//instead of numbers you can type Dynamic to have variable size(better to do that for sizes larger than32(4*8) or unkown)
	//C << 0.0,
	//	0.0,
	//	0.0,
	//	0.0,
	//	-ep->observedEnergy;
	//MatrixXd Ad;
	//Ad = A;
	//Matrix < double, 4, 4 > ATA;
	//ATA = Ad.transpose();// *A;
	//MatrixXd Add= ATA*A;

	//Matrix < double, 5, 5 > D;
	//D << ATA(0, 0), ATA(0, 1), ATA(0, 2), ATA(0, 3), -B(0, 0),
	//	ATA(1, 0), ATA(1, 1), ATA(1, 2), ATA(1, 3), -B(1, 0),
	//	ATA(2, 0), ATA(2, 1), ATA(2, 2), ATA(2, 3), -B(2, 0),
	//	ATA(3, 0), ATA(3, 1), ATA(3, 2), ATA(3, 3), -B(3, 0),
	//	B(0, 0), B(1, 0), B(2, 0), B(3, 0), 0.0;

	//Matrix < double, 5, 1 > ag;
	//ag = D.completeOrthogonalDecomposition().pseudoInverse();// *C;
	
//	HDdouble A[2][4]= {
//		{ velocity[0], velocity[1], 0.0,         0.0},
//		{ 0.0,         0.0,         velocity[0], velocity[1]}
//	};
//	HDdouble B[4][1] = { 
//		{ velocity[0] * velocity[0] },
//		{ velocity[0] * velocity[1] },
//		{ velocity[0] * velocity[1] },
//		{ velocity[1] * velocity[1] }
//	};
//	HDdouble C[4][1] = { { 0.0 }, { 0.0 }, { 0.0 }, { -ep->observedEnergy } };
//	/*hduMatrix A, AT, ATA;
//	A.set(Aa);
//	AT.set(Aa);
//	AT.transpose();
//	ATA = AT * A;*/
//
//	HDdouble AT[4][2];
//	transpose(&A[0][0], &AT[0][0], 2, 4);
//
//	HDdouble ATA[4][4];
//	multiply(&AT[0][0], &A[0][0], &ATA[0][0], 4, 2, 4);
//	
///*
//	hduMatrix BT;
//	BT.set(Ba);
//	BT.transpose();
//	HDdouble*/
//
//	HDdouble D[5][5] = {
//		{ATA[0][0], ATA[0][1], ATA[0][2], ATA[0][3], -B[0][0]},
//		{ATA[1][0], ATA[1][1], ATA[1][2], ATA[1][3], -B[1][0]},
//		{ATA[2][0], ATA[2][1], ATA[2][2], ATA[2][3], -B[2][0]},
//		{ATA[3][0], ATA[3][1], ATA[3][2], ATA[3][3], -B[3][0]},
//		{B[0][0],   B[1][0],   B[2][0],   B[3][0],           0}
//	};
	
	//vp = dotProduct(velocity, force)*force / dotProduct(force,force);
	if (ep->counter > 0.0)
	{
		if (ep->observedEnergy <= 0.0)// && abs(velocity[0])>1)
		{
			ep->alpha = -ep->observedEnergy / (dotProduct(velocity , velocity) * sampleTime+0.001 );
			/*Optimization*/
			alm.m_pData[0][0] = Xm.m_pData[0][0];
			alm.m_pData[0][1] = Xm.m_pData[1][0];
			alm.m_pData[1][0] = Xm.m_pData[2][0];
			alm.m_pData[1][1] = Xm.m_pData[3][0];
			//ep->alpha = -ep->observedEnergy / (dotProduct(vp, vp) * sampleTime + 0.001);
			//if (ep->alpha > 0.01){
			//	ep->alpha = 0.01;
			//}
		}
		else
		{
			ep->alpha = 0.0;
			/*Optimization*/
			alm.m_pData[0][0] = 0.0;
			alm.m_pData[1][0] = 0.0;
			alm.m_pData[0][1] = 0.0;
			alm.m_pData[1][1] = 0.0;
		}
	}
	else
	{
		ep->alpha = 0.0;
		/*Optimization*/
		alm.m_pData[0][0] = 0.0;
		alm.m_pData[1][0] = 0.0;
		alm.m_pData[0][1] = 0.0;
		alm.m_pData[1][1] = 0.0;
	}
	//if (abs(velocity[0]) < 0.1){
	//	ep->alpha = ep->oldAlpha;
	//}
	/*computing damping variable in  predictive TDPA*/


	//if (oldEnergy > ep->observedEnergy && ep->observedEnergy <= ePassive)// && abs(velocity[0])>7)// && abs(velocity[0])>15)///change two ands to one
	//{
	//	ep->alpha = -(ep->observedEnergy - ePassive) / (velocity[0] * velocity[0] * sampleTime+0.001);// *(1 - ep->observedEnergy / (ep->Emaxp + 0.00001));
	//}
	//else
	//{
	//	ep->alpha = 0.0;
	//}



	//if (positionTwell[0] < kForceInfluence)
	//{
	//	force[0] -= ep->alpha*velocity[0];
	//}
	//else{
	//	force[0] = 0;
	//}
	//	
	/*energy reference TDPA*/

	//if (ep->observedEnergy < ePassive && abs(velocity[0])>1)///change two ands to one
	//{
	//	ep->alpha = -(ep->observedEnergy - ePassive) / (velocity[0] * velocity[0] * sampleTime);
	//}
	//else
	//{
	//	ep->alpha = 0.0;
	//}
	Fpc = 0.0*vp;
	if (positionTwell[0]<0.0 || positionTwell[1]<0.0)// kForceInfluence)
	{
		//force -= ep->alpha*velocity;
		//force -= ep->alpha*vp;
		//Fpc = ep->alpha*vp;

		/*Optimization*/
		CMatrix Force_matrix = alm*velocity2d;
		Fpc[0] = Force_matrix.m_pData[0][0];
		Fpc[1] = Force_matrix.m_pData[1][0];
		Fpc[2] = 0.0;
		force -= Fpc;
	}

	/*Optimization*/
	velocity = velocity*1000;



	/* removing noisy behvior of TDPA*/
	//	HDdouble dx;
	//	HDdouble Fpc;
	//	HDdouble Fc;
	//	const HDdouble delta = 0.055;
	//	const HDdouble Fmax = 100.000;
	////	//add a for loop if initial is not zero
	////	if (ep->counter == 0){
	////		oldEnergy = 0.0;
	////		ep->energy = 0.0;
	////		ep->oldx = 0.0;
	////		ep->observedEnergy=0.0;
	////		ep->olddx = 0.0;
	////		ep->doldFc = 0.0;
	////		Fc = 0.0;
	////		Fpc = 0.0;
	////	}
	//	if (ep->counter == 1){
	//		ep->energy = 0.0;
	//		ep->doldFc = 0.0;
	//		ep->oldFc = 0.0;
	//		ep->observedEnergy = 0.0;
	//		ep->oldx = 0.0;
	//	}
	//	oldEnergy = ep->observedEnergy;
	//	//if (oldEnergy > ep->energy)
	//	//{
	//	//	z = 0.5*ep->oldx * ep->oldx;
	//	//	l = ep->oldP*z / (lambda + z*ep->oldP*z);
	//	//	p = (1 / lambda)*(ep->oldP - l*z*ep->oldP);
	//	//	khat = ep->oldkhat + l*(oldEnergy - z*ep->oldkhat);
	//	//	if (khat < 0.0){
	//	//		khat = 0.0;
	//	//	}
	//	//}
	//	//else
	//	//{
	//	//	khat = ep->oldkhat;
	//	//}
	//	//khat = 0.1;
	//	//updating old variables;
	//	//ep->oldP = p;
	//	//ep->oldkhat = khat;

	////ePassive = 0.5*khat*positionTwell[0] * positionTwell[0]*1.0;
	//dx = position[0] - ep->oldx;
	////ep->observedEnergy = oldEnergy + ep->oldFc*dx + Forcedx[0]*dx;
	////if (ep->observedEnergy < 0.0){
	////	Fpc = -(ep->observedEnergy) / (dx);
	////}
	////else{
	////	Fpc = 0.0;
	////}

	//

	////if (position[0] <= 0.0){
	////	Fpc = 0.0;
	////	ep->observedEnergy = oldEnergy+ep->oldFc*dx;
	////}
	////else{
	//	if (abs(dx)<delta){
	//		Fpc =  ep->oldFpc;
	//		ep->observedEnergy = oldEnergy + ep->oldFc*dx;
	//	}
	//	else{
	//		if (abs(dx) == delta && dx == -1*ep->olddx){
	//			ep->observedEnergy = oldEnergy + ep->doldFc*ep->olddx;
	//		}
	//		else{
	//			ep->observedEnergy = oldEnergy + ep->oldFc*dx;
	//		}
	//		ep->olddx = dx;
	//		ep->doldFc = ep->oldFc;
	//		if (ep->observedEnergy < 0.0){
	//			Fpc = -(ep->observedEnergy) / dx;
	//		}
	//		else{
	//			Fpc = 0.0;
	//			}
	//		}
	//Fc = Forcedx[0] + Fpc;
	//if (Fc>Fmax){
	//	Fc = Fmax;
	//	Fpc = (Fmax - (Forcedx[0]));
	//}
	//if (Fc < -Fmax){
	//	Fc = -Fmax;
	//	Fpc = (-Fmax - (Forcedx[0]));
	//}
	//	
	//force[0] = -Fc;
	////updating data
	////ep->doldFc = ep->oldFc;
	//ep->oldFc = Fc;
	////ep->olddx = dx;
	//ep->oldFpc = Fpc;
	//ep->oldx = position[0];
	//ep->energy = oldEnergy;

	/* Adaptive with dx*/
	//	HDdouble dx;
	//	HDdouble Fpc;
	//	HDdouble Fc;
	//	const HDdouble delta = 0.0230;
	//	const HDdouble Fmax = 7.9000;
	//	//add a for loop if initial is not zero
	//	if (ep->counter == 0){
	//		oldEnergy = 0.0;
	//		ep->energy = 0.0;
	//		ep->oldx = 0.0;
	//		ep->observedEnergy=0.0;
	//		ep->olddx = 0.0;
	//		ep->doldFc = 0.0;
	//		Fc = 0.0;
	//		Fpc = 0.0;
	//	}
	//	if (ep->counter == 1){
	//		ep->energy = 0.0;
	//		ep->doldFc = 0.0;
	//	}
	//	oldEnergy = ep->observedEnergy;
	//	if (oldEnergy > ep->energy)
	//	{
	//		z = 0.5*ep->oldx * ep->oldx;
	//		l = ep->oldP*z / (lambda + z*ep->oldP*z);
	//		p = (1 / lambda)*(ep->oldP - l*z*ep->oldP);
	//		khat = ep->oldkhat + l*(oldEnergy - z*ep->oldkhat);
	//		if (khat < 0.0){
	//			khat = 0.0;
	//		}
	//	}
	//	else
	//	{
	//		khat = ep->oldkhat;
	//	}
	//	//khat = 0.1;
	//	//updating old variables;
	//	ep->oldP = p;
	//	ep->oldkhat = khat;

	//	ePassive = 0.5*khat*positionTwell[0] * positionTwell[0]*1.0;
	//dx = position[0] - ep->oldx;
	//if (position[0] <= 0.0){
	//	Fpc = 0.0;
	//	ep->observedEnergy = oldEnergy+ep->oldFc*dx;
	//}
	//else{
	//	if (abs(dx)<delta){
	//		Fpc = ep->oldFpc;
	//		ep->observedEnergy = oldEnergy + ep->oldFc*dx;
	//	}
	//	else{
	//		if (abs(dx) == delta && dx == -1*ep->olddx){
	//			ep->observedEnergy = oldEnergy + ep->doldFc*dx;
	//		}
	//		else{
	//			ep->observedEnergy = oldEnergy + ep->oldFc*dx;
	//		}
	//		ep->olddx = dx;
	//			ep->doldFc = ep->oldFc;
	//		if (ep->observedEnergy < ePassive){
	//			Fpc = -(ep->observedEnergy - ePassive) / dx;
	//		}
	//		else{
	//			Fpc = 0.0;
	//		}
	//	}
	//}
	//Fc = Forcedx[0] + Fpc;
	//if (Fc>Fmax){
	//	Fc = Fmax;
	//	Fpc = (Fmax - (Forcedx[0]));
	//}
	//if (Fc < -Fmax){
	//	Fc = -Fmax;
	//	Fpc = (-Fmax - (Forcedx[0]));
	//}
	//	force[0] = -Fc;
	//updating data
	//ep->doldFc = ep->oldFc;


	//ep->oldFc = Fc;
	////ep->olddx = dx;
	ep->oldFpc = Fpc;
	//ep->oldx = position[0];
	//ep->energy = oldEnergy;

	/* updating variables*/
	ep->counter++;
	ep->oldVelocity = velocity;
	ep->oldAlpha = ep->alpha;
	printEnergy = oldEnergy - dotProduct(force , velocity )* sampleTime;
	csvEnergy << ep->observedEnergy << endl;
	//cout << sampleTime*force[0] * velocity[0] << endl;
	//cout << ep->observedEnergy << endl;
	//csvEnergy << ep->observedEnergy << endl;
	csvPosition << position << endl;
	csvVelocity << velocity << endl;
	csvTorque << force << endl;
	csvSampling << sampleTime << endl;
	/*optimization*/
	csvAlpha << Xm.m_pData[0][0] << "," << Xm.m_pData[1][0] << "," << Xm.m_pData[2][0] << "," << Xm.m_pData[3][0] << endl;
	//csvAlpha << ep->alpha << endl;
	csvKhat << khat << endl;
	csvControlForce << controlForce << endl;
	//csvEnergy << ep->energy << endl;
	//csvEnergy << ep->alpha << endl;
	//csvEnergy << sampleTime *force[0] * velocity[0] * -1.0 << endl;
	//cout << 1.0/currentRate << endl;


    if (hduVecMagnitude(gimbalAngleOfTwist) < kTorqueInfluence)
    {
        /* >  T = k * r  < We calculate torque by assuming a torsional spring at each joint. 
           T: Torque in Milli Newton . meter (mN.m)
           k: Torque Spring Constant (mN.m / radian)
           r: Angle of twist from fulcrum point (radians)*/
        hduVecScale(gimbalTorque, gimbalAngleOfTwist, kStylusTorqueConstant);
    }
    
    if (jointAngles[0] < kTorqueInfluence)
    {
        /* >  T = k * r  < We calculate torque by assuming a torsional spring at each joint. 
           T: Torque in Milli Newton . meter (mN.m)
           k: Torque Spring Constant (mN.m / radian)
           r: Angle of twist from fulcrum point (radians)*/

        hduVecScale(jointTorque, jointAngleOfTwist, kJointTorqueConstant);
    }
	else
	{
		jointTorque[0] = 0;
		jointTorque[1] = 0;
		jointTorque[2] = 0;
	}

// Clamp the base torques to the nominal values. 
    for (int i=0; i<3; i++)
    {
        if (jointTorque[i] > nominalBaseTorque[i])
            jointTorque[i] = nominalBaseTorque[i];
        else if (jointTorque[i] < -nominalBaseTorque[i])
            jointTorque[i] = -nominalBaseTorque[i];
    }
	jointTorque[1] = 0;
	jointTorque[2] = 0;
// Clamp the gimbal torques to the max continous values. 
    for (int i=0; i<3; i++)
    {
        if (gimbalTorque[i] > maxGimbalTorque[i])
            gimbalTorque[i] = maxGimbalTorque[i];
        else if (gimbalTorque[i] < -maxGimbalTorque[i])
            gimbalTorque[i] = -maxGimbalTorque[i];
    }


    /* Send the forces & torques to the device. */
    /*Switch back between sending forces & torques 
    to the base motors */
    if (!TorqueMode)//ya force ro mide be 3 joint aval ya torque
        hdSetDoublev(HD_CURRENT_FORCE, force);
    else
        hdSetDoublev(HD_CURRENT_JOINT_TORQUE, jointTorque);

    //hdSetDoublev(HD_CURRENT_GIMBAL_TORQUE, gimbalTorque);//age 6 daraje azadi bashe (ke male ma nist) torque joint 4-6 ro ham mide

//    printf("%f\t%f\t%f\n\n", jointTorque[0], jointTorque[1], jointTorque[2]);
    /* End haptics frame. */
	//hdGetDoublev(HD_CURRENT_FORCE, sensorForce);
	//cout << sensorForce[0] << ',' << force[0] << endl;

    hdEndFrame(hHD);


    /* Check for errors and abort the callback if a scheduler error
       is detected. */
    if (HD_DEVICE_ERROR(error = hdGetError()))
    {
        hduPrintError(stderr, &error, 
                      "Error detected while rendering gravity well\n");
        
        if (hduIsSchedulerError(&error))
        {
            return HD_CALLBACK_DONE;
        }
    }

    /* Signify that the callback should continue running, i.e. that
       it will be called again the next scheduler tick. */
	//csvEnergy.close();
    return HD_CALLBACK_CONTINUE;
}

bool initDemo(void)
{
    HDErrorInfo error;
    int calibrationStyle;
    printf("Calibration\n");

    hdGetIntegerv(HD_CALIBRATION_STYLE, &calibrationStyle);
    if (calibrationStyle & HD_CALIBRATION_AUTO || calibrationStyle & HD_CALIBRATION_INKWELL)
    {
        printf("Please prepare for starting the demo by \n");
        printf("placing the device at its reset position.\n\n");
        printf("Press any key to continue...\n");
        getch();
        return 1;
    }
    if (calibrationStyle & HD_CALIBRATION_ENCODER_RESET )
    {
        printf("Please prepare for starting the demo by \n");
        printf("placing the device at its reset position.\n\n");
        printf("Press any key to continue...\n");

        getch();

        hdUpdateCalibration(calibrationStyle);
        if (hdCheckCalibration() == HD_CALIBRATION_OK)
        {
            printf("Calibration complete.\n\n");
            return 1;
        }
        if (HD_DEVICE_ERROR(error = hdGetError()))
        {
            hduPrintError(stderr, &error, "Reset encoders reset failed.");
            return 0;           
        }
    }
}
/*****************************************************************************/
