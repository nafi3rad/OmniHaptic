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

//static const hduVector3Dd maxGimbalTorque(188.0,188.0,48.0); //mNm
//static const hduVector3Dd nominalBaseTorque(400.0,350.0,200.0); //mNm
//static bool TorqueMode = true;

//HDCallbackCode HDCALLBACK jointTorqueCallback(void *data);// declare functions that you wanna define later but u u


//—————— NLopt optimizer code
//typedef struct {
//	double a00, a01, a10, a11, b0, b1;
//} my_constraint_data;

typedef struct {
	double E;
	double Px;
	double Py;
} my_func_data;

double myfunc(unsigned n, const double *x, double *grad, void *data)
{
	my_func_data *d = (my_func_data *)data;
	double r = d->E - x[0] * d->Px *  d->Px - x[1] * d->Py *  d->Py - 2 * x[2] * d->Px *  d->Py;
	if (grad) {
		grad[0] = -2.0 * d->Px *  d->Px * r; // TODO? ronde objective func be ronde x0
		grad[1] = -2.0 * d->Py *  d->Py * r; // TODO? ronde objective func be ronde x1
		grad[2] = -4.0 * d->Px *  d->Py * r;
	}

	return r * r;
}

double myconstraint1(unsigned n, const double *x, double *grad, void *data)
{
	//my_constraint_data *d = (my_constraint_data *)data;
	if (grad) {
		grad[0] = -x[1]; // ronde constraint be ronde x0
		grad[1] = -x[0]; // ronde constraint be ronde x1
		grad[3] = 2 * x[2];
	}
	double r = x[2] * x[2] - x[0] * x[1]; 
	return r; // constraint is assumed to be r <= 0
}

void solve() {
	double lb[2] = { -1.0e9, 0 };
	nlopt_opt opt;
	opt = nlopt_create(NLOPT_LD_SLSQP, 3);
	nlopt_set_lower_bounds(opt, lb);
	//my_constraint_data cdata;
	my_func_data fndata;
	fndata.Px = 1.0;//positionx;
	fndata.Py = 0.2; //positiony;
	fndata.E = 2;// Eactual;
	nlopt_set_min_objective(opt, myfunc, &fndata);
	nlopt_add_inequality_constraint(opt, myconstraint1, NULL, -1e-8);
	//nlopt_add_inequality_constraint(opt, myconstraint2, &cdata, 1e-8);

	nlopt_set_xtol_rel(opt, 1e-4);

	double x[3] = { 0.1, 0.1 , 0.1 };
	double minf;

	if (nlopt_optimize(opt, x, &minf) < 0){
		cout << "nlopt failed!";
	}
	else {
		cout << "min=" << minf << " at x=(" << x[0] << "," << x[1] << "," << x[2] << ")\n";
	}

}

//—————

int main(){
	cout << "start...\n";
	solve();
	getch();
}

/*
Fe=[0.5;0.1];
velo=-[0.01;0.03];
CFp=5;
T=0.001;
E=Fe'*velo*T;
dE=E/T;
fun=@(x)((Fe'*Fe-x'*x)^2);

A=[-Fe';velo];
B=[-0.000001;0];
x0=[0.1;0.1];
myopts = optimoptions('fmincon','Algorithm','sqp');
Xp=fmincon(fun,x0,A,B,[],[],[],[],[])%,myopts);
Xm=Xp;
Fc=Xm;
Eee=(Fc)'*velo*T;
CF=[Fe(1),-Fe(2)]*Fc

*/