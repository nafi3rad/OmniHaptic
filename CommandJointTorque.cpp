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

static const hduVector3Dd maxGimbalTorque(188.0,188.0,48.0); //mNm
static const hduVector3Dd nominalBaseTorque(400.0,350.0,200.0); //mNm
static bool TorqueMode = true;

HDCallbackCode HDCALLBACK jointTorqueCallback(void *data);// declare functions that you wanna define later but u used them before that
HDSchedulerHandle hGravityWell = HD_INVALID_HANDLE;

void mainLoop(void);
bool initDemo(void);

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

} EnergyStruct;

ofstream csvEnergy;
ofstream csvTorque;
ofstream csvPosition;
ofstream csvVelocity;
ofstream csvSampling;
ofstream csvAlpha;
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
	HDdouble energy;
	HDdouble oldEnergy;
	energy = 0.0;
	oldEnergy = 0.0;
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
	EnergyStruct e;
	e.counter = 0;
	e.energy = 0;
	e.observedEnergy = 0;
    /* Schedule the main callback that will render forces to the device. */
    hGravityWell = hdScheduleAsynchronous(
        jointTorqueCallback, &e, 
        HD_MAX_SCHEDULER_PRIORITY);
	
    hdEnable(HD_FORCE_OUTPUT);
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

    /* For cleanup, unschedule callback and stop the scheduler. */
    hdStopScheduler();
    hdUnschedule(hGravityWell);
	csvEnergy.close();
	csvTorque.close();
	csvPosition.close();
	csvVelocity.close();
	csvSampling.close();
	csvAlpha.close();
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

/*******************************************************************************
 Servo callback.  
 Called every servo loop tick.  Simulates a gravity well, which sucks the device 
 towards its center whenever the device is within a certain range.
*******************************************************************************/
HDCallbackCode HDCALLBACK jointTorqueCallback(void *data)
{
	EnergyStruct* ep = (EnergyStruct*) data;
	const HDdouble kStiffness = 0.1;// 0.075; /* N/mm */positive k is passive
	const HDdouble khat = 0.09; //estimation of the k
	const HDdouble bDamping = -0.002;// 75;positive b is passive
    const HDdouble kStylusTorqueConstant = 500; /* torque spring constant (mN.m/radian)*/
    const HDdouble kJointTorqueConstant = 12000; /* torque spring constant (mN.m/radian)*/
 
    const HDdouble kForceInfluence = 0.0; /* mm */
	const HDdouble kTorqueInfluence = 0.0;//3.14; /* radians */

    /* This is the position of the gravity well in cartesian
       (i.e. x,y,z) space. */
    static const hduVector3Dd wellPos(0,0,0);
    static const hduVector3Dd stylusVirtualFulcrum(0.0, 0.0, 0.0); // In radians
    static const hduVector3Dd jointVirtualFulcrum(0.0, 0.0, 0.0); // In radians
    
    HDErrorInfo error;
    hduVector3Dd position;
	hduVector3Dd velocity;

	//double energy = 0;
	HDdouble sampleTime;
	HDdouble ePassive;
	HDdouble oldEnergy;
	HDint currentRate;
	hduVector3Dd force;
	hduVector3Dd sensorForce;
    hduVector3Dd positionTwell;
    hduVector3Dd gimbalAngles;
    hduVector3Dd gimbalTorque;
    hduVector3Dd gimbalAngleOfTwist;
    hduVector3Dd jointAngles;
    hduVector3Dd jointTorque;
    hduVector3Dd jointAngleOfTwist;
    HHD hHD = hdGetCurrentDevice();

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
    if (positionTwell[0] < kForceInfluence)
    {
        /* >  F = k * x  < 
           F: Force in Newtons (N)
           k: Stiffness of the well (N/mm)
           x: Vector from the device endpoint position to the center 
           of the well. */
        //hduVecScale(force, positionTwell, kStiffness);
		//force[0] = kStiffness*positionTwell[0];
		force[0] = -bDamping * velocity[0] + kStiffness*positionTwell[0];
		force[1] = 0;
		force[2] = 0;

    }
	else{
		force[0] = 0;
		force[1] = 0;
		force[2] = 0;
	}
	sampleTime = 1.0 / currentRate;
	
	ep->energy += sampleTime * force[0] * velocity[0] * -1.0;
	ePassive = 0.5*khat*positionTwell[0] * positionTwell[0] / sampleTime;
	oldEnergy = ep->observedEnergy;
	 /*computing observed energy*/
	if (ep->counter>0)
	{
		ep->observedEnergy += -force[0] * velocity[0]+ 
			ep->oldAlpha * ep->oldVelocity[0] * ep->oldVelocity[0];
	}
	else
	{
		ep->observedEnergy = -force[0] * velocity[0];
	}

	/*computing damping variable in TDPA*/
	if (ep->counter > 0)
	{
		if (ep->observedEnergy < 0)
		{
			ep->alpha = -ep->observedEnergy / (velocity[0] * velocity[0]);
		}
		else
		{
			ep->alpha = 0.0;
		}
	}
	else
	{
		ep->alpha = 0.0;
	}

	/*computing damping variable in  predictive TDPA*/
	//if (oldEnergy > ep->observedEnergy)///change two ands to one
	//{
	//	if (ep->observedEnergy < ePassive && abs(velocity[0])>1 )
	//	{
	//		ep->alpha = -(ep->observedEnergy-ePassive) / (velocity[0] * velocity[0]);
	//	}
	//	else
	//	{
	//		ep->alpha = 0.0;
	//	}
	//}
	//else
	//{
	//	ep->alpha = 0.0;
	//}

	if (positionTwell[0] < kForceInfluence)
	{
		force[0] -= ep->alpha*velocity[0];
	}
		



	/* updating variables*/
	ep->counter++;
	ep->oldVelocity = velocity;
	ep->oldAlpha = ep->alpha;
	//cout << sampleTime*force[0] * velocity[0] << endl;
	//cout << ep->observedEnergy << endl;
	csvEnergy << ep->observedEnergy << endl;
	csvPosition << position[0] << endl;
	csvVelocity << velocity[0] << endl;
	csvTorque << force[0] << endl;
	csvSampling << sampleTime << endl;
	csvAlpha << ep->alpha << endl;
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
