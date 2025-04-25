/* state.cpp */

// Defines methods of class State.

#include "state.h"
#include <iostream>
#include <cmath>
#define PI 3.1416
using namespace std;

/*
	method int setStandingStill
	
	This method set the initial conditions of standing still phase
*/
int States::setStandingStill(){
	// Right and left leg tip positions
	Rtipx = startingx; Rtipy = startingy; 
	Rtipz = startingz + hipSeparation;
	Ltipx = startingx; Ltipy = startingy;
	Ltipz = startingz - hipSeparation;
	
	// Right and left leg knee positions
	Rkneex = Rtipx; Rkneey = Rtipy + lowerLeg; Rkneez = Rtipz;
	Lkneex = Ltipx; Lkneey = Ltipy + lowerLeg; Lkneez = Ltipz;
	
	// Right and left hip joint postions
	Rhipx = Rkneex; Rhipy = Rkneey + upperLeg; Rhipz = Rkneez;
	Lhipx = Lkneex; Lhipy = Lkneey + upperLeg; Lhipz = Lkneez;
	
	// Right and left leg angles
	thetaR = 0.0; zetaR = 0.0;
	thetaL = 0.0; zetaL = 0.0;

	return 0;
}

/*
	method int set1liftRightLeg

	This method calculates the final and initial positions of 
	the tip of the legs in State 1.

    The final positions are determined trigonometrically.
*/
int States::set1liftRightLeg(){
	// Final position of the tip of right leg is when the angle
	// thetaR reaches thetaMax(an input) with lower leg hanging down
	// vertically.
	xfR = Rhipx + upperLeg*sin(thetaMax);
	yfR = Rhipy - upperLeg*cos(thetaMax) - lowerLeg;
	zfR = Rtipz;

	// Initial position of the tip of right leg is the position of the 
	// tip at the end of the preceding phase.
	xiR = Rtipx;
	yiR = Rtipy;
	ziR = Rtipz;
	
	// Initial and final positions of the tip of left leg.
	// Left leg does not move during this state.
	xiL = xfL = Ltipx;
	yiL = yfL = Ltipy;
	ziL = zfL = Ltipz;

	return 0;
}


/*
    Method liftRightLeg

    This method calculates the exact
	positions of the joints of the right leg in each frame during 
	State 1. The left leg is fixed at its position
	from the previous state. Delta theta and delta zeta are calculated,
	and the precise x, y coords are calculated from them.

    Returns 1 when the state is ready to terminate.
*/
int States::liftRightLeg(){
	// set dxR and dyR
	dxR = (xfR - xiR) / N;
	dyR = (yfR - yiR) / N;

	// Compute the partial differentials.
	double dxOVERdtheta;
	dxOVERdtheta = upperLeg*cos(thetaR) + lowerLeg*cos(thetaR-zetaR);
	double dxOVERdzeta;
	dxOVERdzeta = -lowerLeg*cos(thetaR-zetaR);
	double dyOVERdtheta;
	dyOVERdtheta = upperLeg*sin(thetaR) + lowerLeg*sin(thetaR-zetaR);
	double dyOVERdzeta;
	dyOVERdzeta = -lowerLeg*sin(thetaR-zetaR);

	// Determinant of the Jacobian J
	double detJ;
	detJ = upperLeg*lowerLeg*sin(zetaR);
	
	// If -zero < detJ < zero, do not use inverse kinematics,
	// just set del_thetaR and del_zetaR to some predefined values
	double zero = 0.1;
	if (detJ >= -zero && detJ <= zero){
		del_thetaR = thetaMax / (N / 2);
		del_zetaR = thetaMax / N;
	}

	// If detJ is not singular, use inverse kinematics to compute
	// del_thetaR and del_zetaR.
	else {
		del_thetaR = (dyOVERdzeta * dxR - dxOVERdzeta * dyR) / detJ;
		del_zetaR = (-dyOVERdtheta * dxR + dxOVERdtheta * dyR) / detJ;
	}

	// Calculate the angles thetaR and zetaR. Check whether the 
	// final point is reached or not.
	thetaR = thetaR + del_thetaR;
	zetaR = zetaR + del_zetaR;
	thetaL = 0.0; zetaL = 0.0;
	Rkneex = Rhipx + upperLeg*sin(thetaR);
	Rkneey = Rhipy - upperLeg*cos(thetaR);
	Rtipx = Rkneex + lowerLeg*sin(thetaR - zetaR);
	Rtipy = Rkneey - lowerLeg*cos(thetaR - zetaR);

	// If coords of the right leg are within a certain distance from
	// the calculated final position, terminate this state.
	if (((Rtipx-xfR)>-zero && (Rtipx-xfR)<zero) 
		&& ((Rtipy-yfR)>-zero && (Rtipy-yfR)<zero)){
	    // Rtipx=xfR;Rtipy=yfR;
		return 1;
	}

	return 0;
}

/*
	method set2bodyLeanForwardRightLegStepOutFinal
	This method sets the initial and the final positions of the tip of
	legs in state 2 ( body leans forward and right leg steps out).
*/
int States::set2bodyLeanForwardRightLegStepOut(){
	// Reset the counter
	k = 0;
	N = N0*2;
	// Initial position of the tip of right leg.
	xiR = Rtipx; yiR = Rtipy; ziR = Rtipz;
	// Final position of the tip of right leg.
	xfR = Ltipx + s;yfR = Ltipy; zfR = Rtipz;
	
	// Initial and final positions of the tip of left leg same as in
	// the state "lift right leg," so do nothing.
	
	// Set initial dxR and dyR.
	dxR = (xfR - xiR)/N; dyR = (yfR - yiR)/N;

	// Set initial values of thetaR and zetaR for this phase
	thetaR = thetaMax; zetaR = thetaR;
	// Set the initial values of alpha as 90 degrees.
	alfa = PI/2.0;
	thetaL = -(PI/2.0 - alfa);zetaL = 0.0;
	return 0;
}

/*
	Method bodyLeanForwardRightLegStepOut

	Describe the process of the body leaning forward 
	and the right leg steppping forward by a distance s.
	During this process, the counter k is incremented by 1,
	starting at 0, for each frame, and the alpha angle is increased
	by a predetermined amount through the equation
	   del_alfa = 2*A2*(k/N) + A1,	  alfa = alfa + del_alfa
    where A2 and A1 are input parameters.
    At each new alfa value, the new position of the tip of the right leg
	is recalculated; then del_x and del_y (the increment by which to move the
	tip) are also recalculated, and inverse kinematics is done to
	determine the increase of thetaR and zetaR angles.
*/
int States::bodyLeanForwardRightLegStepOut(){
	//calculate the increment of alfa
	del_alfa = (2*A2*k/N + A1)/N;
	alfa = alfa - del_alfa;
	thetaL = -(PI/2.0 - alfa);zetaL = 0.0;
	k = k + 1;
	//determine new hip positions
	Lhipx = (upperLeg+lowerLeg)*cos(alfa)+Ltipx; 
	Rhipx = Lhipx;
	Lhipy = (upperLeg+lowerLeg)*sin(alfa); Rhipy = Lhipy;

	//compute the partial differentials
	double dxOVERdtheta;
	dxOVERdtheta = upperLeg*cos(thetaR) + lowerLeg*cos(thetaR-zetaR);
	double dxOVERdzeta;
	dxOVERdzeta = -lowerLeg*cos(thetaR-zetaR);
	double dyOVERdtheta;
	dyOVERdtheta = upperLeg*sin(thetaR) + lowerLeg*sin(thetaR-zetaR);
	double dyOVERdzeta;
	dyOVERdzeta = -lowerLeg*sin(thetaR-zetaR);

	// determinant of the Jacobian J
	double detJ;
	//detJ = dxOVERdtheta * dyOVERdzeta - dxOVERdzeta * dyOVERdtheta;
	detJ = upperLeg*lowerLeg*sin(zetaR);
	// if -zero < detJ < zero do not use inverse kinematics,
	// just set del_thetaR and del_zetaR to some predefined values
	double zero;
	zero = 0.1*upperLeg*lowerLeg;
	if (detJ >= -zero && detJ <= zero){
		//del_thetaR = -thetaMax / N/100;
		//del_zetaR = thetaMax / N / 100;

		if (cos(thetaR) >= sin(thetaR)){
			dxR = (xfR - Rtipx) / (N - k + 1);
			dyR = dxR * sin(thetaR) / cos(thetaR);
		}
		else{
			dyR = (yfR - Rtipy) / (N - k + 1);
			dxR = dyR * cos(thetaR) / sin(thetaR);
		}
		del_thetaR = lowerLeg*(cos(thetaR)*dxR + sin(thetaR)*dyR);
		del_zetaR = del_thetaR;

	}
	// if detJ is not singular, use inverse kinematics to compute
	// del_thetaR and del_zetaR
	else {
		dxR = (xfR - Rtipx) / (N - k + 1);
		dyR = (yfR - Rtipy) / (N - k + 1);
		del_thetaR = (dyOVERdzeta * dxR - dxOVERdzeta * dyR) / detJ;
		del_zetaR = (-dyOVERdtheta * dxR + dxOVERdtheta * dyR) / detJ;
	}
	thetaR = thetaR + del_thetaR;
	zetaR = zetaR + del_zetaR;
	if (zetaR < 0.0) zetaR = 0.0;
	//compute Rtipx, Rtipy, Rkneex, Rkneey
	Rkneex = Rhipx + upperLeg*sin(thetaR);
	Rtipx = Rkneex + lowerLeg*sin(thetaR-zetaR);
	Rkneey = Rhipy - upperLeg*cos(thetaR);
	Rtipy = Rkneey - lowerLeg*cos(thetaR-zetaR);
	//compute Lkneex, Lkneey
	Lkneex = Ltipx + lowerLeg*cos(alfa);
	Lkneey = Ltipy + lowerLeg*sin(alfa);
//cout<<"in step 2: Rtip="<<Rtipx<<" "<<Rtipy<<endl;
	//calculate the termination of this phase
	//zero = 0.2*zero;
	if (((Rtipx-xfR)>-zero && (Rtipx-xfR)<zero) 
		&& ((Rtipy-yfR)>-zero && (Rtipy-yfR)<zero)){
		Rtipx = xfR;
		Rtipy = yfR;
		return 1;
	}
	return 0;	
}


/*
    Method set3stretchRightLegPullLeftLegForward

    This method sets the initial and finial positions of the 
	right hip joint for the State 3 (straighten right leg and
	pull left leg forward).


*/
int States::set3stretchRightLegPullLeftLegForward(){
	// Critical point is the right hip joint. 
	// Initial position of the critical point is Rhipx, y, z from the
	// preceding step
	xiR = Rhipx; yiR = Rhipy; ziR = Rhipz;
	
	// Final position of the critical point is computed
	xfR = Rtipx; yfR = upperLeg + lowerLeg; zfR = Rtipz;
	
	// Angles
	thetaL = -(PI/2 - alfa);zetaL = 0.0;
	
	// Set counter k to zero
	k = 0;
	N = N0/2;
	terminateR =0;
	terminateL = 0;
	return 1;
}


/*
    Method stretchRightLegPullLeftLegForward
	
    This method moves the right-hip-joint toward the position
	where the right leg is stretched and in the vertical position
*/
int States::stretchRightLegPullLeftLegForward(){
	if (terminateR == 0){
		dxR = (xfR - xiR) / N;
		dyR = (yfR - yiR) / N;

		double detJ;
		detJ = upperLeg*lowerLeg*sin(zetaR);

		//compute the partial differentials
		double dxOVERdtheta;
		dxOVERdtheta = -upperLeg*cos(thetaR) - lowerLeg*cos(thetaR-zetaR);
		double dxOVERdzeta;
		dxOVERdzeta = lowerLeg*cos(thetaR-zetaR);
		double dyOVERdtheta;
		dyOVERdtheta = -upperLeg*sin(thetaR) - lowerLeg*sin(thetaR-zetaR);
		double dyOVERdzeta;
		dyOVERdzeta = -lowerLeg*sin(thetaR-zetaR);

		// If detJ near zero
		double zero;
		zero = 0.1*upperLeg*lowerLeg;
		if (detJ < zero){
			if (cos(thetaR) >= sin(thetaR))
				dyR = sin(thetaR)/cos(thetaR)*dxR;
			else
				dxR = cos(thetaR)/sin(thetaR)*dyR;

			del_thetaR = -lowerLeg*(cos(thetaR)*dxR + sin(thetaR)*dyR);
			del_zetaR = del_thetaR;
		}
		// if detJ not near zero
		else{
			del_thetaR = (dyOVERdzeta*dxR - dxOVERdzeta*dyR)/detJ;
			del_zetaR = (-dyOVERdtheta * dxR + dxOVERdtheta * dyR) / detJ;
		}
		thetaR = thetaR + del_thetaR;
		zetaR = zetaR + del_zetaR;
		if (zetaR < 0.0) zetaR = 0.0;

		//compute x, y coordinates of the right critical point
		double Rx, Ry;
		Rx = Rtipx - lowerLeg*sin(thetaR - zetaR) - upperLeg*sin(thetaR);
		Ry = Rtipy + lowerLeg*cos(thetaR - zetaR) + upperLeg*cos(thetaR);

		//right knee and tip position
	
		Rkneex = Rtipx - lowerLeg*sin(thetaR-zetaR);
		Rkneey = Rtipy + lowerLeg*cos(thetaR-zetaR);
		Rhipx = Rkneex - upperLeg*sin(thetaR);
		Rhipy = Rkneey + upperLeg*cos(thetaR);	

		//check termination
		if (((Rx-xfR)<=0.05 && (Rx-xfR)>=-0.05) 
			&& ((Ry-yfR)<=0.05 && (Ry-yfR)>=-0.05)) terminateR = 1;

	}

	// adjust left leg position
	thetaL = thetaL + 0.2/N;
	if (thetaL >= thetaMax) thetaL=thetaMax;
	zetaL = zetaL + 0.2/N;
	if (zetaL >= 3.1416/2.0) zetaL=3.1416/2.0;
	//left hip joint positions
	Lhipx = Rhipx; Lhipy = Rhipy;
	//compute left leg parameters
	Lkneex = Lhipx + upperLeg*sin(thetaL);
	Lkneey = Lhipy - upperLeg*cos(thetaL);
	Ltipx = Lkneex - lowerLeg*sin(zetaL-thetaL);
	Ltipy = Lkneey - lowerLeg*cos(zetaL-thetaL);
	//termnation
	if (terminateR == 1)
		return 1;
	else
		return 0;
}

/*
    Method set4liftLeftLeg

    This method set the initial and final positions of the tip
	of the left leg to be used in the phase "liftLeftLeg"
*/
int States::set4liftLeftLeg(){
	//final position of the tip of left leg
	xfL = Lhipx + upperLeg*sin(thetaMax);
	yfL = Lhipy - upperLeg*cos(thetaMax) - lowerLeg;
	zfL = Ltipz;
	//initial position of the tip of left leg
	xiL = Ltipx;
	yiL = Ltipy;
	ziL = Ltipz;
	
	//intitial and final positions of the tip of right leg
	xiR = xfR = Rtipx;
	yiR = yfR = Rtipy;
	ziR = zfR = Rtipz;

	//right leg angles
	thetaR = 0.0;zetaR = 0.0;

	return 0;
}

/*
    Method liftLeftLeg

    This method calculates in each incremented frame the exact
	positions of the joins of the left leg during the phase of
	"liftLeftLeg", wherease the right leg is fixed at its position
	inherited from the previous phase

*/
int States::liftLeftLeg(){
	// set dxL and dyL
	dxL = (xfL - xiL) / N;
	dyL = (yfL - yiL) / N;

	// compute the partial differentials
	double dxOVERdtheta;
	dxOVERdtheta = upperLeg*cos(thetaL) + lowerLeg*cos(thetaL-zetaL);
	double dxOVERdzeta;
	dxOVERdzeta = -lowerLeg*cos(thetaL-zetaL);
	double dyOVERdtheta;
	dyOVERdtheta = upperLeg*sin(thetaL) + lowerLeg*sin(thetaL-zetaL);
	double dyOVERdzeta;
	dyOVERdzeta = -lowerLeg*sin(thetaL-zetaL);

	// determinant of the Jacobian J
	double detJ;
	//detJ = dxOVERdtheta * dyOVERdzeta - dxOVERdzeta * dyOVERdtheta;
	detJ = upperLeg*lowerLeg*sin(zetaL);
	// if -zero < detJ < zero do not use inverse kinematics,
	// just set del_thetaR and del_zetaR to some predefined values
	double zero = 0.1*upperLeg*lowerLeg;
	if (detJ >= -zero && detJ <= zero){
		del_thetaL = thetaMax / (N / 2);
		del_zetaL = thetaMax / N;
	}
	// if detJ is not singular, use inverse kinematics to compute
	// del_thetaL and del_zetaL
	else {
		del_thetaL = (dyOVERdzeta * dxL - dxOVERdzeta * dyL) / detJ;
		del_zetaL = (-dyOVERdtheta * dxL + dxOVERdtheta * dyL) / detJ;
	}

	// calculate the angles thetaL and zetaL. Check whether the 
	// final point is reached or not.
	thetaL = thetaL + del_thetaL;
	zetaL = zetaL + del_zetaL;
	Lkneex = Lhipx + upperLeg*sin(thetaL);
	Lkneey = Lhipy - upperLeg*cos(thetaL);
	Ltipx = Lkneex + lowerLeg*sin(thetaL - zetaL);
	Ltipy = Lkneey - lowerLeg*cos(thetaL - zetaL);
	//calculate the termination of this phase
	if (((Ltipx-xfL)>-zero && (Ltipx-xfL)<zero) 
		&& ((Ltipy-yfL)>-zero && (Ltipy-yfL)<zero)){
	    Ltipx=xfL;Ltipy=yfL;
		return 1;
	}

	return 0;
}

/*
    Method set5bodyLeanForwardLeftLegStepOut

    This method sets the initial and final positions of the tip of
	left leg;see "set2bodyLeanForwardRightLegStepOut" 
*/
int States::set5bodyLeanForwardLeftLegStepOut(){
	//reset the counter
	k = 0;
	N = N0*2;
	//initial position of the tip of left leg
	xiL = Ltipx; yiL = Ltipy; ziL = Ltipz;
	//final position of the tip of left leg
	xfL = Rtipx + s;yfL = Rtipy; zfL = Ltipz;
	//initial and final positions of the tip of rightt leg same as in
	//state 1 so do nothing
	//dxL and dyL
	dxL = (xfL - xiL)/N; dyL = (yfL - yiL)/N;
	//set initial value of theta and zeta for this phase
	thetaL = thetaMax; zetaL = thetaL;
	//set the initial value of alfa
	alfa = PI/2;
    thetaR = -(PI/2.0 - alfa);zetaR = 0.0;
	return 0;
}

/*
    Method bodyLeanForwardLeftLegStepOut

    See bodyLeanForwardRightLegStepOut
*/
int States::bodyLeanForwardLeftLegStepOut(){
	//calculate the increment of alfa
	del_alfa = (2*A2*k/N + A1)/N;
	alfa = alfa - del_alfa;
	thetaR = -(PI/2.0 - alfa);zetaR = 0.0;
	k = k + 1;
	Rhipx = (upperLeg+lowerLeg)*cos(alfa)+Rtipx; Lhipx = Rhipx;
	Rhipy = (upperLeg+lowerLeg)*sin(alfa); Lhipy = Rhipy;

	//compute the partial differentials
	double dxOVERdtheta;
	dxOVERdtheta = upperLeg*cos(thetaL) + lowerLeg*cos(thetaL-zetaL);
	double dxOVERdzeta;
	dxOVERdzeta = -lowerLeg*cos(thetaL-zetaL);
	double dyOVERdtheta;
	dyOVERdtheta = upperLeg*sin(thetaL) + lowerLeg*sin(thetaL-zetaL);
	double dyOVERdzeta;
	dyOVERdzeta = -lowerLeg*sin(thetaL-zetaL);

	// determinant of the Jacobian J
	double detJ;
	//detJ = dxOVERdtheta * dyOVERdzeta - dxOVERdzeta * dyOVERdtheta;
	detJ = upperLeg*lowerLeg*sin(zetaL);
	// if -zero < detJ < zero do not use inverse kinematics,
	// just set del_thetaR and del_zetaR to some predefined values
	double zero = 0.1*upperLeg*lowerLeg;
	if (detJ >= -zero && detJ <= zero){
		//del_thetaL = thetaMax / N / 100;
		//del_zetaL = thetaMax / N / 100;

		if (cos(thetaL) >= sin(thetaL)){
			dxL = (xfL - Ltipx) / (N - k + 1);
			dyL = dxL * sin(thetaL) / cos(thetaL);
		}
		else{
			dyL = (yfL - Ltipy) / (N - k + 1);
			dxL = dyL * cos(thetaL) / sin(thetaL);
		}
		del_thetaL = lowerLeg*(cos(thetaL)*dxL + sin(thetaL)*dyL);
		del_zetaL = del_thetaL;

	}
	// if detJ is not singular, use inverse kinematics to compute
	// del_thetaL and del_zetaL
	else {
		dxL = (xfL - Ltipx) / (N - k + 1);
		dyL = (yfL - Ltipy) / (N - k + 1);
		del_thetaL = (dyOVERdzeta * dxL - dxOVERdzeta * dyL) / detJ;
		del_zetaL = (-dyOVERdtheta * dxL + dxOVERdtheta * dyL) / detJ;
	}
	thetaL = thetaL + del_thetaL;
	zetaL = zetaL + del_zetaL;
	if (zetaR < 0.0) zetaR = 0.0;
	//compute Rtipx, Rtipy, Rkneex, Rkneey
	Lkneex = Lhipx + upperLeg*sin(thetaL);
	Ltipx = Lkneex + lowerLeg*sin(thetaL-zetaL);
	Lkneey = Lhipy - upperLeg*cos(thetaL);
	Ltipy = Lkneey - lowerLeg*cos(thetaL-zetaL);
	//compute Rkneex, Rkneey
	Rkneex = Rtipx + lowerLeg*cos(alfa);
	Rkneey = Rtipy + lowerLeg*sin(alfa);

	//calculate the termination of this phase
	//zero = 0.2*zero;
	if (((Ltipx-xfL)>-zero && (Ltipx-xfL)<zero) 
		&& ((Ltipy-yfL)>-zero && (Ltipy-yfL)<zero)){
		Ltipx = xfL;
		Ltipy = yfL;
		return 1;
	}

	return 0;
}

/*
    Method set6stretchLeftLegPullRightLegForward

    This method sets the initial and final positions used in the
	phase "stretchLeftLegPullRightLegForward"; see
	"stretchRightLegPullLeftLegForward" for details
*/
int States::set6stretchLeftLegPullRightLegForward(){
	// for the stretch out of the left leg, the critical point is
	// the left hip joint. 
	// Initial position of the critical point is Lhipx, y, z of the
	// preceeding step
	xiL = Lhipx; yiL = Lhipy; ziL = Lhipz;
	// Final position of the critical point is computed
	xfL = Ltipx; yfL = upperLeg + lowerLeg; zfL = Ltipz;

	//angles
	thetaR = -(PI/2 - alfa);zetaR = 0.0;
	//set counter k to zero
	k = 0;
	N = N0/2;
	terminateR =0;
	terminateL = 0;

	return 0;
}

/*
    Method stretchLeftLegPullRightLegForward

    See "stretchRightLegPullLeftLegForward" for details
*/
int States::stretchLeftLegPullRightLegForward(){
	if (terminateL == 0){
		dxL = (xfL - xiL) / N;
		dyL = (yfL - yiL) / N;

		double detJ;
		detJ = upperLeg*lowerLeg*sin(zetaL);

		//compute the partial differentials
		double dxOVERdtheta;
		dxOVERdtheta = -upperLeg*cos(thetaL) - lowerLeg*cos(thetaL-zetaL);
		double dxOVERdzeta;
		dxOVERdzeta = lowerLeg*cos(thetaL-zetaL);
		double dyOVERdtheta;
		dyOVERdtheta = -upperLeg*sin(thetaL) - lowerLeg*sin(thetaL-zetaL);
		double dyOVERdzeta;
		dyOVERdzeta = -lowerLeg*sin(thetaL-zetaL);

		//if detJ is near zero
		double zero;
		zero = 0.1*upperLeg*lowerLeg;
		if (detJ < zero){
			if (cos(thetaL) >= sin(thetaL))
				dyL = sin(thetaL)/cos(thetaL)*dxL;
			else
				dxL = cos(thetaL)/sin(thetaL)*dyL;

			del_thetaL = -lowerLeg*(cos(thetaL)*dxL + sin(thetaL)*dyL);
			del_zetaL = del_thetaL;
		}
		// if detJ not near zero
		else{
			del_thetaL = (dyOVERdzeta*dxL - dxOVERdzeta*dyL)/detJ;
			del_zetaL = (-dyOVERdtheta * dxL + dxOVERdtheta * dyL) / detJ;
		}
		thetaL = thetaL + del_thetaL;
		zetaL = zetaL + del_zetaL;
		if (zetaL < 0.0) zetaL = 0.0;

		//compute x, y coordinates of the left critical point
		double Lx, Ly;
		Lx = Ltipx - lowerLeg*sin(thetaL - zetaL) - upperLeg*sin(thetaL);
		Ly = Ltipy + lowerLeg*cos(thetaL - zetaL) + upperLeg*cos(thetaL);

		//right knee and tip position
	
		Lkneex = Ltipx - lowerLeg*sin(thetaL-zetaL);
		Lkneey = Ltipy + lowerLeg*cos(thetaL-zetaL);
		Lhipx = Lkneex - upperLeg*sin(thetaL);
		Lhipy = Lkneey + upperLeg*cos(thetaL);	

		//check termination
		if (((Lx-xfL)<=0.05 && (Lx-xfL)>=-0.05) 
			&& ((Ly-yfL)<=0.05 && (Ly-yfL)>=-0.05)) terminateL = 1;

	}

	// adjust right leg position
	thetaR = thetaR + 0.2/N;
	if (thetaR >= thetaMax) thetaR=thetaMax;
	zetaR = zetaR + 0.2/N;
	if (zetaR >= PI/2.0) zetaR=PI/2.0;
	//right hip joint positions
	Rhipx = Lhipx; Rhipy = Lhipy;
	//compute right leg parameters
	Rkneex = Rhipx + upperLeg*sin(thetaR);
	Rkneey = Rhipy - upperLeg*cos(thetaR);
	Rtipx = Rkneex - lowerLeg*sin(zetaR-thetaR);
	Rtipy = Rkneey - lowerLeg*cos(zetaR-thetaR);

	//termnation
	if (terminateL == 1)
		return 1;
	else

	return 0;
}

