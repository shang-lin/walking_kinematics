/*
	Header file state.h
*/

#ifndef _STATE_H_
#define _STATE_H_

/*
	This class controls all the states of walking and contains data 
	members required to draw a walking person.
*/
class States{
	public:
	// constructor
		States(){}
    private:
	// fields
		// Current right leg positions
		double Rhipx, Rhipy, Rhipz;		//right leg hip positions
		double Rkneex, Rkneey, Rkneez;	//right leg knee positions
		double Rtipx, Rtipy, Rtipz;		//right leg tip positions
		double dxR, dyR, dzR;	// small increments of critical points

		// Current left leg positions
		double Lhipx, Lhipy, Lhipz;		//left leg hip positions
		double Lkneex, Lkneey, Lkneez;	//left leg knee positions
		double Ltipx, Ltipy, Ltipz;		//left leg tip positions
		double dxL, dyL, dzL;	// small increments of critical points

		// Initial positions of critical points of right and left leg 
		// during each phase
		double xiR, yiR, ziR;	// right leg initial positions 
		double xfR, yfR, zfR;	// right leg final position
		double xiL, yiL, ziL;	// left leg initial positions 
		double xfL, yfL, zfL;	// left leg final positions

		// Input parameters
		double startingx, startingy, startingz;
		double thetaMax;	// maximum angle to which the upper leg 
							// can be lifted
		double s;			// step size
		double hipSeparation;	// separation of two hip joints
		int N, N0;			      // number of increments
		double upperLeg, lowerLeg;	// length of upper and lower legs
		
		int k;						// counter used in states 2 and 5
		double A2, A1;				//coefficients to control of del_alfa
		

		double thetaR;					// angle of the right upper leg
		double zetaR;					// angle of the right lower leg
		double del_thetaR, del_zetaR;	//increments of right leg angles
		double thetaL;					// angle of the left upper leg
		double zetaL;					// angle of the left lower leg
		double del_thetaL, del_zetaL;	//increments of left leg angles

		double alfa;				// the falling down angle
		double del_alfa;			// incremental change of alfa

	    // control parameters
		int terminateR, terminateL; // used in steps 3 and 6 to terminate 
									// the process
	public:
	// methods
		int setStandingStill(); 	//set the very original standing 
									//still condition
		
		// State 1 methods.
		int set1liftRightLeg(); // Set final position of the tip of 
								// right leg in state 1.
		int liftRightLeg();		// lift the right leg

		// State 2 methods.
		int set2bodyLeanForwardRightLegStepOut();	// Set final position of the tip of right 
								                    // leg in state 2.
		int bodyLeanForwardRightLegStepOut();	    // Body falls forward and the right leg steps.
								                    // forward.
		
		// State 3 methods.
		int set3stretchRightLegPullLeftLegForward();   // set up for the step of 
									                   // pull UpRightLeg
		int stretchRightLegPullLeftLegForward();	   // right leg pull up to straight position
								                       // and left leg move to near vertical
								                       // position
		
		// State 4 methods.
		int set4liftLeftLeg();	//set initial and final conditions for 
								//step4:lift the left leg
		int liftLeftLeg();		//lift the left leg

		// State 5 methods.
		int set5bodyLeanForwardLeftLegStepOut(); // Set up initial and final conditions
								                 // for step 5:bodyLeanForwardLeftLegStepOut
		int bodyLeanForwardLeftLegStepOut();	// Body falls and the left leg steps
								                // forward.

		// State 6 methods.
		int set6stretchLeftLegPullRightLegForward();// Set up for the step of stretchLeftLegPullRightLegForward
		int stretchLeftLegPullRightLegForward();	// left leg stretches to vertical 
								                    // position, and right leg is pulled
								                    // forward

        // Accessors
		double getRhipx(){return Rhipx;}
		double getRhipy(){return Rhipy;}
		double getRhipz(){return Rhipz;}
		double getRkneex(){return Rkneex;}
		double getRkneey(){return Rkneey;}
		double getRkneez(){return Rkneez;}
		double getRtipx(){return Rtipx;}
		double getRtipy(){return Rtipy;}
		double getRtipz(){return Rtipz;}
		double getLhipx(){return Lhipx;}
		double getLhipy(){return Lhipy;}
		double getLhipz(){return Lhipz;}
		double getLkneex(){return Lkneex;}
		double getLkneey(){return Lkneey;}
		double getLkneez(){return Lkneez;}
		double getLtipx(){return Ltipx;}
		double getLtipy(){return Ltipy;}
		double getLtipz(){return Ltipz;}
		double getHipSeparation(){return hipSeparation;}
	    double getThetaR(){return thetaR;}
		double getThetaL(){return thetaL;}
		double getZetaR(){return zetaR;}
		double getZetaL(){return zetaL;}
		double getStartingx(){return startingx;}
		double getUpperLeg(){return upperLeg;}
		double getLowerLeg(){return lowerLeg;}

		// Modifiers.
		void setStartingx(double x){startingx = x;}
		void setStartingy(double y){startingy = y;}
	    void setStartingz(double z){startingz = z;}
	    void setHipSeparation(double sep){hipSeparation = sep;}
	    void setThetaMax(double max){thetaMax = max;}
		void setStepSize(double a){s = a;}
		void setIncrementN(int K){N = K; N0 = N;}
		void setA1(double a){A1 = a;}
		void setA2(double b){A2 = b;}
		void setUpperLeg(double a){upperLeg = a;}
		void setLowerLeg(double a){lowerLeg = a;}

};




#endif 
