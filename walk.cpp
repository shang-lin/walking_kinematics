/* walk.cpp */

// Main file for walk program.

#ifdef WIN32
#include <windows.h>
#endif

#include <GL/glut.h>
#include <GL/glu.h>
#include <GL/gl.h>

#include <iostream>
#include <cmath>

#include "state.h"


#define PI 3.1416

using namespace std;

// Global definitions.
int xresol=500, yresol=500;    // resolution of the screen
States robotStates;	       // instantiate a States object
bool threeDmode = true;     // Toggle between stick person and 3D-person
bool legs_only_mode = false; // Toggle between drawing only legs and drawing
                             // entire body.

bool pause = false;  // Indicates whether animation is paused.
bool rightmode = true;  // Indicates whether walking left or right.

/* The variable legState indicates the current state.
	legState = 0	both the right and the left legs are 
					vertically stretched out, i.e. in a standing
					still position
	legState = 1	lifting the rightleg
	legState = 2	body falling down, right leg stepping out
	legState = 3	right leg pulling up
	legState = 4	lifting the left leg
	legState = 5	body falling down, left leg stepping out
	legState = 6	left leg pulling up
*/
int legState = 0;
float camerax, cameray, cameraz;  // camera position
float aimx, aimy, aimz;           // camera aiming point

/*
	Initialization method. Sets clearing color and shade
	model.
*/
void init(void){
	glClearColor(1.0, 1.0, 1.0, 0.0);
	glShadeModel(GL_FLAT);
	glEnable(GL_DEPTH_TEST);
}

/*
    Resizes the window. Sets up perspective projection.
	args: int w, h - Width and height of the window.
*/
void reshape(int w, int h){
	glViewport(0, 0, (GLsizei) w, (GLsizei) h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glViewport(0, 0, w, h);

	gluPerspective(85.0, (GLfloat) w/(GLfloat) h, 1.0, 50.0);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

}


//  This method draws a cube using six polygons.
//  args: double size - Distance between two opposite faces of the cube.
void drawCube(double size) {

    // Front face.
    glBegin(GL_POLYGON);
    glNormal3f(0.0, 0.0, 1.0);
    glVertex3f(-0.5 * size, -0.5 * size, 0.5 * size);
    glVertex3f(0.5 * size, -0.5 * size, 0.5 * size);
    glVertex3f(0.5 * size, 0.5 * size, 0.5 * size);
    glVertex3f(-0.5 * size, 0.5 * size, 0.5 * size);
    glEnd();

    glBegin(GL_POLYGON);
	glNormal3f(0.0, 0.0, -1.0);
	glVertex3f(-0.5 * size, -0.5 * size, -0.5 * size);
	glVertex3f(-0.5 * size, 0.5 * size, -0.5 * size);
	glVertex3f(0.5 * size, 0.5 * size, -0.5 * size);
	glVertex3f(0.5 * size, -0.5 * size, -0.5 * size);
    glEnd();

    glBegin(GL_POLYGON);
	glNormal3f(1.0, 0.0, 0.0);
	glVertex3f(0.5 * size, -0.5 * size, 0.5 * size);
	glVertex3f(0.5 * size, -0.5 * size, -0.5 * size);
	glVertex3f(0.5 * size, 0.5 * size, -0.5 * size);
	glVertex3f(0.5 * size, 0.5 * size, 0.5 * size);
    glEnd();

    glBegin(GL_POLYGON);
	glNormal3f(-1.0, 0.0, 0.0);
	glVertex3f(-0.5 * size, -0.5 * size, 0.5 * size);
	glVertex3f(-0.5 * size, 0.5 * size, 0.5 * size);
	glVertex3f(-0.5 * size, 0.5 * size, -0.5 * size);
	glVertex3f(-0.5 * size, -0.5 * size, -0.5 * size);
    glEnd();

    glBegin(GL_POLYGON);
	glNormal3f(0.0, 1.0, 0.0);
	glVertex3f(-0.5 * size, 0.5 * size, 0.5 * size);
	glVertex3f(0.5 * size, 0.5 * size, 0.5 * size);
	glVertex3f(0.5 * size, 0.5 * size, -0.5 * size);
	glVertex3f(-0.5 * size, 0.5 * size, -0.5 * size);
    glEnd();

    glBegin(GL_POLYGON);
	glNormal3f(0.0, -1.0, 0.0);
	glVertex3f(-0.5 * size, -0.5 * size, 0.5 * size);
	glVertex3f(-0.5 * size, -0.5 * size, -0.5 * size);
	glVertex3f(0.5 * size, -0.5 * size, -0.5 * size);
	glVertex3f(0.5 * size, -0.5 * size, 0.5 * size);
    glEnd();
}


/*
	function draw leg
	Draws a person with legs, knees, torso, and head and arms.
*/
int drawPerson(){
	double Rhipx, Rhipy, Rhipz, Lhipx, Lhipy, Lhipz;
    double Rkneex, Rkneey, Rkneez, Lkneex, Lkneey, Lkneez;
    double Rtipx, Rtipy, Rtipz, Ltipx, Ltipy, Ltipz;
	Rhipx = robotStates.getRhipx();
	Rhipy = robotStates.getRhipy();
	Rhipz = robotStates.getRhipz();
	Lhipx = robotStates.getLhipx();
	Lhipy = robotStates.getLhipy();
	Lhipz = robotStates.getLhipz();
	Rkneex = robotStates.getRkneex();
	Rkneey = robotStates.getRkneey();
	Rkneez = robotStates.getRkneez();
	Lkneex = robotStates.getLkneex();
	Lkneey = robotStates.getLkneey();
	Lkneez = robotStates.getLkneez();
	Rtipx = robotStates.getRtipx();
	Rtipy = robotStates.getRtipy();
	Rtipz = robotStates.getRtipz();
	Ltipx = robotStates.getLtipx();
	Ltipy = robotStates.getLtipy();
	Ltipz = robotStates.getLtipz();
	double thetaR, zetaR, thetaL, zetaL;
	thetaR = robotStates.getThetaR();
    thetaL = robotStates.getThetaL();
	zetaR = robotStates.getZetaR();
	zetaL = robotStates.getZetaL();
	double upperLeg, lowerLeg;
	upperLeg = robotStates.getUpperLeg();
    lowerLeg = robotStates.getLowerLeg();

    double hipSeparation;
	hipSeparation = robotStates.getHipSeparation();

	// Draw body.
	double cx, cy, cz;
	cz = (Rhipz + Lhipz)/2;
	cx = Rhipx; cy = Rhipy;

	if (!legs_only_mode) {
		// Draw a stick body if 3D mode is not on.
		if (!threeDmode){
		glBegin(GL_LINES);
			glColor3f(1.0, 0.0, 0.0);
		    glVertex3f(cx, cy, cz);
		    glVertex3f(cx, cy+2.5, cz);
	    glEnd();
	}
	else{
		// Draw a 3D body if 3D mode is on.
	    glPushMatrix();
			// glColor3f(1.0, 1.0, 1.0);
		    glColor3f(0.4, 0.0, 0.6);
			glTranslatef(cx, cy+1.0, cz);
		    glScalef(0.5, 1.0/hipSeparation, 1.97);
			drawCube(1.98*hipSeparation);
	    glPopMatrix();
	}
	// Draw head.
	glPushMatrix();
	glColor3f(1.0, 1.0, 0.0);
		glTranslatef(cx, cy+2.5, cz);
		glutSolidSphere(0.4, 20, 50);
	glPopMatrix();

	// Draw shoulder.
	
	// Draw a stick shoulder.
	if (!threeDmode) {
	    glBegin(GL_LINES);
	    glColor3f(1.0, 1.0, 1.0);
		    glVertex3f(cx, cy+1.8, cz - 1.98*hipSeparation);
		    glVertex3f(cx, cy+1.8, cz + 1.98*hipSeparation);
	    glEnd();
	}
	// Draw arms.
	float armAngle;
	if (legState == 0)
		armAngle = 0.0;
	else if(legState==1 || legState==2 || legState==3)
		armAngle = thetaR*180.0/PI*0.3;
	else if(legState==4 || legState==5 || legState==6)
		armAngle = -thetaL*180.0/PI*0.3;
	
	// Draw right arm.
	double rarmx, rarmy, rarmz;
	rarmx = cx; rarmy = cy+1.8; rarmz = cz+2.01*hipSeparation;
	glPushMatrix();
	    glTranslatef(rarmx, rarmy, rarmz);
	    glRotatef(armAngle, 0.0, 0.0, 1.0);
	    glTranslatef(-rarmx, -rarmy, -rarmz);
		
		if (!threeDmode) {
			// Draw stick right arm.
		    glBegin(GL_LINES);
		        glColor3f(1.0, 0.0, 0.0);
			    glVertex3f(rarmx, rarmy, rarmz);
			    glVertex3f(rarmx, rarmy-1.6, rarmz);
		    glEnd();
		}
		// Draw 3D right arm
		else{
			glPushMatrix();
				glColor3f(1.0, 0.0, 0.0);
				glTranslatef(rarmx, rarmy-0.9, rarmz);
				glScalef(0.1, 1.8, 0.1);
				drawCube(1.0);
		    glPopMatrix();
		}
	glPopMatrix();
	
	// Draw left arm
	double larmx, larmy, larmz;
	larmx = cx, larmy = cy+1.8; larmz = cz-2.01*hipSeparation;
	glPushMatrix();
	    glTranslatef(larmx, larmy, larmz);
	    glRotatef(-armAngle, 0.0, 0.0, 1.0);
	    glTranslatef(-larmx, -larmy, -larmz);
		
		// Draw stick left arm
		if (!threeDmode) {
		glBegin(GL_LINES);
		glColor3f(1.0, 0.0, 0.0);
			glVertex3f(larmx, larmy, larmz);
			glVertex3f(larmx, larmy-1.6, larmz);
		glEnd();
		}
		//draw 3D left arm
		else{
		glPushMatrix();
			glColor3f(1.0, 0.0, 0.0);
			glTranslatef(larmx, larmy-0.9, larmz);
			glScalef(0.1, 1.8, 0.1);
		    drawCube(1.0);
		    glPopMatrix();
		}
	glPopMatrix();
	
	}
	
	if (!threeDmode){
		// Draw legs as stick
	    glPushMatrix();
		// Draw hip bar for stick version.
		glBegin(GL_LINES);
		    glColor3f(1.0, 1.0, 1.0);
			glVertex3f(Rhipx, Rhipy, Rhipz);
			glVertex3f(Lhipx, Lhipy, Lhipz);
	    glEnd();
		// Draw right upper leg
	    glBegin(GL_LINES);
			glColor3f(1.0, 1.0, 0.0);
			glVertex3f(Rhipx, Rhipy, Rhipz);
			glVertex3f(Rkneex, Rkneey, Rkneez);
		glEnd();
		// Draw right lower leg.
		glBegin(GL_LINES);
			glColor3f(1.0, 0.0, 1.0);
			glVertex3f(Rkneex, Rkneey, Rkneez);
			glVertex3f(Rtipx, Rtipy, Rtipz);
		glEnd();
		// Draw left upper leg
		glBegin(GL_LINES);
			glColor3f(0.0, 0.5, 1.0);
			glVertex3f(Lhipx, Lhipy, Lhipz);
			glVertex3f(Lkneex, Lkneey, Lkneez);
		glEnd();
		// Draw left lower leg
		glBegin(GL_LINES);
			glColor3f(0.0, 1.0, 0.5);
			glVertex3f(Lkneex, Lkneey, Lkneez);
			glVertex3f(Ltipx, Ltipy, Ltipz);
		glEnd();
	    glPopMatrix();
	}

	// Draw legs as tubes
	else{
	    //draw right leg
	    glPushMatrix();
		glColor3f(1.0, 1.0, 0.0);
		    glTranslatef(Rhipx, Rhipy, Rhipz);
		    glRotatef(thetaR*180.0/PI, 0.0, 0.0, 1.0);
		    glPushMatrix();
			glTranslatef(0.0, -0.5*upperLeg, 0.0);
			glScalef(0.1*upperLeg, upperLeg, 0.1*upperLeg);
			drawCube(1.0);
		    glPopMatrix();
	    // Draw lower right leg
		glPushMatrix();
			glColor3f(1.0, 0.0, 1.0);
			glTranslatef(0.0, -upperLeg, 0.0);
			glRotatef(zetaR*180.0/PI, 0.0, 0.0, -1.0);
			glTranslatef(0.0, -0.5*lowerLeg, 0.0);
			glScalef(0.1*lowerLeg, lowerLeg, 0.1*lowerLeg);
		drawCube(1.0);
		    glPopMatrix();
	    glPopMatrix();
	    // Draw left leg
	    glPushMatrix();
		// Draw upper leftt leg
		glColor3f(0.0, 0.5, 1.0);
		    glTranslatef(Lhipx, Lhipy, Lhipz);
		    glRotatef(thetaL*180.0/PI, 0.0, 0.0, 1.0);
		    glPushMatrix();
			glTranslatef(0.0, -0.5*upperLeg, 0.0);
			glScalef(0.1*upperLeg, upperLeg, 0.1*upperLeg);
			drawCube(1.0);
		    glPopMatrix();
	    // Draw lower left leg
		glPushMatrix();
			glColor3f(0.0, 1.0, 0.5);
			glTranslatef(0.0, -upperLeg, 0.0);
			glRotatef(zetaL*180.0/PI, 0.0, 0.0, -1.0);
			glTranslatef(0.0, -0.5*lowerLeg, 0.0);
			glScalef(0.1*lowerLeg, lowerLeg, 0.1*lowerLeg);
			drawCube(1.0);
		    glPopMatrix();
	    glPopMatrix();
    }
	return 0;
}

/*
	Rendering routine. Calls methods to calculate states.
*/  
void display(){
	reshape(xresol, yresol);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    gluLookAt(camerax, cameray, cameraz, aimx, aimy, aimz, 0.0f, 1.0f, 0.0f);
	
	// Only do calculations if animation is not paused.
	if (!pause) {
		// In the standing still state
		if (legState == 0){
			legState = 1;
			// Set final and initial positions of the tips of legs in
			// State 1.
			robotStates.set1liftRightLeg();
		}

		// State 1: lifting the right leg
		else if (legState == 1){
		
			// Calculate consecutive right leg angles
			if(robotStates.liftRightLeg()== 1){
				legState = 2;
				// Set final and initial positions of the tips of legs 
				// in state 2.
				robotStates.set2bodyLeanForwardRightLegStepOut();
			}
			drawPerson();
		}
		// State 2: Leaning forward while right leg steps forward
		else if (legState == 2){
			// Calculate leaning and stepping out of right leg.
			if (robotStates.bodyLeanForwardRightLegStepOut()==1){
				legState = 3;
				robotStates.set3stretchRightLegPullLeftLegForward();
			}
			drawPerson();
		}

		// State 3: Stretch the right leg to vertical position
		//			while left leg is pulled forward
		else if (legState == 3){
			if (robotStates.stretchRightLegPullLeftLegForward() == 1){
				legState = 4;
				robotStates.set4liftLeftLeg();
			}
			drawPerson();
		}
		// State 4: lift the left leg to predefined position
		else if (legState == 4){
			if (robotStates.liftLeftLeg() == 1){
				legState = 5;
				robotStates.set5bodyLeanForwardLeftLegStepOut();
			}
			drawPerson();
		}
		// State 5: leaning forward and the left leg steps forward
		else if (legState == 5){
			if (robotStates.bodyLeanForwardLeftLegStepOut() == 1){
				legState = 6;
				robotStates.set6stretchLeftLegPullRightLegForward();
			}
			drawPerson();
		}
		// State 6: stretch the left leg to vertical position
		//			while right leg is pulled forward
		else if (legState == 6){
			if (robotStates.stretchLeftLegPullRightLegForward() == 1){
				legState = 1;
				robotStates.set1liftRightLeg();
			}
			drawPerson();
		}
	}
	else {
		drawPerson();
	}
	
    glutSwapBuffers();
}


/*
	Changes walking direction from left to right and 
	vice versa.
*/
void changeDirection() {
	rightmode = !rightmode;  // Toggle direction.

	// Return to standing still.
	legState = 0;
	
	// Reset position and camera.
	double cPos;
	cPos = robotStates.getRtipx();
	robotStates.setStartingx(cPos);
	robotStates.setStandingStill();
	camerax = 2.0*(cPos - aimx) + aimx;
	aimx = camerax;
	cameraz = -cameraz;
}

/*
   This function handles key board input from
   non-ASCII keys. F1, F2, and left and right 
   arrows are handled.

   args: int key - key pressed.
         int x, y - coordinates of mouse.
*/
void handleKeystroke(int key, int x, int y) {
    const GLfloat increment = PI/100.0;
	
    // Rotate the direction of walking.
	float tmp;
	tmp = sqrt(pow(camerax, 2) + pow(cameraz, 2));
	
	// Toggle between stick person or 3D-person.
	if (key == GLUT_KEY_F1){
		threeDmode = !threeDmode;
	}

	// Toggle between drawing only legs and full body.
	else if (key == GLUT_KEY_F2) {
		legs_only_mode = !legs_only_mode;
	}

	// Toggle direction.
	else if ((key == GLUT_KEY_LEFT) && rightmode) {
		changeDirection();
	}
	else if ((key == GLUT_KEY_RIGHT) && !rightmode) {
		changeDirection();
	}
	
	// Redraw window.
    glutPostRedisplay();
}




/*
	This function is called whenever an ASCII key on the 
	keyboard is pressed.
	'p' is the only key handled.
*/
void handleASCIIkey(unsigned char key, int x, int y) {
	// Toggle pause mode.
	if (key == 'p') {
		pause = !pause;
	}
}

// Main function.
int main(int argc, char **argv){
	// Set initial camera position.
	camerax = 0.0; cameray = 0.0; cameraz = 7.0;
	aimx = 0.0; aimy = 0.0; aimz = 0.0;
	
	if (argc == 1) {
		// Initialize robotStates.
		robotStates.setUpperLeg(1.0);
		robotStates.setLowerLeg(1.0);
		robotStates.setHipSeparation(0.2);
		robotStates.setThetaMax(PI / 5);
		robotStates.setStartingx(-3.0);
		robotStates.setStartingy(0.0);
		robotStates.setStartingz(0.0);
		robotStates.setIncrementN(150);
		robotStates.setStepSize(0.6);
	}
	else if (argc == 2) {
		// TODO: Load input file.
	}
	else{
		cout<<"usage: walk.exe [input-file]"<<endl;
		return 1;
	}
	robotStates.setA2(0.1); robotStates.setA1(0.2);
	

	// Set the initial position, standing still.
	robotStates.setStandingStill();

    // glut
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB);
    glutInitWindowSize(xresol, yresol);
    glutInitWindowPosition(100, 100);
    glutCreateWindow("Walking Animation");
    init();
    glutReshapeFunc(reshape);
    glutDisplayFunc(display);
    glutIdleFunc(display);
	glutKeyboardFunc(handleASCIIkey);
    glutSpecialFunc(handleKeystroke);
    glutMainLoop();

    return 0;

}

