// standard
#include <assert.h>
#include <math.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#define PI 3.14159265
// glut
#include <GL/glut.h>

//================================
// global variables
//================================

//option
int spline_opt, //1 for catmull-rom, 2 for b spline
rotation_opt; // 1 for fixed angle, 2 for quaternion


//==================================
// INPUT CAN BE MODIFIED HERE!!!
// CONTROL POINTS INSTRUCTION:
// qaray is used for stroing control points while using quaternion
//       the format is {x, y, z, rotation degree, xaxis, yaxis, zaxis}
// fixaray is used for stroing control points while using quaternion
//       the format is {x, y, z, rotation degree on xaxis, rotation degree on yaxis,rotation degree on zaxis}
// point_num is the number of contritical points
//       if you would like to add or remove points from qaray or fixaray, please remember to modify this value
// 
// 2D flocking system
// ==================================

//allocate memory for 10 boids
int boidnum = 10;
float boid[10][15] = {0};//boid matrix
float bv[10][2] = {0};//boid velocity on x and y
float bf[10][2] = {0};//force on boid
float rs = 3; // radius of seperation detection. 
float rc = 11; // radius of cohesion

float point_num = 6;
//geometric point for quaternion
float qaray[6][7] = { {-8.0, -6.0, -25.0, 0, 1, 0, 0}, {-2.0, -5.0, -17.0, 1, 0, 1, 0}, {5.5, -2.0, -13.0, 1, 0, 0, 1},{2.0, 3.0, -15.0, 1, 1, 0, 0},
	{-3.5, 6.5, -20.0, 1, 0, 1, 0}, {10.0, 6.0, -15.0, 0, 0, 1, 0} };

//geometric point for fixed angle
float fixaray[6][6] = { {-8.0, -6.0, -25.0, 30, 0, 80}, {-2.0, -5.0, -17.0, 0, 180, 0}, {5.5, -2.0, -13.0, 0, 50, 90},{2.0, 3.0, -15.0, 60, 40, 0},
	{-3.5, 6.5, -20.0, 180, 0, 90}, {10.0, 6.0, -15.0, 0, 90, 30} };

float dt = 0.01; //dt is the spcing to used in the animation

//rotation matrix
float M[16] = { 0 };

//t is the value in both catmull-rom and b spline function.
float t = 0.0;
float x, y, z; //current location for the object

//Matrix for Calculating Catmull Rom, using 1/2 as 'a'
float mCR[4][4] = { {-0.5, 1.5, -1.5, 0.5}, {1.0, -2.5, 2.0, -0.5},
	{-0.5, 0.0, 0.5, 0.0}, {0.0, 1.0, 0.0, 0.0} };
//Matrix for Calculating B-spline
float mB[4][4] = { {-1.0, 3.0, -3.0, 1.0}, {3.0, -6.0, 3.0, 0.0},
	{-3.0, 0.0, 3.0, 0.0}, {1.0, 4.0, 1.0, 0.0} };

// screen size
int g_screenWidth = 0;
int g_screenHeight = 0;

// frame index
int g_frameIndex = 0;

void drawBoids(int bn) {

	//update location;
	//calculate acceleration a = F/m 
	//assume all boid have weight that = 1;
	float ax = bf[bn][0] / 1;
	float ay = bf[bn][1] / 1;

	//calculate velocity
	bv[bn][0] = bv[bn][0] + ax * 0.01;
	bv[bn][1] = bv[bn][1] + ay * 0.01;

	//update location
	float x = boid[bn][12] + bv[bn][0] * 0.01;
	float y = boid[bn][13] + bv[bn][1] * 0.01;

	boid[bn][12] = x;
	boid[bn][13] = y;


	glPushMatrix();
	M[0] = 1.0f;
	M[5] = 1.0f;
	M[10] = 1.0f;
	M[15] = 1.0f;
	M[12] = boid[bn][12];
	M[13] = boid[bn][13];
	M[14] = boid[bn][14];
	glMultMatrixf(M);
	//glTranslatef(x, y, z);
	glutSolidSphere(0.5, 20, 20);
	glPopMatrix();
}

void separate(int bn) {
	// if a boid is 2 unit close to another, a force will compress it to go away
	for (int i = 0; i < boidnum; i++) {
		if (i != bn) {
			float distx = boid[bn][12] - boid[i][12];
			float disty = boid[bn][13] - boid[i][13];
			float distxy = sqrtf(distx * distx + disty * disty);
			if (distxy < rs) {
				//add force on x axis
				if (boid[bn][12] > boid[i][12]) {
					bf[bn][0] =0;
					//change velocity according to distxy
					bv[bn][0] += 0.5 ;
				}
				else {
					bf[bn][0] = 0;
					//bf[bn][0] -= 0.5;
					bv[bn][0] -= 0.5;
				}
				//add force on y axis
				if (boid[bn][13] > boid[i][13]) {
					bf[bn][1] = 0;
					//bf[bn][1] += 0.5;
					bv[bn][1] += 0.5;

				}
				else {
					bf[bn][1] = 0;
					//bf[bn][1] -= 0.5;
					bv[bn][1] -= 0.5;
				}
			}
		}

	}
}

void cohesion(int bn) {
	//a boid show move toward the average postion of local boids.
	//local boids: boids in radius cohesion 
	float targetxt = boid[bn][12];//target x total
	float targetyt = boid[bn][13];//target y total
	int num = 0;//boids num in cohesion radius

	for (int i = 0; i < boidnum; i++) {
		float distx = boid[bn][12] - boid[i][12];
		float disty = boid[bn][13] - boid[i][13];
		float distxy = sqrtf(distx * distx + disty * disty);
		if (distxy < rc) {
			//std::cout << i;
			targetxt += boid[i][12]; //if the boid is in the range, add it's location to the avrage x;
			targetyt += boid[i][13]; //.. y...
			num++;
		}
	}

	float targetx = targetxt / num;
	float targety = targetyt / num;

	if (boid[bn][12] > targetx) {
		//find velocity target
		float vx = (boid[bn][12] - targetx)/10;
		float ax = abs(vx - bv[bn][0]) / 10;
		if (abs(bf[bn][0]) < 10) {
			bf[bn][0] -= ax;
		}

	}
	else {
		float vx = (boid[bn][12] - targetx) / 10;
		float ax = abs(vx - bv[bn][0]) / 10;
		if (abs(bf[bn][0]) < 10) {
			bf[bn][0] += ax;
		}
	}
	if (boid[bn][13] > targety) {
		float vy = (boid[bn][13] - targety) / 10;
		float ay = abs(vy - bv[bn][1]) / 10;
		if (abs(bf[bn][1]) < 10) {
			bf[bn][1] -= ay;
		}
	}
	else {
		float vy = (boid[bn][13] - targety) / 10;
		float ay = abs(vy - bv[bn][1]) / 10;
		if (abs(bf[bn][1]) < 10) {
			bf[bn][1] += ay;
		}
	}
}

//================================
// init
//================================
void init(void) {
	// init something before main loop...
		//init 10 boids location;
	for (int i = 0; i < boidnum; i++) {
		boid[i][12] = rand() % 19 + (-9); //limit by windows width x in range(-9, 9)
		boid[i][13] = rand() % 15 + (-7);//limit by windows height y in range(-7, 7);
		boid[i][14] = -30;//make it 2d
		bv[i][0] = 0;
			//rand() % 11 + (-5); //limit speed on x in range (-5, 5)
		bv[i][1] = 0;
			//rand() % 11 + (-5); //limit speed on y in range (-5, 5)
		//std::cout << i;
	}
	
}

//================================
// update
//================================
void update(void) {
	/***
	// catmull-rom + fixed
	if (spline_opt == 1 && rotation_opt == 1) {
		//if get input 1, choose catmull spline, increase t
		if (t < point_num - 3) {
			float curgeo[6];
			for (int i = 0; i < 6; i++) {
				float G[4] = { fixaray[(int)t][i], fixaray[(int)t + 1][i] ,fixaray[(int)t + 2][i] ,fixaray[(int)t + 3][i] };
				curgeo[i] = matrixTMG(mCR, G);
			}
			x = curgeo[0];
			y = curgeo[1];
			z = curgeo[2];
			fixedangle(curgeo[3], curgeo[4], curgeo[5]);
			t = t + dt;
		}

	}
	//catmull-rom + quaternion
	else if (spline_opt == 1 && rotation_opt == 2) {
		//if get input 1, choose catmull spline, increase t
		if (t < point_num - 3) {
			float curgeo[7];
			for (int i = 0; i < 7; i++) {
				float G[4] = { qaray[(int)t][i], qaray[(int)t + 1][i] ,qaray[(int)t + 2][i] ,qaray[(int)t + 3][i] };
				curgeo[i] = matrixTMG(mCR, G);
			}
			x = curgeo[0];
			y = curgeo[1];
			z = curgeo[2];

			//normalize quanternion
			float wxyz_2 = curgeo[3] * curgeo[3] + curgeo[4] * curgeo[4] + curgeo[5] * curgeo[5] + curgeo[6] * curgeo[6];
			if (wxyz_2 != 1) {
				float wxyz = sqrt(fabs(wxyz_2));
				for (int i = 3; i < 7; i++) {
					curgeo[i] = curgeo[i] / wxyz;
				}
			}
			//get quanternion rotation matrix
			quaternion(curgeo[3], curgeo[4], curgeo[5], curgeo[6]);
			t = t + dt;
		}


	}
	//Bspline + fixed angel
	else if (spline_opt == 2 && rotation_opt == 1) {
		if (t < point_num - 3) {
			float curgeo[6];
			for (int i = 0; i < 6; i++) {
				float G[4] = { fixaray[(int)t][i], fixaray[(int)t + 1][i] ,fixaray[(int)t + 2][i] ,fixaray[(int)t + 3][i] };
				curgeo[i] = matrixTMG(mB, G);
			}
			x = curgeo[0];
			y = curgeo[1];
			z = curgeo[2];
			fixedangle(curgeo[3], curgeo[4], curgeo[5]);
			t = t + dt;
		}
	}
	//Bspline + quaternion
	else if (spline_opt == 2 && rotation_opt == 2) {
		//get all geometric point using b-spline
		if (t < point_num - 3) {
			float curgeo[7];
			for (int i = 0; i < 7; i++) {
				float G[4] = { qaray[(int)t][i], qaray[(int)t + 1][i] ,qaray[(int)t + 2][i] ,qaray[(int)t + 3][i] };
				curgeo[i] = matrixTMG(mB, G);
			}
			x = curgeo[0];
			y = curgeo[1];
			z = curgeo[2];

			//normalize quanternion
			float wxyz_2 = curgeo[3] * curgeo[3] + curgeo[4] * curgeo[4] + curgeo[5] * curgeo[5] + curgeo[6] * curgeo[6];
			if (wxyz_2 != 1) {
				float wxyz = sqrt(fabs(wxyz_2));
				for (int i = 3; i < 7; i++) {
					curgeo[i] = curgeo[i] / wxyz;
				}
			}
			//get quanternion rotation matrix
			quaternion(curgeo[3], curgeo[4], curgeo[5], curgeo[6]);
			t = t + dt;
		}
	}
	***/
}

//===============================
// teapot display
// ==============================
void displayTeapot() {
	//if (rotation_opt == 1) {
	//	glRotated(ax, 1.0, 0.0, 0.0);
	//	glRotated(ay, 0.0, 1.0, 0.0);
	//	glRotated(az, 0.0, 0.0, 1.0);
	//}
	glLoadMatrixf(M);
	// render objects
	glutSolidTeapot(0.3);
}
//================================
// render
//================================
void render(void) {
	// clear buffer
	glClearColor(0.0, 0.0, 0.0, 0.0);
	glClearDepth(1.0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// render state
	glEnable(GL_DEPTH_TEST);
	glShadeModel(GL_SMOOTH);

	// enable lighting
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);

	// light source attributes
	GLfloat LightAmbient[] = { 0.4f, 0.4f, 0.4f, 1.0f };
	GLfloat LightDiffuse[] = { 0.3f, 0.3f, 0.3f, 1.0f };
	GLfloat LightSpecular[] = { 0.4f, 0.4f, 0.4f, 1.0f };
	GLfloat LightPosition[] = { 5.0f, 5.0f, 5.0f, 1.0f };

	glLightfv(GL_LIGHT0, GL_AMBIENT, LightAmbient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, LightDiffuse);
	glLightfv(GL_LIGHT0, GL_SPECULAR, LightSpecular);
	glLightfv(GL_LIGHT0, GL_POSITION, LightPosition);

	// surface material attributes
	GLfloat material_Ka[] = { 0.11f, 0.06f, 0.11f, 1.0f };
	GLfloat material_Kd[] = { 0.43f, 0.47f, 0.54f, 1.0f };
	GLfloat material_Ks[] = { 0.33f, 0.33f, 0.52f, 1.0f };
	GLfloat material_Ke[] = { 0.1f , 0.0f , 0.1f , 1.0f };
	GLfloat material_Se = 10;

	glMaterialfv(GL_FRONT, GL_AMBIENT, material_Ka);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, material_Kd);
	glMaterialfv(GL_FRONT, GL_SPECULAR, material_Ks);
	glMaterialfv(GL_FRONT, GL_EMISSION, material_Ke);
	glMaterialf(GL_FRONT, GL_SHININESS, material_Se);

	// modelview matrix
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	//glTranslatef(0.0, 0.0, -20);
	//displayTeapot();
	for (int i = 0; i < boidnum; i++) {
		separate(i);
		cohesion(i);
		drawBoids(i);
	}
	// disable lighting
	glDisable(GL_LIGHT0);
	glDisable(GL_LIGHTING);

	// swap back and front buffers
	glutSwapBuffers();



}

//================================
// keyboard input
//================================
void keyboard(unsigned char key, int x, int y) {
}

//================================
// reshape : update viewport and projection matrix when the window is resized
//================================
void reshape(int w, int h) {
	// screen size
	g_screenWidth = w;
	g_screenHeight = h;

	// viewport
	glViewport(0, 0, (GLsizei)w, (GLsizei)h);

	// projection matrix
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(45.0, (GLfloat)w / (GLfloat)h, 1.0, 500);
}


//================================
// timer : triggered every 16ms ( about 60 frames per second )
//================================
void timer(int value) {
	// increase frame index
	g_frameIndex++;

	update();

	// render
	glutPostRedisplay();

	// reset timer
	// 16 ms per frame ( about 60 frames per second )
	glutTimerFunc(16, timer, 0);
}

//================================
// main
//================================
int main(int argc, char** argv) {

	// create opengL window
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
	glutInitWindowSize(1600, 1200);
	glutInitWindowPosition(100, 100);
	glutCreateWindow(argv[0]);

	// init
	init();

	// set callback functions
	glutDisplayFunc(render);
	glutReshapeFunc(reshape);
	glutKeyboardFunc(keyboard);
	glutTimerFunc(16, timer, 0);

	// main loop
	glutMainLoop();

	return 0;
}
