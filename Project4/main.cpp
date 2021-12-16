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



//==================================
// INPUT CAN BE MODIFY HERE!!!
// 2D flocking system
// ==================================

//allocate memory for 10 boids
int boidnum = 20;
float boid[20][15] = {0};//boid matrix
float bv[20][2] = {0};//boid velocity on x and y
float bf[20][2] = {0};//force on boid
float rs = 2; // radius of seperation detection. 
float rc = 5; // radius of cohesion
float rv = 3; // radius of velocity match
float dt = 0.01f; //dt is the spcing to used in the animation

//rotation matrix
float M[16] = { 0 };
float Mf[16] = { 0 };

// screen size
int g_screenWidth = 0;
int g_screenHeight = 0;

// frame index
int g_frameIndex = 0;

void drawfood() {

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
	glutSolidCube(0.5);
	glPopMatrix();

}

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
	boid[bn][12] = boid[bn][12] + bv[bn][0] * 0.01;
	boid[bn][13] = boid[bn][13] + bv[bn][1] * 0.01;



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
	glutSolidSphere(0.3, 20, 20);
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
				//printf("sep");
				//the closer two boid to each other, the more force will apply on them
				bf[bn][0] +=  8 *(boid[bn][12] - boid[i][12] / (boid[bn][12] - boid[i][12]) * (boid[bn][12] - boid[i][12])) ;
				bf[bn][1] +=  8 *(boid[bn][13] - boid[i][13] / (boid[bn][13] - boid[i][13]) * (boid[bn][13] - boid[i][13])) ;
				
			}
		}

	}
}

void Velocitymatch(int bn) {
	//boids'velocity will move towards the average velocity in a specifix radius
	float vxsum = 0;
	float vysum = 0;
	float vx = 0;
	float vy = 0;
	int num = 0;
	for (int i = 0; i < boidnum; i++) {
		float distx = boid[bn][12] - boid[i][12];
		float disty = boid[bn][13] - boid[i][13];
		float distxy = sqrtf(distx * distx + disty * disty);
		if (distxy < rv) {
			vxsum += bv[i][0];
			vysum += bv[i][1];
			num++;
		}
		//printf("vxsum %f, vysum%f, num %d", vxsum, vysum, num);
	}
	vx = vxsum / num;
	vy = vysum / num;
	//printf("vx: %f, bv[%d][0]: %f", vx, bn, bv[bn][0]);
	bf[bn][0] += (vx - bv[bn][0]);
	bf[bn][1] += (vy - bv[bn][1]);
}

void cohesion(int bn) {
	//a boid show move toward the average postion of local boids.
	//local boids: boids in radius cohesion 
	float targetxt = 0;//target x total
	float targetyt = 0;//target y total
	int num = 0;//boids num in cohesion radius

	for (int i = 0; i < boidnum; i++) {
		float distx = boid[bn][12] - boid[i][12];
		float disty = boid[bn][13] - boid[i][13];
		float distxy = sqrtf(distx * distx + disty * disty);
		if (distxy < rc) {
			targetxt += boid[i][12]; //if the boid is in the range, add it's location to the avrage x;
			targetyt += boid[i][13]; //.. y...
			num++;
		}
	}

	float targetx = targetxt / num;
	float targety = targetyt / num;
	//printf("targetx: %f cureentx:%f \n", targetx, boid[bn][12]);
	//the more far away those boid, ther more they will be attract to the center
	bf[bn][0] += ( targetx - boid[bn][12])/2;
	bf[bn][1] +=  (targety - boid[bn][13])/2;


}

//================================
// init
//================================
void init(void) {
	// init something before main loop...
		//init boids location;
	
	for (int i = 0; i < boidnum; i++) {
		boid[i][12] = rand() % 19 + (-9); //limit by windows width x in range(-9, 9)
		boid[i][13] = rand() % 15 + (-7);//limit by windows height y in range(-7, 7);
		boid[i][14] = -30;//make it 2d
		bv[i][0] = 0;//0;
			//rand() % 11 + (-5); //limit speed on x in range (-5, 5)
		bv[i][1] = 0; //0;
			//rand() % 11 + (-5); //limit speed on y in range (-5, 5)
	}
	/***
	boid[0][12] = 0;
	boid[1][12] = 2;
	boid[0][13] = 3;
	boid[1][13] = 2;
	boid[0][14] = -30;
	boid[1][14] = -30;
	bv[0][0] = 0;
	bv[1][0] = 0;
	bv[0][1] = 0;
	bv[1][1] = 0;
	***/


	
}

//================================
// update
//================================
void update(void) {
	for (int i = 0; i < boidnum; i++) {
		bf[i][0] = 0;
		bf[i][1] = 0;
	}
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
	drawfood();
	for (int i = 0; i < boidnum; i++) {
		//printf("before x, bf[0][0] : %f \n", bf[0][0]);
		cohesion(i);
		separate(i);
		Velocitymatch(i);
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
