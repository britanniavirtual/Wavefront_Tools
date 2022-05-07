#include "JointHeader.h"

#pragma comment(lib, "glew32.lib")

const int width = 1024, height = 1024;
const int GRID_SIZE = 5;
const float timeStep =  1/60.0f;

float currentTime = 0;
double accumulator = timeStep;
int selected_index = -1;

int oldX = 0;
int oldY = 0;
float rX = 0;
float rY = -45;
int state = 1;
float dist = 0;

GLint viewport[4];
GLdouble MV[16];
GLdouble P[16];

glm::vec3 Up=glm::vec3(0,1,0), Right, viewDir;

LARGE_INTEGER frequency;//Ticks per second
LARGE_INTEGER t1, t2;
double frameTimeQP = 0;
float frameTime = 0;
float startTime = 0;
float fps = 0;
int totalFrames = 0;
char info[MAX_PATH] = {0};

void drawCube(float x, float y, float z);

float angle;
float lx = 1;
float lz = 1;
float ly = 3;
float xx = 3;
float yy = 3;
float zz = 3;

void processSpecialKeys(int key, int x, int y)
{
	float fraction = 0.1f;

	float m_yaw = 0;// *0.017;
	float m_moveCommand = 0.1;

	Vector3D m_moveCommand2(0.0f, 0.0f, 0.0f);

	switch (key)
	{
	case GLUT_KEY_LEFT:
		m_yaw = (rY - 90) * 0.017;
		zz -= m_moveCommand * (cosf(m_yaw) - m_moveCommand * sinf(m_yaw)) / 2;
		xx += m_moveCommand * (sinf(m_yaw) + m_moveCommand * cosf(m_yaw)) / 2;
		break;
	case GLUT_KEY_RIGHT:
		m_yaw = (rY + 90) * 0.017;
		zz -= m_moveCommand * (cosf(m_yaw) - m_moveCommand * sinf(m_yaw)) / 2;
		xx += m_moveCommand * (sinf(m_yaw) + m_moveCommand * cosf(m_yaw)) / 2;
		break;
	case GLUT_KEY_UP:
		m_yaw = rY * 0.017;
		zz -= m_moveCommand * (cosf(m_yaw) - m_moveCommand * sinf(m_yaw) ) / 2;
		xx += m_moveCommand * (sinf(m_yaw) + m_moveCommand * cosf(m_yaw)) / 2;
		break;
	case GLUT_KEY_DOWN:
		m_yaw = rY * 0.017;
		zz += m_moveCommand * (cosf(m_yaw) - m_moveCommand * sinf(m_yaw)) / 2;
		xx -= m_moveCommand * (sinf(m_yaw) + m_moveCommand * cosf(m_yaw)) / 2;
		break;
	}
}


void keyDown(unsigned char key, int x, int y)
{
	
	if (key == 'w')
	{
		float fraction = 0.1f;
		float m_yaw = 0;// *0.017;
		float m_moveCommand = 0.1;
		m_yaw = rY * 0.017;
		zz -= m_moveCommand * (cosf(m_yaw) - m_moveCommand * sinf(m_yaw)) / 2;
		xx += m_moveCommand * (sinf(m_yaw) + m_moveCommand * cosf(m_yaw)) / 2;
	}

	if (key == 's')
	{
		float fraction = 0.1f;
		float m_yaw = 0;// *0.017;
		float m_moveCommand = 0.1;
		m_yaw = rY * 0.017;
		zz += m_moveCommand * (cosf(m_yaw) - m_moveCommand * sinf(m_yaw)) / 2;
		xx -= m_moveCommand * (sinf(m_yaw) + m_moveCommand * cosf(m_yaw)) / 2;
	}


	if (key == 'a')
	{
		float fraction = 0.1f;
		float m_yaw = 0;// *0.017;
		float m_moveCommand = 0.1;
		m_yaw = (rY - 90) * 0.017;
		zz -= m_moveCommand * (cosf(m_yaw) - m_moveCommand * sinf(m_yaw)) / 2;
		xx += m_moveCommand * (sinf(m_yaw) + m_moveCommand * cosf(m_yaw)) / 2;
	}

	if (key == 'd')
	{
		float fraction = 0.1f;
		float m_yaw = 0;// *0.017;
		float m_moveCommand = 0.1;
		m_yaw = (rY + 90) * 0.017;
		zz -= m_moveCommand * (cosf(m_yaw) - m_moveCommand * sinf(m_yaw)) / 2;
		xx += m_moveCommand * (sinf(m_yaw) + m_moveCommand * cosf(m_yaw)) / 2;
	}

	if (key == 'x')
	{
		yy += 0.1;
	}

	if (key == 'z')
	{
		yy -= 0.1;
	}
}


void OnMouseDown(int button, int s, int x, int y)
{
	if (s == GLUT_DOWN)
	{
		oldX = x;
		oldY = y;
		int window_y = (height - y);
		float norm_y = float(window_y)/float(height/2.0);
		int window_x = x ;
		float norm_x = float(window_x)/float(width/2.0);

		float winZ=0;
		glReadPixels( x, height-y, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &winZ );
		if(winZ==1)
			winZ=0;
		double objX=0, objY=0, objZ=0;
		gluUnProject(window_x,window_y, winZ,  MV,  P, viewport, &objX, &objY, &objZ);
		glm::vec3 pt(objX,objY, objZ);
		size_t i=0;
	}

	if(button == GLUT_MIDDLE_BUTTON)
		state = 0;
	else
		state = 1;

	if(s==GLUT_UP) {
		selected_index= -1;
		glutSetCursor(GLUT_CURSOR_INHERIT);
	}
}

void OnMouseMove(int x, int y)
{
	if(selected_index == -1)
	{
		if (state == 0)
			dist *= (1 + (y - oldY)/60.0f);
		else
		{
			rY += (x - oldX)/5.0f;
			rX += (y - oldY)/5.0f;
		}
	}
	else
	{
		float delta = 1500/abs(dist);
		float valX = (x - oldX)/delta;
		float valY = (oldY - y)/delta;
		if(abs(valX)>abs(valY))
			glutSetCursor(GLUT_CURSOR_LEFT_RIGHT);
		else
			glutSetCursor(GLUT_CURSOR_UP_DOWN);
	}

	oldX = x;
	oldY = y;

	glutPostRedisplay();
}

void DrawMeshVertexRef()
{
	//Draw vertex point references
	glColor3f(1, 0, 0);
	for (int a = 0; a < objWireframeMesh.originalVertexCount; a++)
	{
		stringstream str3;
		str3 << a;

		Vector3D vec(objWireframeMesh.originalVertices[a].x, objWireframeMesh.originalVertices[a].y, objWireframeMesh.originalVertices[a].z);

		glRasterPos3f(vec.x, vec.y, vec.z);
		glutBitmapString(GLUT_BITMAP_TIMES_ROMAN_24, (const unsigned char*)str3.str().c_str());
	}
}

//Draw loaded obj shape file
void DrawWavefrontGeoPoints()
{
	glBegin(GL_POINTS);
	glColor3f(1.0f, 1.0f, 1.0f);

	int triCount = 0;
	for (int a = 0; a < objWireframeMesh.vertexCount; a++)
	{
		Vector3D vert(objWireframeMesh.vertices[a].x, objWireframeMesh.vertices[a].y, objWireframeMesh.vertices[a].z);
		glVertex3f(vert.x, vert.y, vert.z);
	}

	glEnd();
}

//Draw loaded obj shape file
void DrawWavefrontGeo()
{
	glBegin(GL_TRIANGLES);
	glColor3f(0.0f, 1.0f, 0.0f);

	int triCount = 0;
	int vertCount1 = 0;
	
	for (int a = 0; a < objWireframeMesh.indicesCount; a += 3)
	{
		int i1 = objWireframeMesh.indices[a];
		int i2 = objWireframeMesh.indices[a + 1];
		int i3 = objWireframeMesh.indices[a + 2];

		Vector3D vert1(objWireframeMesh.vertices[i1].x, objWireframeMesh.vertices[i1].y, objWireframeMesh.vertices[i1].z);
		Vector3D vert2(objWireframeMesh.vertices[i2].x, objWireframeMesh.vertices[i2].y, objWireframeMesh.vertices[i2].z);
		Vector3D vert3(objWireframeMesh.vertices[i3].x, objWireframeMesh.vertices[i3].y, objWireframeMesh.vertices[i3].z);

		glVertex3f(vert1.x, vert1.y, vert1.z);
		glVertex3f(vert2.x, vert2.y, vert2.z);
		glVertex3f(vert3.x, vert3.y, vert3.z);

		vertCount1 += 3;

		triCount++;
	}

	glEnd();
}


//////////////
//Draw cube
//////////////

GLfloat color[8][3] =
{
	{0.0,0.0,0.0},
	{1.0,0.0,0.0},
	{1.0,1.0,0.0},
	{0.0,1.0,0.0},
	{0.0,0.0,1.0},
	{1.0,0.0,1.0},
	{1.0,1.0,1.0},
	{0.0,1.0,1.0},
};

void quad(int a, int b, int c, int d, float x, float y, float z)
{
	float scale = 0.05;
	float ver[8][3] =
	{
		{-1.0 * scale,-1.0 * scale,1.0 * scale},
		{-1.0 * scale,1.0 * scale,1.0 * scale},
		{1.0 * scale,1.0 * scale,1.0 * scale},
		{1.0 * scale,-1.0 * scale,1.0 * scale},
		{-1.0 * scale,-1.0 * scale,-1.0 * scale},
		{-1.0 * scale,1.0 * scale,-1.0 * scale},
		{1.0 * scale,1.0 * scale,-1.0 * scale},
		{1.0 * scale,-1.0 * scale,-1.0 * scale},
	};

	//-------------------
	ver[0][0] += x;
	ver[0][1] += y;
	ver[0][2] += z;

	ver[1][0] += x;
	ver[1][1] += y;
	ver[1][2] += z;

	ver[2][0] += x;
	ver[2][1] += y;
	ver[2][2] += z;

	ver[3][0] += x;
	ver[3][1] += y;
	ver[3][2] += z;

	ver[4][0] += x;
	ver[4][1] += y;
	ver[4][2] += z;

	ver[5][0] += x;
	ver[5][1] += y;
	ver[5][2] += z;

	ver[6][0] += x;
	ver[6][1] += y;
	ver[6][2] += z;

	ver[7][0] += x;
	ver[7][1] += y;
	ver[7][2] += z;

	//-------------------

	glBegin(GL_QUADS);
	glColor3fv(color[a]);
	glVertex3fv(ver[a]);

	glColor3fv(color[b]);
	glVertex3fv(ver[b]);

	glColor3fv(color[c]);
	glVertex3fv(ver[c]);

	glColor3fv(color[d]);
	glVertex3fv(ver[d]);
	glEnd();
}

void drawCube(float x, float y, float z)
{
	quad(0, 3, 2, 1, x, y, z);
	quad(2, 3, 7, 6, x, y, z);
	quad(0, 4, 7, 3, x, y, z);
	quad(1, 2, 6, 5, x, y, z);
	quad(4, 5, 6, 7, x, y, z);
	quad(0, 1, 5, 4, x, y, z);
}


void DrawGrid()
{
	glBegin(GL_LINES);
	glColor3f(0.5f, 0.5f, 0.5f);
	for(int i = -GRID_SIZE; i <= GRID_SIZE; i++)
	{
		glVertex3f((float)i,0,(float)-GRID_SIZE);
		glVertex3f((float)i,0,(float)GRID_SIZE);
		glVertex3f((float)-GRID_SIZE,0,(float)i);
		glVertex3f((float)GRID_SIZE,0,(float)i);
	}
	glEnd();
}

void InitGL()
{
	startTime = (float)glutGet(GLUT_ELAPSED_TIME);
	currentTime = startTime;

	//get ticks per second
    QueryPerformanceFrequency(&frequency);

    //start timer
    QueryPerformanceCounter(&t1);

	glEnable(GL_DEPTH_TEST);
	
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);//GL_FILL or GL_LINE
	glPointSize(10);

	wglSwapIntervalEXT(0);
}

void OnReshape(int nw, int nh)
{
	glViewport(0, 0, nw, nh);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(60, (GLfloat)nw / (GLfloat)nh, 0.01, 1000.0f);
	glGetIntegerv(GL_VIEWPORT, viewport);
	glGetDoublev(GL_PROJECTION_MATRIX, P);
	glMatrixMode(GL_MODELVIEW);

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glLoadIdentity();
	glTranslatef(0, 0, dist);
	glRotatef(rX, 1, 0, 0);
	glRotatef(rY, 0, 1, 0);

	glGetDoublev(GL_MODELVIEW_MATRIX, MV);
	viewDir.x = (float)-MV[2];
	viewDir.y = (float)-MV[6];
	viewDir.z = (float)-MV[10];
	Right = glm::cross(viewDir, Up);

}



void OnRender()
{
	float fraction = 0.1f;

	lx = sin(angle);
	lz = -cos(angle);

	lx = sin(angle);
	lz = -cos(angle);

	xx += lx * fraction;
	zz += lz * fraction;

	xx -= lx * fraction;
	zz -= lz * fraction;

	size_t i = 0;
	float newTime = (float)glutGet(GLUT_ELAPSED_TIME);
	frameTime = newTime - currentTime;
	currentTime = newTime;

	QueryPerformanceCounter(&t2);
	frameTimeQP = (t2.QuadPart - t1.QuadPart) * 1000.0 / frequency.QuadPart;
	t1 = t2;
	accumulator += 0.01;

	++totalFrames;
	if ((newTime - startTime) > 1000)
	{
		float elapsedTime = (newTime - startTime);
		fps = (totalFrames / elapsedTime) * 1000;
		startTime = newTime;
		totalFrames = 0;
	}

	sprintf_s(info, "Wavefront Viewer | Frame rate: %3.2f fps", fps);
	glutSetWindowTitle(info);

	glClearColor(0,
		0,
		0.5,
		1);

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glLoadIdentity();
	glTranslatef(0, 0, dist);
	glRotatef(rX, 1, 0, 0);
	glRotatef(rY, 0, 1, 0);

	glGetDoublev(GL_MODELVIEW_MATRIX, MV);
	viewDir.x = (float)-MV[2];
	viewDir.y = (float)-MV[6];
	viewDir.z = (float)-MV[10];
	Right = glm::cross(viewDir, Up);

	// Set the camera
	gluLookAt(xx, yy, zz, // Camera position
		xx + lx, yy, zz + lz,  // Look at point
		0.0f, 1.0f, 0.0f); // Up vector


	//[Draw world grid]
	DrawGrid();

	//[Draw wavefront geometry]
	DrawWavefrontGeoPoints();
	DrawWavefrontGeo();

	DrawMeshVertexRef();

	//glEnd();
	glutSwapBuffers();
}

void OnShutdown()
{
}


void OnIdle()
{
	glutPostRedisplay();
}



void main(int argc, char** argv)
{
	objWireframeMesh.loadObj("IcoSphere.obj");

	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
	glutInitWindowSize(width, height);
	glutCreateWindow("Wavefront Viewer");

	glutDisplayFunc(OnRender);
	glutReshapeFunc(OnReshape);//Window resize event
	glutIdleFunc(OnIdle);
	glutMouseFunc(OnMouseDown);
	glutMotionFunc(OnMouseMove);
	glutKeyboardFunc(keyDown);
	glutSpecialFunc(processSpecialKeys);
	glutCloseFunc(OnShutdown);

	glewInit();
	InitGL();//<-- Wireframe mode set

	glutMainLoop();
}