#include "JointHeader.h"

#pragma comment(lib, "glew32.lib")

const string MESH_FILE = "uvsphere.obj";
const bool SHOW_NORMALS = false;//Face normals
const bool SHOW_AVERGE_NORMALS = true;//Show computed avg normals for each vertex
const float NORMAL_WIDTH = 2;
const float NORMAL_LENGTH = 0.1;
const float POINT_SIZE = 10;
const int WIDTH = 1024;
const int HEIGHT = 1024;
const int GRID_SIZE = 5;
const float TIME_STEP =  1/60.0f;
const bool SHOW_GRID = true;
const bool SHOW_VERTEX_REFS = true;

float currentTime = 0;
double accumulator = TIME_STEP;
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

glm::vec3 Up(0, 1, 0);
glm::vec3 Right(0, 1, 0);
glm::vec3 viewDir(0, 1, 0);

LARGE_INTEGER frequency;
LARGE_INTEGER t1, t2;
double frameTimeQP = 0;
float frameTime = 0;
float startTime = 0;
float fps = 0;
int totalFrames = 0;
char info[MAX_PATH] = {0};

float angle;
float lx = 1;
float lz = 1;
float ly = 3;
float xx = 3;
float yy = 3;
float zz = 3;

//Forward declarations
void drawCube(float x, float y, float z);
//

int matching[65536][16];
int matchingCount[65536];
int foundCount = 0;
Vector3D averages[65536];//Computed Average for unique vertex

//Triangles are defined by three unique points for simplicity of rendering (I.e. in DirectX) (No points shared between polys)
//Get the matching point arrays.
void computeVertexGroups()
{
	//vector <int> iterated;//Dont add duplicates

	for (int a = 0; a < objWireframeMesh.vertexCount; a++)//For every vertex
	{
		Vector3D curVertex1(objWireframeMesh.vertices[a].x, objWireframeMesh.vertices[a].y, objWireframeMesh.vertices[a].z);

		bool foundA = false;
		for (int b = 0; b < objWireframeMesh.vertexCount; b++)//Find matching
		{
			bool found = false;
			//if (b == a) { continue; }
			Vector3D curVertex2(objWireframeMesh.vertices[b].x, objWireframeMesh.vertices[b].y, objWireframeMesh.vertices[b].z);

			if (curVertex1.x == curVertex2.x && curVertex1.y == curVertex2.y && curVertex1.z == curVertex2.z)
			{
				found = true;
			}

			/*
			bool found2 = false;

			for (int c = 0; c < iterated.size(); c++)
			{
				if (iterated[c] == b)
				{
					found2 = true;
					break;
				}
			}
			*/
			
			if (found)// && !found2)
			{
				//cout << a << " - " << b << endl;
				matching[a][matchingCount[a]] = b;
				matchingCount[a]++;
				foundA = true;

				//iterated.push_back(b);
				//iterated.push_back(a);
			}
		}

		if (foundA) { foundCount++; }
	}
}

void computeAverageNormals()
{
	computeVertexGroups();

	for (int a = 0; a < foundCount; a++)
	{
		Vector3D avgNormal(0,0,0);
	
		vector <Vector3D> curNormals;

		//Tgt vertex: Find all matching. NB: Some vertex indexes will have no matching!
		int curMatching = matching[a][0];//Only look at one (Expand to all in the group later)
		Vector3D tgtVertex(objWireframeMesh.vertices[curMatching].x, objWireframeMesh.vertices[curMatching].y, objWireframeMesh.vertices[curMatching].z);

		//Compute the average for every face that contains the tgt vertex
		for (int c = 0; c < objWireframeMesh.indicesCount; c += 3)//For every face
		{
			int i1 = objWireframeMesh.indices[c];
			int i2 = objWireframeMesh.indices[c + 1];
			int i3 = objWireframeMesh.indices[c + 2];

			bool found = false;

			if (objWireframeMesh.vertices[i1].x == tgtVertex.x && objWireframeMesh.vertices[i1].y == tgtVertex.y && objWireframeMesh.vertices[i1].z == tgtVertex.z)
			{
				found = true;
			}

			if (objWireframeMesh.vertices[i2].x == tgtVertex.x && objWireframeMesh.vertices[i2].y == tgtVertex.y && objWireframeMesh.vertices[i2].z == tgtVertex.z)
			{
				found = true;
			}

			if (objWireframeMesh.vertices[i3].x == tgtVertex.x && objWireframeMesh.vertices[i3].y == tgtVertex.y && objWireframeMesh.vertices[i3].z == tgtVertex.z)
			{
				found = true;
			}

			if (found == true)//All points have entries
			{
				Vector3D curNorm(objWireframeMesh.normals[i1].x, objWireframeMesh.normals[i1].y, objWireframeMesh.normals[i1].z);//Correct normal for this face
				//cout << "found. Normal: [" << curNorm.x << " " << curNorm.y << " " << curNorm.z << "]" << endl;
				curNormals.push_back(curNorm);
			}
		}

		//[Remove duplicate normals for correct computation. Should be maximum 3 matches for each point.]
		vector <Vector3D> uniqueNormals;
		
		for (int a = 0; a < curNormals.size(); a++)
		{
			Vector3D curNormal(curNormals[a].x, curNormals[a].y, curNormals[a].z);
			bool found = false;
			for (int a = 0; a < uniqueNormals.size(); a++)
			{
				if (curNormal.x == uniqueNormals[a].x && curNormal.y == uniqueNormals[a].y && curNormal.z == uniqueNormals[a].z)
				{
					found = true;
				}
			}

			if (!found) { uniqueNormals.push_back(curNormal); }
		}

		curNormals = uniqueNormals;

		//[Average normal. Compute average for every face that contained the tgt vertex (prior block)]
		for (int a = 0; a < curNormals.size(); a++)
		{
			avgNormal.x += curNormals[a].x;
			avgNormal.y += curNormals[a].y;
			avgNormal.z += curNormals[a].z;
		}

		avgNormal.x /= curNormals.size() - 1;
		avgNormal.y /= curNormals.size() - 1;
		avgNormal.z /= curNormals.size() - 1;
		//cout << curNormals.size() << " normals found." << endl;
		//cout << "Avg. Normal: [" << avgNormal.x << " " << avgNormal.y << " " << avgNormal.z << "]" << endl << endl;
		//------------------------

		//cout << "a:" << a << endl;

		for (int b = 0; b < matchingCount[a]; b++)
		{
			int curInt = matching[a][b];
			for (int b = 0; b < matchingCount[a]; b++)
			{
				averages[curInt] = avgNormal;
			}
		}
	}
}


Vector3D computeCentroid()
{
	Vector3D centroid(0, 0, 0);

	float avgX = 0;
	float avgY = 0;
	float avgZ = 0;

	int n = 0;

	//[Get collision mesh triangles]
	for (int c = 0; c < objWireframeMesh.indicesCount; c += 3)
	{
		int i1 = objWireframeMesh.indices[c];
		int i2 = objWireframeMesh.indices[c + 1];
		int i3 = objWireframeMesh.indices[c + 2];

		//[CM verts]
		avgX += objWireframeMesh.vertices[i1].x;
		avgY += objWireframeMesh.vertices[i1].y;
		avgZ += objWireframeMesh.vertices[i1].z;

		avgX += objWireframeMesh.vertices[i2].x;
		avgY += objWireframeMesh.vertices[i2].y;
		avgZ += objWireframeMesh.vertices[i2].z;

		avgX += objWireframeMesh.vertices[i3].x;
		avgY += objWireframeMesh.vertices[i3].y;
		avgZ += objWireframeMesh.vertices[i3].z;

		n += 3;
	}

	centroid.x = avgX / n;
	centroid.y = avgY / n;
	centroid.z = avgZ / n;

	return centroid;
}


void meshExpand(float amount)
{
	for (int a = 0; a < objWireframeMesh.vertexCount; a++)
	{
		Vector3D expand(averages[a].x,  averages[a].y, averages[a].z);
		expand = vector3DUtils.setVectorMagnitude(expand, amount);

		objWireframeMesh.vertices[a].x += expand.x;
		objWireframeMesh.vertices[a].y += expand.y;
		objWireframeMesh.vertices[a].z += expand.z;
	}
}


void drawAvgNormals()
{
	glColor3f(0.5f, 0.5f, 0.5f);
	glLineWidth(NORMAL_WIDTH);
	glBegin(GL_LINES);
	for (int a = 0; a < objWireframeMesh.vertexCount; a++)
	{
		averages[a] = vector3DUtils.setVectorMagnitude(averages[a], NORMAL_LENGTH);
			
		glVertex3f(objWireframeMesh.vertices[a].x, objWireframeMesh.vertices[a].y, objWireframeMesh.vertices[a].z);
		glVertex3f(objWireframeMesh.vertices[a].x + averages[a].x, objWireframeMesh.vertices[a].y + averages[a].y, objWireframeMesh.vertices[a].z + averages[a].z);
	}
	glEnd();
	glLineWidth(1);
}


void drawNormals()
{
	glColor3f(0.5f, 0.5f, 0.5f);

	glLineWidth(NORMAL_WIDTH);
	for (int a = 0; a < objWireframeMesh.indicesCount; a += 3)
	{

		int i1 = objWireframeMesh.indices[a];
		int i2 = objWireframeMesh.indices[a + 1];
		int i3 = objWireframeMesh.indices[a + 2];

		Vector3D tgtVertex(objWireframeMesh.vertices[i1].x, objWireframeMesh.vertices[i1].y, objWireframeMesh.vertices[i1].z);

		bool found = false;

		if (objWireframeMesh.vertices[i1].x == tgtVertex.x && objWireframeMesh.vertices[i1].y == tgtVertex.y && objWireframeMesh.vertices[i1].z == tgtVertex.z)
		{
			found = true;
		}

		if (objWireframeMesh.vertices[i2].x == tgtVertex.x && objWireframeMesh.vertices[i2].y == tgtVertex.y && objWireframeMesh.vertices[i2].z == tgtVertex.z)
		{
			found = true;
		}

		if (objWireframeMesh.vertices[i3].x == tgtVertex.x && objWireframeMesh.vertices[i3].y == tgtVertex.y && objWireframeMesh.vertices[i3].z == tgtVertex.z)
		{
			found = true;
		}

		if (found)
		{
			Vector3D faceNorm(objWireframeMesh.normals[a].x, objWireframeMesh.normals[a].y, objWireframeMesh.normals[a].z);
			faceNorm = vector3DUtils.setVectorMagnitude(faceNorm, NORMAL_LENGTH);

			Vector3D centroid((objWireframeMesh.vertices[i1].x + objWireframeMesh.vertices[i2].x + objWireframeMesh.vertices[i3].x) / 3, (objWireframeMesh.vertices[i1].y + objWireframeMesh.vertices[i2].y + objWireframeMesh.vertices[i3].y) / 3, (objWireframeMesh.vertices[i1].z + objWireframeMesh.vertices[i2].z + objWireframeMesh.vertices[i3].z) / 3);

			glBegin(GL_LINES);
			glVertex3f(centroid.x, centroid.y, centroid.z);
			glVertex3f(centroid.x + faceNorm.x, centroid.y + faceNorm.y, centroid.z + faceNorm.z);
			glEnd();
		}
	}

	glLineWidth(1);
}

void processSpecialKeys(int key, int x, int y)
{
	float m_yaw = 0;// *0.017;
	float m_moveCommand = 0.1;

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
		float m_yaw = 0;// *0.017;
		float m_moveCommand = 0.1;
		m_yaw = rY * 0.017;
		zz -= m_moveCommand * (cosf(m_yaw) - m_moveCommand * sinf(m_yaw)) / 2;
		xx += m_moveCommand * (sinf(m_yaw) + m_moveCommand * cosf(m_yaw)) / 2;
	}

	if (key == 's')
	{
		float m_yaw = 0;// *0.017;
		float m_moveCommand = 0.1;
		m_yaw = rY * 0.017;
		zz += m_moveCommand * (cosf(m_yaw) - m_moveCommand * sinf(m_yaw)) / 2;
		xx -= m_moveCommand * (sinf(m_yaw) + m_moveCommand * cosf(m_yaw)) / 2;
	}

	if (key == 'a')
	{
		float m_yaw = 0;// *0.017;
		float m_moveCommand = 0.1;
		m_yaw = (rY - 90) * 0.017;
		zz -= m_moveCommand * (cosf(m_yaw) - m_moveCommand * sinf(m_yaw)) / 2;
		xx += m_moveCommand * (sinf(m_yaw) + m_moveCommand * cosf(m_yaw)) / 2;
	}

	if (key == 'd')
	{
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


void onMouseDown(int button, int s, int x, int y)
{
	if (s == GLUT_DOWN)
	{
		oldX = x;
		oldY = y;
		int window_y = (HEIGHT - y);
		float norm_y = float(window_y)/float(HEIGHT/2.0);
		int window_x = x ;
		float norm_x = float(window_x)/float(WIDTH/2.0);

		float winZ=0;
		glReadPixels( x, HEIGHT-y, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &winZ );
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

void onMouseMove(int x, int y)
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

void drawMeshVertexRef()
{
	if (!SHOW_VERTEX_REFS) { return; }

	//Draw original vertex point references
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
void drawWavefrontGeoPoints()
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
void drawWavefrontGeo()
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

void drawGrid()
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

void initGL()
{
	startTime = (float)glutGet(GLUT_ELAPSED_TIME);
	currentTime = startTime;

	//get ticks per second
    QueryPerformanceFrequency(&frequency);

    //start timer
    QueryPerformanceCounter(&t1);

	glEnable(GL_DEPTH_TEST);
	
	glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);//GL_FILL or GL_LINE
	glPointSize(POINT_SIZE);

	wglSwapIntervalEXT(0);
}

void onReshape(int nw, int nh)
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


void onRender()
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

	//Set the camera
	gluLookAt(xx, yy, zz, //Camera position
		xx + lx, yy, zz + lz, //Look at point
		0.0f, 1.0f, 0.0f); //Up vector

	//[Draw world grid]
	if (SHOW_GRID)
	{
		drawGrid();
	}

	if (SHOW_NORMALS)
	{
		drawNormals();
	}

	if (SHOW_AVERGE_NORMALS)
	{
		drawAvgNormals();
	}

	//[Draw wavefront geometry]
	drawWavefrontGeoPoints();
	drawWavefrontGeo();
	drawMeshVertexRef();

	glutSwapBuffers();
}

void onShutdown()
{
}


void onIdle()
{
	glutPostRedisplay();
}


void main(int argc, char** argv)
{
	objWireframeMesh.loadObj(MESH_FILE);
	computeAverageNormals();
	//meshExpand(0.5);//Expand loaded mesh outwards along the average normals similar to Blender's solidify modifier

	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
	glutInitWindowSize(WIDTH, HEIGHT);
	glutCreateWindow("Wavefront Viewer");

	glutDisplayFunc(onRender);
	glutReshapeFunc(onReshape);//Window resize event
	glutIdleFunc(onIdle);
	glutMouseFunc(onMouseDown);
	glutMotionFunc(onMouseMove);
	glutKeyboardFunc(keyDown);
	glutSpecialFunc(processSpecialKeys);
	glutCloseFunc(onShutdown);

	glewInit();
	initGL();//<-- Wireframe mode set

	glutMainLoop();
}