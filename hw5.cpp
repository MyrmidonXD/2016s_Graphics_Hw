/// 2014 - 11111 Gyeonghun Mun
//  
//  Graphics Homework #5

#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/freeglut.h>
#include <iostream>
#include <cmath>
#include <cstdlib>
#include <algorithm>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <parser.h>
#include <spline.h>
#include <models.h>
#include <EasyBMP.h>

using namespace std;
using namespace Eigen;

GLdouble rotMatrix[16] =
{
	1, 0, 0, 0,
	0, 1, 0, 0,
	0, 0, 1, 0,
	0, 0, 0, 1
};

float initCamDist = 500.0f;

float eye[3] = { 0.0f, 150.0f, initCamDist };
float ori[3] = { 0.0f, 150.0f, 0.0f };
float upv[3] = { 0.0f, 1.0f, 0.0f };
float rot[3] = { 0.0f, 0.0f, 0.0f };

Quaternionf prevRotation(1.0, 0.0, 0.0, 0.0);
void loadGlobalCoord()
{
    glLoadIdentity();
    gluLookAt(  eye[0], eye[1], eye[2],
                ori[0], ori[1], ori[2],
                upv[0], upv[1], upv[2]  );
    glMultMatrixd(rotMatrix);
}

bool zoomChanged = false;
float horizontalTranslate = 0.0f;
float verticalTranslate = 0.0f;
float fovy = 45.0f, zNear = 0.1f, zFar = 2000.0f;
int width = 800, height = 600;

//////////////////// Needed field for HW3
// Splines
Spline *pos_spline;
FloatSpline *scale_spline;
QuaternionSpline *rot_spline;
vector<Spline> cp_splines;

// integers
int sec_num, cp_num;

int surf_level = 3, leng_level = 3;

int drawing_mode = 1; // 0, 1, 2

// strings
string sp_type = "";

// Model Vertices Data
// vector<vector<vector<Vector3f>>> model;

////////////////////

//////////////////// Needed field for HW4
// Lights
GLfloat light0Pos[] = { 0.0, 300.0, 0.0, 1.0 };
GLfloat light1Pos[] = { 0.0, 400.0, 0.0, 1.0 };
GLfloat light2Pos[] = { 0.0, 0.0, -200.0, 1.0 };
GLfloat light3Pos[] = { 0.0, 100.0, 200.0, 1.0};

// Scene
GScene *scene;

// Global flag
bool useBSP = true;

//-- Material
// 1) For BackGround.
GLfloat tr1_ambient[] = { 0.1, 0.1, 0.1, 1.0 };
GLfloat tr1_diffuse[] = { 0.3, 0.3, 0.3, 1.0 };
GLfloat tr1_specular[] = { 0.6, 0.6, 0.6, 1.0 };
GLfloat tr1_shininess = 112.0;

// 2) For objects

GLfloat tr2_ambient[] = { 0.35, 0.2, 0.2, 1.0 };
GLfloat tr2_diffuse[] = { 0.965, 0.789, 0.785, 0.6 };
GLfloat tr2_specular[] = { 0.0, 0.0, 0.0, 1.0 };
GLfloat tr2_shininess = 108.0;

GLfloat tr3_ambient[] = { 0.1, 0.0, 0.0, 1.0 };
GLfloat tr3_diffuse[] = { 0.8, 0.0, 0.0, 0.2 };
GLfloat tr3_specular[] = { 0.7, 0.1, 0.1, 1.0 };
GLfloat tr3_shininess = 108.0;

GLfloat tr4_ambient[] = { 0.02, 0.02, 0.02, 1.0 };
GLfloat tr4_diffuse[] = { 0.0, 0.4, 0.0, 0.2 };
GLfloat tr4_specular[] = { 0.6, 0.6, 0.6, 1.0 };
GLfloat tr4_shininess = 108.0;

// 3) Silver

GLfloat mat2_ambient[] = { 0.2, 0.2, 0.2, 1.0 };
GLfloat mat2_diffuse[] = { 0.3, 0.3, 0.3, 1.0 };
GLfloat mat2_specular[] = { 0.8, 0.8, 0.8, 1.0 };
GLfloat mat2_shininess = 16.0;

// 4) Gold

GLfloat mat3_ambient[] = { 0.4, 0.3, 0.1, 1.0 };
GLfloat mat3_diffuse[] = { 0.8, 0.6, 0.2, 1.0 };
GLfloat mat3_specular[] = { 0.83, 0.7, 0.0, 1.0 };
GLfloat mat3_shininess = 16.0;

// 5) Rubber(red)

GLfloat mat4_ambient[] = { 0.18, 0.0, 0.0, 1.0 };
GLfloat mat4_diffuse[] = { 0.92, 0.0, 0.0, 1.0 };
GLfloat mat4_specular[] = { 0.3, 0.0, 0.0, 1.0 };
GLfloat mat4_shininess = 120.0;

// 6) Skin

GLfloat mat5_ambient[] = { 0.25, 0.199, 0.149, 1.0 };
GLfloat mat5_diffuse[] = { 0.8, 0.638, 0.478, 1.0 };
GLfloat mat5_specular[] = { 0.0, 0.0, 0.0, 1.0 };
GLfloat mat5_shininess = 128.0;

// 7) Black Enamel Leather

GLfloat mat6_ambient[] = { 0.02, 0.02, 0.02, 1.0 };
GLfloat mat6_diffuse[] = { 0.05, 0.05, 0.05, 1.0 };
GLfloat mat6_specular[] = { 1.0, 1.0, 1.0, 1.0 };
GLfloat mat6_shininess = 32.0;

// 8) For opaque swept surfaces
GLfloat mat1_ambient[] = { 0.042, 0.064, 0.204, 1.0 };
GLfloat mat1_diffuse[] = { 0.166, 0.356, 0.816, 1.0 };
GLfloat mat1_specular[] = { 0.3, 0.3, 0.3, 1.0 };
GLfloat mat1_shininess = 112.0;


//--

///////////////////

GModel* makeCube(GLfloat minX, GLfloat minY, GLfloat minZ, GLfloat maxX, GLfloat maxY, GLfloat maxZ)
{
  vector<Vector3f> vlist;
  static vector<Vector3i> idxlist;

  GModel *cube = new GModel();

  vlist.emplace_back(minX, maxY, minZ);
  vlist.emplace_back(minX, maxY, maxZ);
  vlist.emplace_back(maxX, maxY, maxZ);
  vlist.emplace_back(maxX, maxY, minZ);
  vlist.emplace_back(minX, minY, minZ);
  vlist.emplace_back(minX, minY, maxZ);
  vlist.emplace_back(maxX, minY, maxZ);
  vlist.emplace_back(maxX, minY, minZ);


  if(idxlist.empty())
  {
    idxlist.emplace_back(0, 1, 2);
    idxlist.emplace_back(0, 2, 3);
    idxlist.emplace_back(0, 4, 5);
    idxlist.emplace_back(0, 5, 1);
    idxlist.emplace_back(1, 5, 6);
    idxlist.emplace_back(1, 6, 2);
    idxlist.emplace_back(2, 6, 7);
    idxlist.emplace_back(2, 7, 3);
    idxlist.emplace_back(3, 7, 4);
    idxlist.emplace_back(3, 4, 0);
    idxlist.emplace_back(4, 7, 6);
    idxlist.emplace_back(4, 6, 5);
  }
  
  assert(idxlist.size() == 12);

  for(int i = 0; i < 12; i++)
  {
    Vector3i idx = idxlist[i];
    GSurface *curr = new GSurface(vlist[idx[0]], vlist[idx[1]], vlist[idx[2]]);
    
    for(int j = 0; j < 3; j++)
    {
      curr->pushNormal(curr->getSurfaceNormal());
    }

    cube->addSurface(curr);
  }

  return cube;
}

GModel* makeSphere(GLfloat radius, GLfloat x, GLfloat y, GLfloat z)
{
  Vector3f center(x, y, z);

  GModel *sphere = new GModel();

  sphere->setSphere(radius, center);

  return sphere;
}

GModel* makeBackground(GLfloat size)
{
  vector<Vector3f> vlist;
  static vector<Vector3i> idxlist;

  GModel *background_cube = new GModel();

  vlist.emplace_back(-size, 2.0*size, -size);
  vlist.emplace_back(-size, 2.0*size, 3.0*size);
  vlist.emplace_back(size, 2.0*size, 3.0*size);
  vlist.emplace_back(size, 2.0*size, -size);
  vlist.emplace_back(-size, 0.0, -size);
  vlist.emplace_back(-size, 0.0, 3.0*size);
  vlist.emplace_back(size, 0.0, 3.0*size);
  vlist.emplace_back(size, 0.0, -size);


  if(idxlist.empty())
  {
    idxlist.emplace_back(0, 3, 2);
    idxlist.emplace_back(0, 2, 1);
    idxlist.emplace_back(1, 5, 4);
    idxlist.emplace_back(1, 4, 0);
    idxlist.emplace_back(0, 4, 7);
    idxlist.emplace_back(0, 7, 3);
    idxlist.emplace_back(3, 7, 6);
    idxlist.emplace_back(3, 6, 2);
    idxlist.emplace_back(2, 6, 5);
    idxlist.emplace_back(2, 5, 1);
    idxlist.emplace_back(4, 5, 6);
    idxlist.emplace_back(4, 6, 7);
  }
  
  assert(idxlist.size() == 12);

  for(int i = 0; i < 12; i++)
  {
    Vector3i idx = idxlist[i];
    GSurface *curr = new GSurface(vlist[idx[0]], vlist[idx[1]], vlist[idx[2]]);
    
    for(int j = 0; j < 3; j++)
    {
      curr->pushNormal(curr->getSurfaceNormal());
    }

    background_cube->addSurface(curr);
  }

  return background_cube;
}

void lightSetup()
{
  // Light Data
  GLfloat ambientLight0[] = { 0.2, 0.2, 0.2, 1.0 };
  GLfloat diffuseLight0[] = { 1.0, 1.0, 1.0, 1.0 };
  GLfloat specularLight0[] = { 1.0, 1.0, 1.0, 1.0 };

  /*
  GLfloat ambientLight1[] = { 0.3, 0.3, 0.3, 1.0 };
  GLfloat diffuseLight1[] = { 1.0, 1.0, 1.0, 1.0 };
  GLfloat specularLight1[] = { 1.0, 1.0, 1.0, 1.0 };
  
  GLfloat ambientLight2[] = { 0.1, 0.1, 0.1, 1.0 };
  GLfloat diffuseLight2[] = { 1.0, 1.0, 1.0, 1.0 };
  GLfloat specularLight2[] = { 0.8, 0.8, 0.8, 1.0 };
  
  GLfloat ambientLight3[] = { 0.0, 0.0, 0.2, 1.0 };
  GLfloat diffuseLight3[] = { 0.6, 0.6, 0.6, 1.0 };
  GLfloat specularLight3[] = { 0.4, 0.4, 0.4, 1.0 };
  */

  glEnable(GL_LIGHTING);
  glLightfv(GL_LIGHT0, GL_AMBIENT, ambientLight0);
  glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuseLight0);
  glLightfv(GL_LIGHT0, GL_SPECULAR, specularLight0);
  glLightfv(GL_LIGHT0, GL_POSITION, light0Pos);
  glEnable(GL_LIGHT0);
  
  /*
  glLightfv(GL_LIGHT1, GL_AMBIENT, ambientLight1);
  glLightfv(GL_LIGHT1, GL_DIFFUSE, diffuseLight1);
  glLightfv(GL_LIGHT1, GL_SPECULAR, specularLight1);
  glLightfv(GL_LIGHT1, GL_POSITION, light1Pos);
  glEnable(GL_LIGHT1);
  
  glLightfv(GL_LIGHT2, GL_AMBIENT, ambientLight2);
  glLightfv(GL_LIGHT2, GL_DIFFUSE, diffuseLight2);
  glLightfv(GL_LIGHT2, GL_SPECULAR, specularLight2);
  glLightfv(GL_LIGHT2, GL_POSITION, light2Pos);
  glEnable(GL_LIGHT2);
  
  glLightfv(GL_LIGHT3, GL_AMBIENT, ambientLight3);
  glLightfv(GL_LIGHT3, GL_DIFFUSE, diffuseLight3);
  glLightfv(GL_LIGHT3, GL_SPECULAR, specularLight3);
  glLightfv(GL_LIGHT3, GL_POSITION, light3Pos);
  glEnable(GL_LIGHT3);
  */
}

void camMouseSeek(int x, int y)
{
    GLint viewport[4];
    GLdouble modelview[16];
    GLdouble projection[16];
    GLfloat winX, winY, winZ;
    GLdouble worldX, worldY, worldZ;
    bool onObject = false;

    glGetIntegerv(GL_VIEWPORT, viewport);
    glGetDoublev(GL_MODELVIEW_MATRIX, modelview);
    glGetDoublev(GL_PROJECTION_MATRIX, projection);
    
    winX = (float)x;
    winY = (float)height - (float) y;
    glReadPixels(winX, winY, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &winZ);
    
    if(winZ < 0.99999f)
    {
        onObject = true;
        gluUnProject(winX, winY, winZ, modelview, projection, viewport, &worldX, &worldY, &worldZ);
    }

    Vector3f upVector(upv[0], upv[1], upv[2]);
    Vector3f oriToCam(eye[0]-ori[0], eye[1]-ori[1], eye[2]-ori[2]);
    Vector3f prevOri(ori[0], ori[1], ori[2]);
    Vector3f xAxisVec = upVector.cross(oriToCam).normalized();
    Vector3f v1, v2;

    if(onObject)
    {
        Vector3f newOri((float)worldX, (float)worldY, (float)worldZ);

        ori[0] = newOri[0];
        ori[1] = newOri[1];
        ori[2] = newOri[2];

        v1 = (-1.0 * oriToCam).normalized();
        v2 = (newOri - (prevOri + oriToCam)).normalized();
    }
    else
    {
        float yCoeff = oriToCam.norm() * tan((3.141592 / 180.0) * (fovy/2.0));
        float xCoeff = yCoeff * ((GLfloat) width / (GLfloat) height);

        float xWorldDelta = ((float) (x - width/2.0) / (float) (width/2.0)) * xCoeff;
        float yWorldDelta = ((float) (height/2.0 - y) / (float) (height/2.0)) * yCoeff;

        Vector3f newOri = prevOri + (xWorldDelta * xAxisVec) + (yWorldDelta * upVector);

        ori[0] = newOri[0];
        ori[1] = newOri[1];
        ori[2] = newOri[2];

        v1 = (-1.0 * oriToCam).normalized();
        v2 = (newOri - (prevOri + oriToCam)).normalized();
    }

    Vector3f crossProduct = v1.cross(v2);
    float dotProduct = v1.dot(v2);

    float sin = crossProduct.norm() / (v1.norm() * v2.norm());
    float cos = dotProduct / (v1.norm() * v2.norm());

    if(sin > 0.00001f)
    {
        Quaternionf camRotation(*(new AngleAxisf(atan2(sin,cos), crossProduct.normalized())));
        Quaternionf upVecQuat(0.0, upv[0], upv[1], upv[2]);

        upVecQuat = camRotation * upVecQuat * camRotation.inverse();
        upv[0] = upVecQuat.x();
        upv[1] = upVecQuat.y();
        upv[2] = upVecQuat.z();

        prevRotation = camRotation * prevRotation;
    }
}

void camShowAll()
{
    float idealDist = initCamDist * tan((3.141592 / 180.0) * (90.0 - fovy/2.0));
    Vector3f camPos(eye[0], eye[1], eye[2]);
    Vector3f camToOri(ori[0]-eye[0], ori[1]-eye[1], ori[2]-eye[2]);

    Vector3f v1, v2;
    v1 = camToOri.normalized();
    v2 = -1.0 * camPos.normalized();

    Vector3f crossProduct = v1.cross(v2);
    float dotProduct = v1.dot(v2);

    float sin = crossProduct.norm() / (v1.norm() * v2.norm());
    float cos = dotProduct / (v1.norm() * v2.norm());

    if(sin > 0.00001f)
    {
        Quaternionf camRotation(*(new AngleAxisf(atan2(sin,cos), crossProduct.normalized())));
        Quaternionf upVecQuat(0.0, upv[0], upv[1], upv[2]);

        upVecQuat = camRotation * upVecQuat * camRotation.inverse();
        upv[0] = upVecQuat.x();
        upv[1] = upVecQuat.y();
        upv[2] = upVecQuat.z();

        prevRotation = camRotation * prevRotation;
    }


    if(idealDist < 40.0) idealDist += 40.0; // When FOV is too high (near 180 deg)
    
    if (idealDist > 1500.0f) // When FOV is too low (near 0 deg)
    {
        zNear = idealDist - 750.0f;
        zFar = idealDist + 750.0f;
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        gluPerspective(fovy, (GLfloat)width/(GLfloat)height, zNear, zFar);
        glMatrixMode(GL_MODELVIEW);
    }
    else if(zFar > 1501.0f)
    {
        zNear = 0.1f;
        zFar = 1500.0f;
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        gluPerspective(fovy, (GLfloat)width/(GLfloat)height, zNear, zFar);
        glMatrixMode(GL_MODELVIEW);
    }

    camPos = idealDist * camPos.normalized();
    for(int i = 0; i < 3; i++)
    {
        eye[i] = camPos[i];
        ori[i] = 0.0f;
    }
}

void camTranslate(float xDelta, float yDelta)
{
    Vector3f oriToCam(eye[0]-ori[0], eye[1]-ori[1], eye[2]-ori[2]);
    Vector3f upVector(upv[0], upv[1], upv[2]); // assumption: Size of upVector is always 1.
    Vector3f xAxisVec = (upVector.cross(oriToCam)).normalized();

    xAxisVec = xAxisVec * xDelta;
    upVector = upVector * yDelta;

    Vector3f res = xAxisVec + upVector;
    for(int i = 0; i < 3; i++)
    {
        eye[i] += res[i];
        ori[i] += res[i];
    }

}

void camDolly(float delta)
{
    Vector3f oriToCam(eye[0]-ori[0], eye[1]-ori[1], eye[2]-ori[2]);

    oriToCam = (oriToCam.norm() + delta) * oriToCam.normalized();

    eye[0] = ori[0] + oriToCam[0];
    eye[1] = ori[1] + oriToCam[1];
    eye[2] = ori[2] + oriToCam[2];
}

void glutKeyboardInput(unsigned char key, int x, int y)
{
    switch(key) 
    {
        case 27:
            exit(0);
            break;
        case 'a':
        case 'A':
            camTranslate(-8.0, 0.0);
            break;
        case 'd':
        case 'D':
            camTranslate(8.0, 0.0);
            break;
        case 'w':
        case 'W':
            camTranslate(0.0, 6.0);
            break;
        case 's':
        case 'S':
            camTranslate(0.0, -6.0);
            break;
        case 'z':
        case 'Z':
            if(fovy > 6.0f) 
            {
                fovy -= 5.0f;
                zoomChanged = true;
            }
            break;
        case 'x':
        case 'X':
            if(fovy < 174.0f)
            {
                fovy += 5.0f;
                zoomChanged = true;
            }
            break;
        case 't':
        case 'T':
            camDolly(-10.0f);
            break;
        case 'g':
        case 'G':
            camDolly(10.0f);
            break;
        case 'r':
        case 'R':
            camShowAll();
            break;

        case '1':
            useBSP = true;
            break;
        case '2':
            useBSP = false;
            break;

        case 'p':
        case 'P': // Print Camera Setting
            cout << "eye: " << eye[0] << ", " << eye[1] << ", " << eye[2] << endl;
            cout << "fov: " << fovy << endl;
            cout << "zNear: " << zNear << ", zFar: " << zFar <<endl;
        default:
            break;
    }
}

bool leftButton = false;
GLfloat mousePosX, mousePosY;

float zOnTrackball(float rad, float &x, float &y)
{
    float z_squared = rad*rad - x*x - y*y;
    if(z_squared < 0.0001f) // Renomalize 
    {
        Vector3f vecOnBall(x, y, 0.0001f);
        vecOnBall = rad * vecOnBall.normalized();
        x = vecOnBall(0);
        y = vecOnBall(1);
        return 0.0001f;
    }
    else
        return sqrt(z_squared);
}

//------------------------------------------------------------------------
// Moves the screen based on mouse pressed button
//------------------------------------------------------------------------

void glutMotion(int x, int y)
{
	if ( leftButton ) {

	    float v1x = (float)mousePosX - (float)width/2, v1y = (float)height/2 - (float)mousePosY;
	    float v2x = (float)x - (float)width/2, v2y = (float)height/2 - (float)y;
	    float radius = 0.9 * min(width/2.0, height/2.0);

	    float v1z = zOnTrackball(radius, v1x, v1y);
	    float v2z = zOnTrackball(radius, v2x, v2y);

	    
	    Quaternionf v1Quat(0.0, v1x, v1y, v1z);
	    Quaternionf v2Quat(0.0, v2x, v2y, v2z);

	    v1Quat = prevRotation * v1Quat * prevRotation.inverse();
	    v2Quat = prevRotation * v2Quat * prevRotation.inverse();

	    Vector3f v1(v1Quat.x(), v1Quat.y(), v1Quat.z());
	    Vector3f v2(v2Quat.x(), v2Quat.y(), v2Quat.z());

	    Vector3f cross_product = v2.cross(v1);
	    float dot_product = v2.dot(v1);

	    float cos = dot_product / (v1.norm() * v2.norm());
	    float sin = cross_product.norm() / (v1.norm() * v2.norm()); 
	    // when sin is near zero (zero rotation, 180 deg rotation)
	    if (sin < 0.00001f) return;

	    float angle = atan2(sin, cos);
	    Vector3f axis = cross_product.normalized();
	    
		mousePosX = x;
		mousePosY = y;

		Quaternionf currRotation(*(new AngleAxisf(angle, axis)));
		Quaternionf camPointQuat(0.0, eye[0]-ori[0], eye[1]-ori[1], eye[2]-ori[2]);
		Quaternionf upVecQuat(0.0, upv[0], upv[1], upv[2]);

		camPointQuat = currRotation * camPointQuat * currRotation.inverse();
		upVecQuat = currRotation * upVecQuat * currRotation.inverse();

		for(int i = 0; i < 3; i++)
        {
            float camPointElem, upVecElem;
            switch(i)
            {
                case 0:
                    camPointElem = camPointQuat.x();
                    upVecElem = upVecQuat.x();
                    break;
                case 1:
                    camPointElem = camPointQuat.y();
                    upVecElem = upVecQuat.y();
                    break;
                case 2:
                    camPointElem = camPointQuat.z();
                    upVecElem = upVecQuat.z();
                    break;
                default:
                    camPointElem = 0.0;
                    upVecElem = 0.0;
                    break;
            }
            eye[i] = ori[i] + camPointElem;
            upv[i] = upVecElem;
        }

		prevRotation = currRotation * prevRotation;

		glutPostRedisplay();
	}
	return;
}

//------------------------------------------------------------------------
// Function that handles mouse input
//------------------------------------------------------------------------
void glutMouse(int button, int state, int x, int y)
{
	switch ( button )
	{
		case GLUT_LEFT_BUTTON:
			if ( state == GLUT_DOWN )
			{
				mousePosX = x;
				mousePosY = y;
				leftButton = true;
			}
			else if ( state == GLUT_UP )
			{
				leftButton = false;
			}
			break;
		case GLUT_RIGHT_BUTTON:
		    if(state == GLUT_DOWN)
		        camMouseSeek(x, y);
		    break;
		case 3:break;
		default:break;
	}
	return;
}

float timeElapsed = 0.0f;


void display()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    if(zoomChanged) 
    {
        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
        gluPerspective(fovy, (GLfloat)width/(GLfloat)height, zNear, zFar);
        glMatrixMode(GL_MODELVIEW);
        zoomChanged = false;
    }
    loadGlobalCoord();

    glPushMatrix();
    { // x, y, z axis lines in world coordinate
      glEnable(GL_COLOR_MATERIAL);
      
      glLineWidth(2.5);
      // x axis
      glColor3f(1.0, 0.0, 0.0);
      glBegin(GL_LINES);
      glVertex3f(0.0, 0.0, 0.0);
      glVertex3f(1500.0, 0.0, 0.0);
      glEnd();

      // y axis
      glColor3f(0.0, 1.0, 0.0);
      glBegin(GL_LINES);
      glVertex3f(0.0, 0.0, 0.0);
      glVertex3f(0.0, 1500.0, 0.0);
      glEnd();

      // z axis
      glColor3f(0.0, 0.0, 1.0);
      glBegin(GL_LINES);
      glVertex3f(0.0, 0.0, 0.0);
      glVertex3f(0.0, 0.0, 1500.0);
      glEnd(); 
      
      glDisable(GL_COLOR_MATERIAL);
    }
    glPopMatrix();

    // Material spheres drawing
    glPushMatrix();
    {
      glPushMatrix();
      glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, mat2_ambient);
      glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, mat2_diffuse);
      glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, mat2_specular);
      glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, mat2_shininess);
      glTranslatef(0.0, 200.0, -150.0);
      glutSolidSphere(20.0, 16, 16);
      glPopMatrix();
      
      glPushMatrix();
      glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, mat3_ambient);
      glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, mat3_diffuse);
      glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, mat3_specular);
      glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, mat3_shininess);
      glTranslatef(142.66, 200.0, -46.35);
      glutSolidSphere(20.0, 16, 16);
      glPopMatrix();
      
      glPushMatrix();
      glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, mat4_ambient);
      glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, mat4_diffuse);
      glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, mat4_specular);
      glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, mat4_shininess);
      glTranslatef(-142.66, 200.0, -46.35);
      glutSolidSphere(20.0, 16, 16);
      glPopMatrix();
      
      glPushMatrix();
      glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, mat5_ambient);
      glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, mat5_diffuse);
      glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, mat5_specular);
      glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, mat5_shininess);
      glTranslatef(-88.17, 200.0, 121.35);
      glutSolidSphere(20.0, 16, 16);
      glPopMatrix();
      
      glPushMatrix();
      glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, mat6_ambient);
      glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, mat6_diffuse);
      glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, mat6_specular);
      glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, mat6_shininess);
      glTranslatef(88.17, 200.0, 121.35);
      glutSolidSphere(20.0, 16, 16);
      glPopMatrix();
    }
    glPopMatrix();
        

    glPushMatrix();
    /* Draw scene here */
    glLineWidth(1.0);
    //glScalef(0.2, 0.2, 0.2);
    Vector3f eyeVec(eye[0], eye[1], eye[2]);
    //eyeVec = (1.0 / 0.2) * eyeVec; // handling glScalef
    scene->drawScene(useBSP, eyeVec);
    glPopMatrix();

    glPushMatrix();
    glLightfv(GL_LIGHT0, GL_POSITION, light0Pos);
    //glLightfv(GL_LIGHT1, GL_POSITION, light1Pos);
    //glLightfv(GL_LIGHT2, GL_POSITION, light2Pos);
    //glLightfv(GL_LIGHT3, GL_POSITION, light3Pos);
    glPopMatrix();

    glutSwapBuffers();
}

void resize(int w, int h)
{
    width = w; height = h;
    glViewport(0, 0, w, h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(fovy, (GLfloat)w / (GLfloat)h, zNear, zFar);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

unsigned timeStep = 30;
void Timer(int unused)
{
	glutPostRedisplay();
	timeElapsed += 0.03;
	glutTimerFunc(timeStep, Timer, 0);
}


int main(int argc, char *argv[])
{
  // Calculate normal vector for model
  // calcModelNormal();
  
  // Scene generation
  
  vector<GModel*> mlist;
  mlist.push_back(makeBackground(200.0));
  
  mlist[0]->setMaterial(GL_AMBIENT, tr1_ambient);
  mlist[0]->setMaterial(GL_DIFFUSE, tr1_diffuse);
  mlist[0]->setMaterial(GL_SPECULAR, tr1_specular);
  mlist[0]->setShininess(tr1_shininess);
  mlist[0]->setOpaqueness(true);

  mlist.push_back(makeCube(2.0, 48.0, 2.0, 82.0, 128.0, 82.0));

  mlist[1]->setMaterial(GL_AMBIENT, mat2_ambient);
  mlist[1]->setMaterial(GL_DIFFUSE, mat2_diffuse);
  mlist[1]->setMaterial(GL_SPECULAR, mat2_specular);
  mlist[1]->setShininess(mat2_shininess);
  mlist[1]->setOpaqueness(true);

  mlist.push_back(makeCube(55.0, 25.0, 55.0, 125.0, 95.0, 125.0));
  
  mlist[2]->setMaterial(GL_AMBIENT, mat3_ambient);
  mlist[2]->setMaterial(GL_DIFFUSE, mat3_diffuse);
  mlist[2]->setMaterial(GL_SPECULAR, mat3_specular);
  mlist[2]->setShininess(mat3_shininess);
  mlist[2]->setOpaqueness(true);

  mlist.push_back(makeSphere(70.0, -60.0, 70.0, 120.0));

  mlist[3]->setMaterial(GL_AMBIENT, mat4_ambient);
  mlist[3]->setMaterial(GL_DIFFUSE, mat4_diffuse);
  mlist[3]->setMaterial(GL_SPECULAR, mat4_specular);
  mlist[3]->setShininess(mat4_shininess);
  mlist[3]->setOpaqueness(true);
  
  scene = new GScene(mlist);

  // Option parsing
  string see_in_GL("--GL");
  if(argc >= 2 && see_in_GL.compare(argv[1]) == 0)
  {
    // OpenGL Stuffs
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
    glutInitWindowSize(width, height);
    glutInitWindowPosition(80, 60);
    glutCreateWindow("[Graphics HW5] 2014-11111");

    // Setup the lighting
    lightSetup();

    // Setup the shading Model
    glShadeModel(GL_SMOOTH);

    glutReshapeFunc(resize);
    glutDisplayFunc(display);
    glutMouseFunc(glutMouse);
    glutMotionFunc(glutMotion);
    glutKeyboardFunc(glutKeyboardInput);
    glutTimerFunc(timeStep, Timer, 0);

    glEnable(GL_DEPTH_TEST);

    glutMainLoop();

    return 0;
  }

  // Shooting Rays
}
