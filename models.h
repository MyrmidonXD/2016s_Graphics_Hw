#ifndef __HW4_MODELS_H__
#define __HW4_MODELS_H__

// Eigen Library
#include <Eigen/Dense>
#include <Eigen/Geometry>

// OpenGL Related
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/freeglut.h>

// Other Libraries
#include <cstdio>
#include <cassert>
#include <cmath>
#include <iostream>
#include <vector>
#include <string>

using namespace std;
using namespace Eigen;

class GSurface;
class GModel;
class GScene;
class BSPNode;

// Class representing a surface of the model
class GSurface 
{
  public:
    // Constructors
    GSurface(void);                                   // Defualt Constructor
    GSurface(vector<Vector3f>& vlist);                // Vertex Vector Constructor
    GSurface(Vector3f p1, Vector3f p2, Vector3f p3);  // 3 Vertices Constructor

    // Draw this surface
    // @retval true if successfully draw this surface, false otherwise. 
    virtual void drawSurface();

    // Split this surface by other surface. Used for BSP tree construction.
    // @retval true if successfully splitted, false otherwise.
    virtual GSurface* splitBySurface(GSurface *other, GSurface **backSurf);

    // Check the position of the point with this surface
    // @retval 0 if point is on the surface, >0 if point is in the front, 
    //        <0 if point is in the back.
    int checkPointPosition(Vector3f &point);

    // Set parent model to get material info.
    void setModel(GModel *model) { _model = model; }

    // Push normal vecter to inner normal vector list
    void pushNormal(Vector3f normal);

    // Make the representative string of this surface
    string toString(void);

    // Getters
    int getNVertices() { return _vertices.size(); }
    Vector3f& getVertex(int i) { return _vertices.at(i); }
    Vector3f& getVertexNormal(int i) { return _vertex_normals.at(i); }
    Vector3f& getSurfaceNormal(void) { return _surf_normal; }
    float getDConstant() { return _D; }

  private:
    vector<Vector3f> _vertices;
    vector<Vector3f> _vertex_normals;
    Vector3f _surf_normal;

    float _D; // Plane equation constant: Ax+By+Cz+'_D' = 0

    GModel *_model;
};

// Class representing a model
class GModel
{
  public:
    // Constructors
    GModel(void); // Default Constructor
    GModel(vector<vector<Vector3f>>& vlist); // Vertex list constructor (for swept surface models)
    GModel(vector<GSurface*> &slist); // Surface list constructor
   
    // Setters
    void setMaterial(GLenum pname, const GLfloat* params); // Set Material values.
    void setShininess(GLfloat value); // Set Material Shininess (= glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, value);)
    void setOpaqueness(bool isOpaque); // Set opaque flag to the value of isOpaque.

    // Getters
    bool isOpaque(void) { return _flag_opaque; } // Get flag for opaqueness (true if opaque)
    vector<GSurface*>& getSurfaces(void) { return _surfaces; }

    // Draw this model
    void drawModel(void);

    void applyMaterial(void); // Apply material coefficients to OpenGL.

    void addSurface(GSurface *surf); // Add the surface to this model.

  private:
    // Material properties
    GLfloat _ambient[4] = { 0.2, 0.2, 0.2, 1.0 };
    GLfloat _diffuse[4] = { 0.8, 0.8, 0.8, 1.0 };
    GLfloat _specular[4] = { 0.0, 0.0, 0.0, 1.0 };
    GLfloat _emission[4] = { 0.0, 0.0, 0.0, 1.0 };
    GLfloat _shininess = 0.0;

    // Model properties
    bool _flag_opaque = true;

    // Surface list
    vector<GSurface*> _surfaces;

};

// class representing the whole scene objects
// This is the set of all models.
class GScene
{
  public:
    // Constructors
    GScene(void); // Default Constructor
    GScene(vector<GModel*> &mlist); // Model list Constructor

    // Add a model to the scene.
    bool addModel(GModel *model);

    // Draw a scene.
    bool drawScene(bool useBSP, Vector3f &camPoint);

  private:
    vector<GModel*> _models;
    vector<GSurface*> _translucent_surfs;

    BSPNode* _BSPTree;
};

class BSPNode
{
  public:
    // Constructor
    BSPNode(GSurface* surf);

    // Tree Consturct
    static BSPNode* constructTree(vector<GSurface*> &surfaces);

    // Child Setters
    void setFrontChild(BSPNode *fc) { _frontChild = fc; }
    void setBackChild(BSPNode *bc) { _backChild = bc; }

    // Draw surfaces following this BSP tree.
    void drawOnBSPTree(Vector3f &camPoint);

    // Print BSP tree for debugging
    void printTree(string indent);

    // Destructor
    ~BSPNode();

  private:
    GSurface *_surface;
    
    BSPNode *_frontChild;
    BSPNode *_backChild;
};

#endif
