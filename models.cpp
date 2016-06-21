#include <models.h>

/////////////////////////////////////////
//                                     //
// class GSurface implementation       //
//                                     //
/////////////////////////////////////////

GSurface::GSurface(void)
{
}

GSurface::GSurface(vector<Vector3f> &vlist)
{
  assert(vlist.size() >= 3);

  _vertices = vlist;

  Vector3f vec1 = vlist[1] - vlist[0];
  Vector3f vec2 = vlist[2] - vlist[0];

  _surf_normal = vec1.cross(vec2);

  if(!_surf_normal.isZero(1e-4))
    _surf_normal.normalize();

  // Set plane equation constant
  _D = -(_surf_normal.dot(_vertices[0]));
  // DEBUG
  //cout << "_D: " << _D << endl;

  // Check other vertices are on the same plane.
  for(int i = 3; i < vlist.size(); i++)
  {
    int is_on_surface = checkPointPosition(vlist[i]);
    //assert(is_on_surface == 0);
  }
}

GSurface::GSurface(Vector3f p1, Vector3f p2, Vector3f p3)
{
  _vertices.push_back(p1);
  _vertices.push_back(p2);
  _vertices.push_back(p3);

  Vector3f vec1 = p2 - p1;
  Vector3f vec2 = p3 - p1;

  _surf_normal = vec1.cross(vec2);

  if(!_surf_normal.isZero(1e-4))
    _surf_normal.normalize();

  _D = -(_surf_normal.dot(p1));
  // DEBUG
  //cout << "_D: " << _D << endl;
  
}

void GSurface::drawSurface()
{
  assert(_vertices.size() == _vertex_normals.size());
  assert(_vertices.size() >= 3);
  assert(_model != NULL);

  if(_vertices.size() == 3) glBegin(GL_TRIANGLES);
  else glBegin(GL_POLYGON);

  _model->applyMaterial();

  for(int i = 0; i < _vertices.size(); i++)
  {
    glNormal3fv(_vertex_normals[i].data());
    glVertex3fv(_vertices[i].data());
  }

  glEnd();
}

GSurface* GSurface::splitBySurface(GSurface *other, GSurface **backSurf)
{
  vector<Vector3f> frontVertices;
  vector<Vector3f> frontNormals;
  vector<Vector3f> backVertices;
  vector<Vector3f> backNormals;

  bool frontContext = true;

  //cout << "other: " << other->toString() << endl;

  if(other->checkPointPosition(_vertices[0]) >= 0)
  {
    frontContext = true;
    frontVertices.push_back(_vertices[0]);
    frontNormals.push_back(_vertex_normals[0]);
    //cout << "First point in front: " << _vertices[0][0] << ", " << _vertices[0][1] << ", " << _vertices[0][2];
    //cout << " [inner product value: " << (_vertices[0].dot(other->getSurfaceNormal()) + other->getDConstant()) << "]" << endl;
  }
  else
  {
    frontContext = false;
    backVertices.push_back(_vertices[0]);
    backNormals.push_back(_vertex_normals[0]);
    //cout << "First point in back: " << _vertices[0][0] << ", " << _vertices[0][1] << ", " << _vertices[0][2] << endl;
    //cout << " [inner product value: " << (_vertices[0].dot(other->getSurfaceNormal()) + other->getDConstant()) << "]" << endl;
  }

  int _nv = _vertices.size();

  for(int i = 0; i < _nv; i++)
  {
    if((frontContext == true && other->checkPointPosition(_vertices[(i+1)%_nv]) < 0) ||
      (frontContext == false && other->checkPointPosition(_vertices[(i+1)%_nv]) >= 0))
    {
      float d1 = fabs(_vertices[i].dot(other->getSurfaceNormal()) + other->getDConstant());
      float d2 = fabs(_vertices[(i+1)%_nv].dot(other->getSurfaceNormal()) + other->getDConstant());
      float t = d1/(d1+d2);
      float one_minus_t = 1.0 - t;
      Vector3f dividePoint = (one_minus_t * _vertices[i]) + (t * _vertices[(i+1)%_nv]);
      Vector3f divideNormal = (one_minus_t * _vertex_normals[i]) + (t * _vertex_normals[(i+1)%_nv]);

      frontVertices.push_back(dividePoint);
      backVertices.push_back(dividePoint);

      frontNormals.push_back(divideNormal);
      backNormals.push_back(divideNormal);

      frontContext = !frontContext;
      // cout << "Context change occured!" << endl; // DEBUG
    }

    if(i+1 != _nv)
    {
      if(frontContext == true)
      {
        frontVertices.push_back(_vertices[i+1]);
        frontNormals.push_back(_vertex_normals[i+1]);
      }
      else
      {
        backVertices.push_back(_vertices[i+1]);
        backNormals.push_back(_vertex_normals[i+1]);
      }
    }
  }

  // Trimming zero-length sides
  for(int i = 0; i < frontVertices.size(); i++)
  {
    if((frontVertices[i] - frontVertices[(i+1)%frontVertices.size()]).isZero(1e-4))
    {
      frontVertices.erase(frontVertices.begin() + ((i+1)%frontVertices.size()));
      frontNormals.erase(frontNormals.begin() + ((i+1)%frontNormals.size()));
    }
  }
  for(int i = 0; i < backVertices.size(); i++)
  {
    if((backVertices[i] - backVertices[(i+1)%backVertices.size()]).isZero(1e-4))
    {
      backVertices.erase(backVertices.begin() + ((i+1)%backVertices.size()));
      backNormals.erase(backNormals.begin() + ((i+1)%backNormals.size()));
    }
  }

  if(frontVertices.size() < 3)
  {
    (*backSurf) = this;
    return nullptr;
  }
  else if(backVertices.size() < 3)
  {
    (*backSurf) = nullptr;
    return this;
  }
  else
  {
    GSurface *frontSurf = new GSurface(frontVertices);
    (*backSurf) = new GSurface(backVertices);

    frontSurf->setModel(_model);
    (*backSurf)->setModel(_model);

    for(int i = 0; i < frontVertices.size(); i++)
      frontSurf->pushNormal(frontNormals[i]);

    for(int i = 0; i < backVertices.size(); i++)
      (*backSurf)->pushNormal(backNormals[i]);

    return frontSurf;
  } 
}

int GSurface::checkPointPosition(Vector3f &point)
{
  float res = point.dot(_surf_normal) + _D;
  float epsilon = 1e-4;

  if(res > epsilon)
    return 1;
  else if (res < -epsilon)
    return -1;
  else
    return 0;
}

Vector3f& GSurface::getPointNormal(Vector3f &point)
{
  // TODO Implement phong shading?

  return _surf_normal; 
}

bool GSurface::checkRayHit(GRay *ray, Vector3f *hit_point)
{
  float s = -(_D + _surf_normal.dot(ray->getOrigin())) / (_surf_normal.dot(ray->getDirection()));

  Vector3f p = ray->getOrigin() + (s * ray->getDirection());

  bool flag = false;
  for(int i = 0; i < _vertices.size(); i++)
  {
    Vector3f winding = _vertices[(i+1)%_vertices.size()] - _vertices[i];
    Vector3f to_p = p - _vertices[i];

    float res = _surf_normal.dot(winding.cross(to_p));
    bool new_flag = (res > 0) ? true : false;

    if(i != 0 && flag != new_flag)
      return false;

    flag = new_flag;
  }

  (*hit_point) = p;
  return true;
}

void GSurface::pushNormal(Vector3f normal)
{
  _vertex_normals.push_back(normal);
}

string GSurface::toString(void)
{
  string surf_str = "*";
  for(int i = 0; i < _vertices.size(); i++)
  {
    if(i != 0) surf_str = surf_str + ", ";

    surf_str = surf_str + "(" + to_string((int)_vertices[i][0]) + "," + to_string((int)_vertices[i][1]) 
      + "," + to_string((int)_vertices[i][2]) + ")";
  }

  return surf_str;
}

/////////////////////////////////////////
//                                     //
// class GModel implementation         //
//                                     //
/////////////////////////////////////////


GModel::GModel()
{
}

GModel::GModel(vector<vector<Vector3f>> &vlist)
{
  vector<vector<Vector3f>> model_normal;
  for(int i = 0; i < vlist.size(); i++)
  {
    model_normal.emplace_back();
  }

  // Calculate Vertex Normal Vectors
  for(int i = 0; i < vlist.size(); i++)
  {
      for(int j = 0; j < vlist[0].size(); j++)
      {
        int _mod = vlist[0].size() - 1;
        vector<Vector3f> adjVectors;
        Vector3f point = vlist[i][j];
        Vector3f normal(0.0, 0.0, 0.0);

        if(j == _mod)
        {
          normal = model_normal[i][0]; 
          model_normal[i].push_back(normal);
          continue;
        }

        adjVectors.push_back(vlist[i][(j+1)%_mod] - point);
        if(i < vlist.size() - 1)
        {
          adjVectors.push_back(vlist[i+1][j] - point);
          adjVectors.push_back(vlist[i+1][(j-1+_mod)%_mod] - point);
        }
        adjVectors.push_back(vlist[i][(j-1+_mod)%_mod] - point);
        if(i > 0)
        {
          adjVectors.push_back(vlist[i-1][(j-1+_mod)%_mod] - point);
          adjVectors.push_back(vlist[i-1][j] - point);
        }


        int n_faces = adjVectors.size();

        for(int k = 0; k < n_faces; k++)
        {
          if((k == 0 && i == vlist.size()-1) || (k == n_faces - 1 && i == 0 ))
            continue;
          
          Vector3f face_normal = adjVectors[k].cross(adjVectors[(k+1)%n_faces]);
          if(!face_normal.isZero(1e-4))
            face_normal.normalize();

          normal = normal + face_normal;
        }

        if(!normal.isZero(1e-4))
          normal.normalize();

        model_normal[i].push_back(normal);
      }
  }

  // Create surfaces
  for(int i = 0; i < vlist.size() - 1; i++)
  {
    for(int j = 0; j < vlist[0].size() - 1; j++)
    {
      GSurface *surf1 = new GSurface(vlist[i][j], vlist[i][j+1], vlist[i+1][j]);
      surf1->pushNormal(model_normal[i][j]);
      surf1->pushNormal(model_normal[i][j+1]);
      surf1->pushNormal(model_normal[i+1][j]);

      surf1->setModel(this);
      _surfaces.push_back(surf1);

      GSurface *surf2 = new GSurface(vlist[i][j+1], vlist[i+1][j+1], vlist[i+1][j]);
      surf2->pushNormal(model_normal[i][j+1]);
      surf2->pushNormal(model_normal[i+1][j+1]);
      surf2->pushNormal(model_normal[i+1][j]);

      surf2->setModel(this);
      _surfaces.push_back(surf2);
    }
  } 
}

GModel::GModel(vector<GSurface*> &slist)
{
  _surfaces = slist; // copy assignment
}

void GModel::setScene(GScene *scene)
{
  _parent_scene = scene;
}

void GModel::setMaterial(GLenum pname, const GLfloat *params)
{
  switch(pname)
  {
    case GL_AMBIENT:
      copy(params, params+4, _ambient);
      break;
    case GL_DIFFUSE:
      copy(params, params+4, _diffuse);
      break;
    case GL_SPECULAR:
      copy(params, params+4, _specular);
      break;
    case GL_EMISSION:
      copy(params, params+4, _emission);
      break;
    default:
      break;
  }
}

void GModel::setShininess(GLfloat value)
{
  _shininess = value;
}

void GModel::setOpaqueness(bool isOpaque)
{
  _flag_opaque = isOpaque;
}

void GModel::setSphere(float radius, Vector3f &center)
{
  _radius = radius;
  _center = center;
  _flag_sphere = true;
}

Color GModel::getAmbient(float I_a)
{
  Color a(_ambient[0], _ambient[1], _ambient[2]);
  return I_a * a;
}

Color GModel::getDiffuse(float I_l)
{
  Color d(_diffuse[0], _diffuse[1], _diffuse[2]);
  return I_l * d;
}

Color GModel::getSpecular(float I_l)
{
  Color s(_specular[0], _specular[1], _specular[2]);
  return I_l * s;
}

float GModel::getShininess(void)
{
  return _shininess;
}

void GModel::drawModel(void)
{
  if(isSphere())
  {
    glPushMatrix();
    applyMaterial();
    glTranslatef(_center[0], _center[1], _center[2]);
    glutSolidSphere(_radius, 16, 16);
    glPopMatrix();
  }
  else
  {
    for(vector<GSurface*>::const_iterator it = _surfaces.begin(); it != _surfaces.end(); it++)
    {
      (*it)->drawSurface();
    }
  }
}

void GModel::applyMaterial(void)
{
  glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, _ambient);
  glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, _diffuse);
  glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, _specular);
  glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, _emission);
  glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, _shininess);
}

void GModel::addSurface(GSurface *surf)
{
  surf->setModel(this);
  _surfaces.push_back(surf);
}

/////////////////////////////////////////
//                                     //
// class GScene implementation         //
//                                     //
/////////////////////////////////////////


GScene::GScene(void)
{
}

GScene::GScene(vector<GModel*> &mlist)
{
  _models = mlist;
  
  for(vector<GModel*>::const_iterator it = mlist.begin(); it != mlist.end(); it++)
  {
    /*
    if((*it)->isOpaque() == false) // if current model is translucent
    {
      vector<GSurface*>& surflist = (*it)->getSurfaces();
      _translucent_surfs.insert(_translucent_surfs.end(), surflist.begin(), surflist.end());
    }
    */

    (*it)->setScene(this);
  }

  // BSP Construction
  //_BSPTree = BSPNode::constructTree(_translucent_surfs);

  // DEBUG: BSP Print
  //_BSPTree->printTree("");
}

bool GScene::addModel(GModel *model)
{
  try
  {
    _models.push_back(model);
    
    if(model->isOpaque() == false)
    {
      vector<GSurface*>& surflist = model->getSurfaces();
      _translucent_surfs.insert(_translucent_surfs.end(), surflist.begin(), surflist.end());
    }
  }
  catch (...)
  {
    return false;
  }

  return true;
}

bool GScene::drawScene(bool useBSP, Vector3f &camPoint)
{
  glEnable(GL_CULL_FACE);
  glFrontFace(GL_CCW);
  
  // Draw opaque models
  for(vector<GModel*>::const_iterator it = _models.begin(); it != _models.end(); it++)
  {
    if((*it)->isOpaque() == true) 
      (*it)->drawModel();
  }

  // Draw translucent models

  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
 
  if(useBSP) 
  {
    glDisable(GL_CULL_FACE);
    if(_BSPTree != nullptr) 
      _BSPTree->drawOnBSPTree(camPoint);
  }
  else
  {
    glCullFace(GL_FRONT); // First pass : Draw only back faces
    for(vector<GSurface*>::const_iterator it = _translucent_surfs.begin(); it != _translucent_surfs.end(); it++)
      (*it)->drawSurface();
    
    
    glCullFace(GL_BACK); // Second pass : Draw only front faces
    for(vector<GSurface*>::const_iterator it = _translucent_surfs.begin(); it != _translucent_surfs.end(); it++)
      (*it)->drawSurface();
  
    glDisable(GL_CULL_FACE);
  }
  glDisable(GL_BLEND);
}

/////////////////////////////////////////
//                                     //
// class BSPNode implementation        //
//                                     //
/////////////////////////////////////////

BSPNode::BSPNode(GSurface* surf) : _surface(surf)
{
}

BSPNode* BSPNode::constructTree(vector<GSurface*> &surfaces)
{
  vector<GSurface*> frontSurfaces;
  vector<GSurface*> backSurfaces;

  if(surfaces.empty()) return nullptr;

  BSPNode *root = new BSPNode(surfaces[0]);
  for(int i = 1; i < surfaces.size(); i++)
  {
    GSurface *frontPart, *backPart;
    frontPart = surfaces[i]->splitBySurface(surfaces[0], &backPart);

    if(frontPart != nullptr) frontSurfaces.push_back(frontPart);
    if(backPart != nullptr) backSurfaces.push_back(backPart);
  }

  root->setFrontChild(constructTree(frontSurfaces));
  root->setBackChild(constructTree(backSurfaces));

  return root;
}

void BSPNode::drawOnBSPTree(Vector3f &camPoint)
{
  int relPos = _surface->checkPointPosition(camPoint);
  
  if(relPos >= 0) // if cam is in front side.
  {
    if(_backChild != nullptr) _backChild->drawOnBSPTree(camPoint);
    _surface->drawSurface();
    if(_frontChild != nullptr) _frontChild->drawOnBSPTree(camPoint);
  }
  else
  {
    if(_frontChild != nullptr) _frontChild->drawOnBSPTree(camPoint);
    _surface->drawSurface();
    if(_backChild != nullptr) _backChild->drawOnBSPTree(camPoint);
  }
}

void BSPNode::printTree(string indent)
{
  cout << indent << _surface->toString() << endl;

  if(_frontChild != nullptr) _frontChild->printTree(indent + " F");
  if(_backChild != nullptr) _backChild->printTree(indent + " B");
}

BSPNode::~BSPNode(void)
{
  if(_frontChild != nullptr) delete _frontChild;
  if(_backChild != nullptr) delete _backChild;
  
  if(_surface != nullptr) delete _surface;
}


/////////////////////////////////////////
//                                     //
// class GRay implementation           //
//                                     //
/////////////////////////////////////////

GRay::GRay(Vector3f &origin, Vector3f &direction)
{
  _origin = origin;
  _direction = direction;

  float epsilon = 1e-4;

  if(fabs(_direction.norm() - 1.0) > epsilon)
    _direction.normalize();
}

Color GRay::TraceRay(GModel* target, int max_depth, bool *was_hit)
{
  bool is_sphere = target->isSphere();

  if(max_depth == 0)
  {
    *was_hit = false;
    return Color::Zero();
  }

  // Find intersection point
  Vector3f ic_point;
  Vector3f ic_normal;
  Color local_color;
  if(is_sphere)
  {
    Vector3f delta_p = target->getCenter() - _origin;
    float u_dot_delta_p = _direction.dot(delta_p);
    float r = target->getRadius();

    float discriminant = (u_dot_delta_p * u_dot_delta_p) 
                        - (delta_p.squaredNorm() - r * r);

    if(discriminant < 0.0) // if not hit
    {
      *was_hit = false;
      return Color::Zero();
    }
    else
    {
      *was_hit = true;
      float s = u_dot_delta_p - sqrt(discriminant);
      ic_point = _origin + (s * _direction);
    }
  }
  else
  {
    // For all surfaces in this model, check whether the ray hits one of those.
    vector<GSurface*> &surflist = target->getSurfaces();

    *was_hit = false;
    for(vector<GSurface*>::iterator it = surflist.begin(); it != surflist.end(); it++)
    {
      if((*it)->checkRayHit(this, &ic_point))
      {
        *was_hit = true;
        ic_normal = (*it)->getPointNormal(ic_point);
        break;        
      }
    }

    if(*was_hit == false) 
      return Color::Zero();
  }

  if(max_depth < 0) 
    return Color::Zero(); // returning the result of hit/not hit, for shadow rays.

  // Shadow Ray Casting
  vector<GLight*> activeLight;
  vector<GModel*> mlist = target->getScene()->getModels();

  for(vector<GLight*>::iterator it = GLight::LightList.begin(); it != GLight::LightList.end(); it++)
  {
    Vector3f dir = (*it)->GetPosition() - ic_point;
    GRay shadow_ray(ic_point, dir);
    bool result = false;
    for(vector<GModel*>::iterator mit = mlist.begin(); mit != mlist.end(); mit++)
    {
      shadow_ray.TraceRay((*mit), -1, &result); // shadow ray casting
      if(result == true)
      {
        if((*mit)->isOpaque()) 
          break;
        else 
          result = false;        
      }
    }

    if(result == false)
      activeLight.push_back(*it);
  }

  // Phong Ilumination
  float I_a = 0.2;
  if(is_sphere)
    ic_normal = (ic_point - target->getCenter()).normalized();
  
  Color ambient_color = target->getAmbient(I_a);
  Color diffuse_color = Color::Zero();
  Color specular_color = Color::Zero();

  for(vector<GLight*>::iterator it = activeLight.begin(); it != activeLight.end(); it++)
  {
    Vector3f N = ic_normal;
    Vector3f L = ((*it)->GetPosition() - ic_point).normalized();
    Vector3f R = ((2*L).dot(N))*N - L;
    Vector3f V = _direction;

    diffuse_color += (N.dot(L)) * target->getDiffuse((*it)->GetIntensity());
    specular_color += pow(R.dot(V), target->getShininess()) * target->getSpecular((*it)->GetIntensity());
  }

  local_color = ambient_color + diffuse_color + specular_color;    

  // TODO Reflection
  
  // TODO Refraction
  
  // Return result

  return local_color;
}

/////////////////////////////////////////
//                                     //
// class GLight implementation         //
//                                     //
/////////////////////////////////////////

GLight::GLight(Vector3f &pos, float intensity)
{
  _position = pos;
  _intensity = intensity;
}

vector<GLight*> GLight::LightList;

Vector3f& GLight::GetPosition(void)
{
  return _position;
}

float GLight::GetIntensity(void)
{
  return _intensity;
}
