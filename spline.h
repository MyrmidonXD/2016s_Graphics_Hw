#ifndef __SPLINE_H__
#define __SPLINE_H__

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <cmath>
#include <cstdio>

using namespace Eigen;
using namespace std;

class Spline
{
  public:
    Spline(void);
    Spline(vector<Vector3f>& vlist);

    Vector3f getSpline(float t, string spline_t="CATMULL_ROM", bool closed=false);

    ~Spline(void);
    
  private:
    Vector3f _catmullRom(float t, bool closed);
    Vector3f _bspline(float t);

    vector<Vector3f> _vector_list;
};


class QuaternionSpline : public Spline
{
  public:
    QuaternionSpline(vector<Quaternionf>& vlist);

    Quaternionf getSpline(float t, string spline_t="CATMULL_ROM", bool closed=false);
    
  private:
    Quaternionf _catmullRom(float t, bool closed=false);
    //Vector3f _bspline(float t);

    Quaternionf safe_slerp(float t, Quaternionf& from, Quaternionf& to);
    vector<Quaternionf> _quat_list;
};


class FloatSpline : public Spline
{
  public:
    FloatSpline(vector<float>& vlist);

    float getSpline(float t, string spline_t="CATMULL_ROM", bool closed=false);
    
  private:
    float _catmullRom(float t, bool closed=false);
    //Vector3f _bspline(float t);



    vector<float> _val_list;
};


#endif
