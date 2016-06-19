#include <spline.h>

// class Spline
Spline::Spline()
{
}

Spline::Spline(vector<Vector3f>& vlist)
{
  _vector_list = vlist;
}

Vector3f Spline::getSpline(float t, string spline_t, bool closed)
{
  if(spline_t == "CATMULL_ROM")
  {
    return _catmullRom(t, closed);
  }
  else if(spline_t == "BSPLINE")
  {
    return _bspline(t);
  }
  else
  {
    cout << "Wrong spline type provided." << endl;
  }

  Vector3f default_vec(0.0,0.0,0.0);
  return default_vec;
}
/*
Spline::~Spline()
{
  for(int i = 0; i < _vector_list.size(); i++)
    delete &(_vector_list[i]);

  delete &(_vector_list);
}
*/

Spline::~Spline()
{
}

Vector3f Spline::_catmullRom(float t, bool closed) 
{
  if(t < 0.0) t = 0.0;

  int curve_index = (int) t;
  float curve_value = t - (float)curve_index;
  int section_num = _vector_list.size();

  if(t >= (float)section_num)
  {
    curve_index = section_num - 1;
    curve_value = 1.0;
  }

  if(!closed)
  {
    if(curve_index < 1)
    {
      curve_index = 1;
      curve_value = 0.0;
    }
    else if (curve_index >= section_num-2)
    {
      curve_index = section_num-3;
      curve_value = 1.0;
    }
  }

  // Matrix multiplication for CATMULL_ROM
  Vector3f b0 = _vector_list[curve_index];
  Vector3f b3 = _vector_list[(curve_index+1) % section_num];
  Vector3f p_n = _vector_list[(curve_index+2) % section_num];
  Vector3f p_p = _vector_list[(curve_index-1+section_num) % section_num];

  Vector3f b1 = b0 + ((1.0/6.0) * (b3 - p_p));
  Vector3f b2 = b3 - ((1.0/6.0) * (p_n - b0));

  Matrix<float, 3, 4> control_point_mat;
  control_point_mat.col(0) << b0;
  control_point_mat.col(1) << b1;
  control_point_mat.col(2) << b2;
  control_point_mat.col(3) << b3;

  Matrix4f bernstein_mat;
  bernstein_mat << -1.0, 3.0, -3.0, 1.0,
                    3.0, -6.0, 3.0, 0.0,
                    -3.0, 3.0, 0.0, 0.0,
                    1.0, 0.0, 0.0, 0.0;

  Vector4f monomial(curve_value*curve_value*curve_value, 
                    curve_value*curve_value,
                    curve_value,
                    1.0
                    );

  Vector3f result = (control_point_mat * bernstein_mat) * monomial;
  return result;
}

Vector3f Spline::_bspline(float t)
{
  
  if(t < 0.0) t = 0.0;

  int curve_index = (int) t;
  float curve_value = t - (float)curve_index;
  int section_num = _vector_list.size();

  if(t >= (float)section_num)
  {
    curve_index = section_num - 1;
    curve_value = 1.0;
  }

  // Matrix multiplication for BSPLINE
  Vector3f b0 = _vector_list[curve_index];
  Vector3f b1 = _vector_list[(curve_index+1) % section_num];
  Vector3f b2 = _vector_list[(curve_index+2) % section_num];
  Vector3f b3 = _vector_list[(curve_index+3) % section_num];

  Matrix<float, 3, 4> control_point_mat;
  control_point_mat.col(0) << b0;
  control_point_mat.col(1) << b1;
  control_point_mat.col(2) << b2;
  control_point_mat.col(3) << b3;

  Matrix4f basis_mat;
  basis_mat << -1.0, 3.0, -3.0, 1.0,
                3.0, -6.0, 0.0, 4.0,
                -3.0, 3.0, 3.0, 1.0,
                1.0, 0.0, 0.0, 0.0;

  Vector4f monomial(curve_value*curve_value*curve_value, 
                    curve_value*curve_value,
                    curve_value,
                    1.0
                    );

  Vector3f result = (control_point_mat * basis_mat) * monomial;
  return result;
}

// class FloatSpline
FloatSpline::FloatSpline(vector<float>& vlist)
{
  _val_list = vlist;
}

float FloatSpline::getSpline(float t, string spline_t, bool closed)
{
  return _catmullRom(t);
}

float FloatSpline::_catmullRom(float t, bool closed)
{
  int section_num = _val_list.size();
  
  if(t < 0.0) t = 0.0;

  int curve_index = (int) t;
  float curve_value = t - (float)curve_index;

  if(t >= (float)section_num)
  {
    curve_index = section_num - 1;
    curve_value = 1.0;
  }
 
  if(!closed)
  {
    if(curve_index < 1)
    {
      curve_index = 1;
      curve_value = 0.0;
    }
    else if (curve_index >= section_num-2)
    {
      curve_index = section_num-3;
      curve_value = 1.0;
    }
  }

  float b0 = _val_list[curve_index];
  float b3 = _val_list[(curve_index+1) % section_num];
  float p_n = _val_list[(curve_index+2) % section_num];
  float p_p = _val_list[(curve_index-1+section_num) % section_num];

  float b1 = b0 + ((1.0/6.0) * (b3 - p_p));
  float b2 = b3 - ((1.0/6.0) * (p_n - b0));

  Matrix<float, 1, 4> control_point_mat;
  control_point_mat << b0, b1, b2, b3;

  Matrix4f bernstein_mat;
  bernstein_mat << -1.0, 3.0, -3.0, 1.0,
                    3.0, -6.0, 3.0, 0.0,
                    -3.0, 3.0, 0.0, 0.0,
                    1.0, 0.0, 0.0, 0.0;

  Vector4f monomial(curve_value*curve_value*curve_value, 
                    curve_value*curve_value,
                    curve_value,
                    1.0
                    );
  
  Matrix<float, 1, 1> result = (control_point_mat * bernstein_mat) * monomial;
  return result[0];

}

// class QuaternionSpline

QuaternionSpline::QuaternionSpline(vector<Quaternionf>& vlist)
{
  _quat_list = vlist;
}

Quaternionf QuaternionSpline::getSpline(float t, string spline_t, bool closed)
{
  return _catmullRom(t);
}

Quaternionf QuaternionSpline::_catmullRom(float t, bool closed)
{
  int section_num = _quat_list.size();
  
  if(t < 0.0) t = 0.0;

  int curve_index = (int) t;
  float curve_value = t - (float)curve_index;
 
  if(curve_index < 1)
  {
    curve_index = 1;
    curve_value = 0.0;
  }
  else if (curve_index >= section_num-2)
  {
    curve_index = section_num-3;
    curve_value = 1.0;
  }

  float _norm = 0.0;
  Quaternionf result;
  do {
  Quaternionf b0 = _quat_list[curve_index];
  Quaternionf b3 = _quat_list[curve_index+1];
  Quaternionf q_n = _quat_list[curve_index+2];
  Quaternionf q_p = _quat_list[curve_index-1];

  Quaternionf b1 = b0 * q_p.conjugate() * safe_slerp(1.0/6.0, q_p, b3);
  Quaternionf b2 = b3 * q_n.conjugate() * safe_slerp(1.0/6.0, q_n, b0);

  // cout << "norms : " << b0.norm() << ", " << b1.norm() << ", " << b2.norm() << ", " << b3.norm() << endl;

  // De Casteljau Algorithm
  Quaternionf l1_0 = safe_slerp(curve_value, b0, b1);
  Quaternionf l1_1 = safe_slerp(curve_value, b1, b2);
  Quaternionf l1_2 = safe_slerp(curve_value, b2, b3);

  Quaternionf l2_0 = safe_slerp(curve_value, l1_0, l1_1);
  Quaternionf l2_1 = safe_slerp(curve_value, l1_1, l1_2);

  result = safe_slerp(curve_value, l2_0, l2_1);
  _norm = result.norm();
  // cout << "norm : " << _norm << endl;
  } while (_norm != _norm); // checks NaN value

  return result;
}

Quaternionf QuaternionSpline::safe_slerp(float t, Quaternionf& from, Quaternionf& to)
{
  Quaternionf checker = from.inverse() * to;

  if(checker.w() > 0.9999)
    return from;
  else
    return from.slerp(t, to);
}
