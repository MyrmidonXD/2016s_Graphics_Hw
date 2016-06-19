#include <parser.h>

using namespace std;

Parser* Parser::_instance = nullptr;

Parser::Parser(void)
{
  _positions = *(new vector<Vector3f>());
  _scales = *(new vector<float>());
  _rotations = *(new vector<Quaternionf>());
  _c_points = *(new vector<vector<Vector3f>>());
  _section_num = 0;
  _cp_num_per_section = 0;
  _spline_type = "";

  Parser::_instance = this;
}

Parser::Parser(char *filename)
{
  _positions = *(new vector<Vector3f>());
  _scales = *(new vector<float>());
  _rotations = *(new vector<Quaternionf>());
  _c_points = *(new vector<vector<Vector3f>>());
  _parsed = parseDatafile(filename);

  //cout << "Parser called!" << endl;

  Parser::_instance = this;
}

bool Parser::_parseSection(ifstream& file, int cpnum)
{
  try
  {
    string curr_line;
    // parsing control points in this section
    int i = 0;
    while(i < cpnum)
    {
      getline(file, curr_line);

      float x, z;
      int result = sscanf(curr_line.c_str(), "%f %f %*[^\n]", &x, &z);

      if(result < 2) continue;

      _c_points[i++].push_back(*(new Vector3f(x, 0.0, z)));
    }

    // parsing scale factor
    float scale;
    while(true) 
    {
      getline(file, curr_line);
      int result = sscanf(curr_line.c_str(), "%f %*[^\n]", &scale);

      if(result < 1) continue;

      _scales.push_back(scale);
      break;
    }

    // parsing rotation
    float rot_w, rot_x, rot_y, rot_z;
    while(true) 
    {
      getline(file, curr_line);
      int result = sscanf(curr_line.c_str(), "%f %f %f %f %*[^\n]", 
                          &rot_w, &rot_x, &rot_y, &rot_z);

      // cout << "parsing " << curr_line << " : " << result << endl;

      if(result < 4) continue;

      Vector3f axis(rot_x, rot_y, rot_z);
      if(axis.norm() < 0.00001) axis[0] = 1.0;
      axis.normalize();
      AngleAxisf aa(rot_w, axis);
      Quaternionf new_rot(aa);

      float _nrm;
      _nrm = new_rot.norm();
      if(_nrm != _nrm)
      {
        cout << "NaN Alert!! (in parser)" << endl;
        cout << "parsed data : ( " << rot_w << ", " << axis[0] << ", " << axis[1] << ", " << axis[2] << " )" << endl;
        cout << "section num = " << _rotations.size() << endl;
      }


      _rotations.push_back(new_rot);
      break;
    }

    // parsing position
    float pos_x, pos_y, pos_z;
    while(true) 
    {
      getline(file, curr_line);
      int result = sscanf(curr_line.c_str(), "%f %f %f %*[^\n]", 
                          &pos_x, &pos_y, &pos_z);

      if(result < 3) continue;

      Vector3f pos(pos_x, pos_y, pos_z);
      _positions.push_back(pos);
      break;
    }

  }
  catch(...)
  {
    return false;
  }

  return true;
}

bool Parser::parseDatafile(char *filename)
{
  ifstream file(filename);
  string curr_line;
  char buf[15];

  _positions.clear();
  _scales.clear();
  _rotations.clear();
  _c_points.clear();
  _section_num = 0;
  _cp_num_per_section = 0;
  _spline_type = "";
  _parsed = false;

  // spline type parsing
  while(file.good()) 
  {
    getline(file, curr_line);
    sscanf(curr_line.c_str(), "%s %*[^\n]", buf);
    _spline_type = *(new string(buf));

    if(_spline_type == "BSPLINE" || _spline_type == "CATMULL_ROM")
      break;
  }
  if(!file.good()) return false;

  // section number parsing
  while(file.good())
  {
    getline(file, curr_line);
    int result = sscanf(curr_line.c_str(), "%d %*[^\n]", &_section_num);

    if(result >= 1) break; 
  }
  if(!file.good()) return false;

  // control point number parsing
  while(file.good())
  {
    getline(file, curr_line);
    int result = sscanf(curr_line.c_str(), "%d %*[^\n]", &_cp_num_per_section);

    if(result >= 1) break;
  }
  if(!file.good()) return false;

  // initializing _c_points
  for(int i = 0; i < _cp_num_per_section; i++)
  {
    _c_points.push_back(*(new vector<Vector3f>()));
  }

  for(int i = 0; i < _section_num; i++)
  {
    bool res = _parseSection(file, _cp_num_per_section);
    if(!res) return false;
  }

  return true;
}

Parser* Parser::Get(char *filename)
{
  if(_instance == nullptr)
  {
    Parser* new_parser;
    if(filename != nullptr) new_parser = new Parser(filename);
    else new_parser = new Parser();
    return new_parser;
  }
}

bool Parser:: isParsed(void)
{
  return _parsed;
}

vector<Vector3f>& Parser::getPositions(void)
{
  return _positions;
}

vector<float>& Parser::getScales(void)
{
  return _scales;
}

vector<Quaternionf>& Parser::getRotations(void)
{
  return _rotations;
}

vector<vector<Vector3f>>& Parser::getCPoints(void)
{
  return _c_points;
}

vector<Vector3f>& Parser::getCPointsByCol(int col)
{
  return _c_points[col];
}

string Parser::getSplineType(void)
{
  return _spline_type;
}

int Parser::getSectionNum(void)
{
  return _section_num;
}

int Parser::getCPointNum(void)
{
  return _cp_num_per_section;
}


Parser::~Parser(void)
{
  for(int i = 0; i < _section_num; i++)
  {
    delete &(_positions[i]);
    delete &(_rotations[i]);
    
    for(int j = 0; j < _cp_num_per_section; j++)
      delete &(_c_points[i][j]);

    delete &(_c_points[i]);
  }

  delete &(_positions);
  delete &(_scales);
  delete &(_rotations);
  delete &(_c_points);
}



