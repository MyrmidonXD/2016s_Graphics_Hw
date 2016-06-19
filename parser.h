#ifndef __PARSER_H__
#define __PARSER_H__

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include <cmath>
#include <cstdio>

using namespace std;
using namespace Eigen;

class Parser {
  public:
    Parser(char *filename);
    
    bool parseDatafile(char *filename);
    
    vector<Vector3f>&     getPositions(void);
    vector<float>&        getScales(void);
    vector<Quaternionf>&  getRotations(void);
    string                getSplineType(void);
    int                   getSectionNum(void);
    int                   getCPointNum(void);

    // inner vector<Vector3f> holds a control point in all cross sections.
    vector<vector<Vector3f>>&  getCPoints(void);
    vector<Vector3f>&          getCPointsByCol(int col);

    // Get pointer of Parser singleton.
    static Parser* Get(char *filename=nullptr);

    bool isParsed(void);


  private:
    vector<Vector3f>          _positions;
    vector<float>             _scales;
    vector<Quaternionf>       _rotations;
    vector<vector<Vector3f>>  _c_points;
    string                    _spline_type;
    int                       _section_num;
    int                       _cp_num_per_section;

    Parser(void);

    ~Parser(void);

    bool _parsed = false;
    
    static Parser* _instance;

    bool _parseSection(ifstream& file, int cpnum);
};

#endif
