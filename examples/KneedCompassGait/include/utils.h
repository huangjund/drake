#pragma once

#include <cmath>
#include <iomanip>
#include <iostream>
#include <fstream>
#include <map>
#include <memory>
#include <vector>

#include <Eigen/Core>
#include <unsupported/Eigen/MatrixFunctions>

#define PI 3.1416
#define G 9.81

typedef struct Action 
{
  double step_length;
  double dheading;
  double dheight;
  double v_apex;
  double h_apex;
  Action(const double _step_length = 0., const double _dheading = 0,
         const double _dheight = 0, const double _v_apex = 0,
         const double _h_apex = 0.68) 
  {
    step_length = _step_length;
    dheading = _dheading;
    dheight = _dheight;
    v_apex = _v_apex;
    h_apex = _h_apex;
  }
} Action;

