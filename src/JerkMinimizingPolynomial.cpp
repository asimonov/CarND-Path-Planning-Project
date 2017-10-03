//
// Created by Alexey Simonov on 06/08/2017.
//

#include "JerkMinimizingPolynomial.h"

#include <cassert>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/Dense"

#include <iostream>

using namespace std;


// constructor takes start_cond (vector of u,u_dot,u_dot_dot for t=0) and end_cond for t=T
JerkMinimizingPolynomial::JerkMinimizingPolynomial(std::vector<double> start_cond, std::vector<double> end_cond, double T) {
  /*
  Calculate the Jerk Minimizing Trajectory that connects the initial state
  to the final state in time T.

  INPUTS

  start - the vehicles start location given as a length three array
      corresponding to initial values of [s, s_dot, s_double_dot]

  end   - the desired end state for vehicle. Like "start" this is a
      length three array.

  T     - The duration, in seconds, over which this maneuver should occur.

  OUTPUT
  an array of length 6, each value corresponding to a coefficent in the polynomial
  s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5

  EXAMPLE

  > JMT( [0, 10, 0], [10, 10, 0], 1)
  [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
  */
  assert(start_cond.size()==3);
  assert(end_cond.size()==3);
  assert(T>0);

  _start_cond = start_cond;
  _end_cond = end_cond;
  _T = T;

  double T2 = T*T;
  double T3 = T2*T;
  double T4 = T2*T2;
  double T5 = T4*T;

  Eigen::MatrixXd A = Eigen::MatrixXd(3,3);
  A << T3,   T4,    T5,
       3*T2, 4*T3,  5*T4,
       6*T,  12*T2, 20*T3;

  Eigen::VectorXd b = Eigen::VectorXd(3);
  // start = [s, s_dot, s_double_dot]_i
  // end  = [s, s_dot, s_double_dot]_f
  b << _end_cond[0] - (_start_cond[0] + _start_cond[1]*T + 0.5*_start_cond[2]*T2),
       _end_cond[1] - (_start_cond[1] + _start_cond[2]*T),
       _end_cond[2] -  _start_cond[2];

  Eigen::MatrixXd Ai = A.inverse();

  Eigen::MatrixXd C = Ai*b;

  vector <double> result = {_start_cond[0], _start_cond[1], .5*_start_cond[2]};
  for(int i = 0; i < C.size(); i++)
  {
    result.push_back(C.data()[i]);
  }
  _coeff = result;
}


double JerkMinimizingPolynomial::eval(double t) const {
  double t2 = t*t;
  double t3 = t2*t;
  double t4 = t2*t2;
  double t5 = t2*t3;
  double res = _coeff[0] + _coeff[1] * t + _coeff[2] * t2 + _coeff[3] * t3 + _coeff[4] * t4 + _coeff[5] * t5;
  return res;
}


