//
// Created by Alexey Simonov on 06/08/2017.
//

#ifndef PATH_PLANNING_JERKMINIMIZINGPOLYNOMIAL_H
#define PATH_PLANNING_JERKMINIMIZINGPOLYNOMIAL_H

#include <vector>

// class that implements jerk minimizing polynomial in 1D f(t)=u
class JerkMinimizingPolynomial {
public:
    // constructor takes start_cond (vector of u,u_dot,u_dot_dot for t=0) and end_cond for t=T
    JerkMinimizingPolynomial(std::vector<double> start_cond, std::vector<double> end_cond, double T);
    double eval(double t) const;
private:
    JerkMinimizingPolynomial();
    std::vector<double> _start_cond;
    std::vector<double> _end_cond;
    double _T;
    std::vector<double> _coeff; // fitted coefficients
};


#endif //PATH_PLANNING_JERKMINIMIZINGPOLYNOMIAL_H
