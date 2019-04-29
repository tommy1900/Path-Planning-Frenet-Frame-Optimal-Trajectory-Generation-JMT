#ifndef TRAJ_H
#define TRAJ_H

#include <cstdio>
#include <cassert>
#include <vector>
#include <math.h>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <cmath>

#include "Eigen-3.3/Eigen/Dense"

using Eigen::MatrixXd;
using Eigen::VectorXd;

using namespace std;

class Traj{
public:
  Traj(){};
  ~Traj(){};
  Traj(vector<double> BC, double T);
  double getCost(){return cost;}
  double getDist(double t);
  double getVel(double t);
  double getAcel(double t);
  double getJerk(double t);

  double a0;
	double a1;
	double a2;
	double a3;
	double a4;
	double a5;
  double duration;
  double cost;

};

Traj::Traj(vector<double> BC, double T){
  if(BC.size() == 6){
    MatrixXd a(3,3);
    double T2 =  T*T,
           T3 = T2*T,
           T4 = T3*T,
           T5 = T4*T;
    a <<  T3,    T4,    T5,
        3*T2,  4*T3,  5*T4,
         6*T, 12*T2, 20*T3;
    MatrixXd aInv = a.inverse();

    VectorXd b(3);
    b << BC[3] - (BC[0] + BC[1]*T + 0.5*BC[2]*T2),
         BC[4] - (           BC[1]   +     BC[2]*T),
         BC[5] - (                            BC[2]);
    VectorXd alpha = aInv * b;

    a0 = BC[0];
    a1 = BC[1];
    a2 = 0.5 * BC[2];
    a3 = alpha[0];
    a4 = alpha[1];
    a5 = alpha[2];
    duration = T;


    // if jerk =  6*a3 + 24*a4*t, then integral(jerk^2) w.r.t t is:
    // 36*a3^2*t + 144*a3*a4*t^2 + 192*a4^2*t^3
    double jerk_int = 36*a3*a3*T + T3*(192*a4*a4 + 240*a3*a5) + 720*a5*a5*T5 + 144*a3*a4*T2 + 720*a4*a5*T4;
    double tmpCost = jerk_int + 1*T + 1*fabs(BC[0]-BC[3])*fabs(BC[0]-BC[3]);
    cost = tmpCost;
  }else{
    MatrixXd a(2,2);
    double T2 =  T*T,
           T3 = T2*T;

    a << 3*T2,  4*T3,
         6*T, 12*T2;
    MatrixXd aInv = a.inverse();

    VectorXd b(2);
    b << BC[3] - (           BC[1]   +     BC[2]*T),
         BC[4] - (                            BC[2]);
    VectorXd alpha = aInv * b;

    a0 = BC[0];
    a1 = BC[1];
    a2 = 0.5 * BC[2];
    a3 = alpha[0];
    a4 = alpha[1];
    a5 = 0;
    duration = T;

    // if jerk =  6*a3 + 24*a4*t, then integral(jerk^2) w.r.t t is:
    // 36*a3^2*t + 144*a3*a4*t^2 + 192*a4^2*t^3
    double jerk_int = 36*a3*a3*T + 144*a3*a4*T2 + 192*a4*a4*T3;
    double tmpCost = jerk_int + 1*T + 5*fabs(BC[0]-BC[3])*fabs(BC[0]-BC[3]);
    cost = tmpCost;
  }
}

double Traj::getDist(double t)
{
	if (t > duration)
	{
		double dist = a0 + a1*duration + a2*duration*duration + a3*duration*duration*duration + a4*duration*duration*duration*duration + a5*duration*duration*duration*duration*duration;
		double vel = a1 + 2.*a2*duration + 3.*a3*duration*duration + 4.*a4*duration*duration*duration + 5.*a5*duration*duration*duration*duration;

		return (dist + vel*(t-duration));
	}
	else
	{
		return (a0 + a1*t + a2*t*t + a3*t*t*t + a4*t*t*t*t + a5*t*t*t*t*t);
	}
}

double Traj::getVel(double t)
{
	return a1 + 2.*a2*t + 3.*a3*t*t + 4.*a4*t*t*t + 5.*a5*t*t*t*t;
}

double Traj::getAcel(double t)
{
  return 2.*a2 + 6.*a3*t + 12.*a4*t*t + 20.*a5*t*t*t;
}

double Traj::getJerk(double t)
{
	if (t > duration)
	{
		return(0.);
	}
	else
	{
		return (6.*a3 + 24.*a4*t + 60.*a5*t*t);
	}
}
#endif
