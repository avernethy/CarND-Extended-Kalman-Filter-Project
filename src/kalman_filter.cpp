#include "kalman_filter.h"
#include <iostream>
//#include <math.h>
//#include <stdlib.h>

#define PI 3.14159265

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in){
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
  //xprime = F * x + noise
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
 // z = H * x + w
 // Update Step
 VectorXd z_pred = H_ * x_;
 VectorXd y = z - z_pred;
 MatrixXd Ht = H_.transpose();
 MatrixXd S = H_ * P_ * Ht + R_;
 MatrixXd Si = S.inverse();
 MatrixXd K = P_ * Ht * Si;

//new state
x_ = x_ + (K * y);
MatrixXd I = MatrixXd::Identity(4, 4);
P_ = (I - K * H_) * P_;

}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
 
 // y = z - h(x') use Hj instead of H ()
 // S = H * P' Ht + R
 // K = P' *  Ht * Si
 // x = x' * K * y
 // reminder z = [rho phi rhoDot]

 VectorXd z_pred(3);
 z_pred << 0, 0, 0;
 
 float px = x_[0];
 float py = x_[1];
 float vx = x_[2];
 float vy = x_[3];
 float rho_pred = sqrt(px*px + py*py);
 
 z_pred[0] = rho_pred;
 z_pred[1] = atan2(py , px);
 
 if (abs(rho_pred) < 0.0001){
   z_pred[2] = z[2];
 }
 else{
   z_pred[2] = (py*vy + px*vx) / rho_pred;
 }

 VectorXd y = z - z_pred; // need to be in polar coordinates
 
 //angle normalization.  Keep adding or subtracting 2*PI until the value is between -PI and PI
 while (y[1] >    PI) y[1] -=2.*PI;    
 while (y[1] <   -PI) y[1] +=2.*PI;

 //same as for Kalman Filter
 MatrixXd Ht = H_.transpose();
 MatrixXd S = H_ * P_ * Ht + R_;
 MatrixXd Si = S.inverse();
 MatrixXd K = P_ * Ht * Si;

//new state
x_ = x_ + (K * y);
MatrixXd I = MatrixXd::Identity(4, 4);
P_ = (I - K * H_) * P_; 
}
