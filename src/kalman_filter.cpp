#include "kalman_filter.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in){
                       // MatrixXd &H_laser_in, MatrixXd &R_laser_in,
                       // MatrixXd &Hj_in, MatrixXd &R_radar_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
  //H_laser_ = H_laser_in;
  //R_laser_ = R_laser_in;
  //Hj_ = Hj_in;
  //R_radar_ = R_radar_in;
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
 cout << "CHECK R MATRIX" << endl;
 cout << R_ << endl;
 //VectorXd z_pred = H_ * x_;
 //VectorXd y = z - z_pred;
 //MatrixXd Ht = H_.transpose();
 //MatrixXd S = H_ * P_ * Ht + R_;
 //MatrixXd Si = S.inverse();
 //MatrixXd K = P_ * Ht * Si;

//new state
//x_ = x_ * (K * y);
//MatrixXd I = MatrixXd::Identity(4, 4);
//P_ = (I - K * H_) * P_;

}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
 // y = z - H * x' use Hj instead of H (already taken care of in H_ ?)
 // S = H * P' Ht + R
 // K = P' *  Ht * Si
 // x = x' * K * y
 // P = (I - K * H) * P' 

}
