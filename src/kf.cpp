#include <iostream>

#include "kf.h"

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}



void KalmanFilter::init(const Eigen::Vector2d &x0, const Eigen::Matrix2d &p, double sigmaA, double sensorStdDev)
{
    xHat = x0;
    this->P = p;
    this->sigmaA = sigmaA;
    dt = 0;
    H << 1, 0;
    R << std::pow(sensorStdDev,2.0);
    I.setIdentity();
    initialized = true;
    std::cout<< "Kalman Filter initialized"<< std::endl;

}

void KalmanFilter::update( double dt, const Eigen::VectorXd &z)
{
    if(!initialized)
    {
        throw std::runtime_error("Filter has not been initialized!");
    }
    this->dt = dt;
    F << 1, this->dt, 
        0, 1;
    G << 0.5*dt*dt, dt;
    Q = G*G.transpose()*(sigmaA*sigmaA);
    
    // Predict Step
    xHatNew = F * xHat;
    P = F*P*F.transpose() + Q;

    // Update Step
    y = z - H*xHatNew;
    K = P * H.transpose()* (H*P*H.transpose() + R).inverse();
    xHatNew += K*y;
    P = (I - K*H)*P;

    xHat = xHatNew; 

}