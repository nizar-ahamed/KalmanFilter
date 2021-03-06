#ifndef kf_h
#define kf_h

#include "Eigen/Dense"

class KalmanFilter{
    public: 
        KalmanFilter();
        ~KalmanFilter();

        /*********************************************************
         * initialize the kalman filter
         * *******************************************************/
        void init(const Eigen::Vector2d &x0, const Eigen::Matrix2d &p, double sigmaA, double r);
        
        /*********************************************************
         * update the kalman filter
         * *******************************************************/
        void update( double dt, const Eigen::VectorXd &z);
        
        /*********************************************************
         * return the state vector
         * *******************************************************/
        Eigen::Vector2d getState() {return xHat;};
        
        /*********************************************************
         * return the filtered position X
         * *******************************************************/
        double getFilteredPosX() {return xHat[0];};

    private:
        double dt, sigmaA, sensorStdDev;
        int numStates = 2;
        int numMeas = 1;
        bool initialized;
        Eigen::Vector2d xHat, xHatNew, G;
        Eigen::Matrix2d F, P, Q, I;         // F - state transition matrix; P - Process covariance matrix; Q - Process noise covariance matrix; I -Identity matrix
        Eigen::Matrix<double, 1, 2> H;      // H - Measuring matrix
        Eigen::MatrixXd K;                  // K - Kalman Gain
        Eigen::Matrix<double, 1, 1> R, y;   // R - Measurement noise covariance, y - residual

};

#endif