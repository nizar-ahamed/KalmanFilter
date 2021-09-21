#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include <random>

#include "kf.h"

struct data{
    double time;
    double posX;
};

/*************************************************************
 * param in  - file to load data from (to be implemented)
 * param out - vector containing timestamp and position
 * remarks   - data is assumed to be stored in csv format - timestamp, posX (in m)
 * **********************************************************/
void loadData(std::vector<data> &out, const std::string& filename)
{
    std::string line;
    std::ifstream stream("../data/" + filename); 
    if (stream.is_open())
    {
        while (std::getline(stream, line))
        {
            std::stringstream ss(line);

            std::string stime, sposX;
            std::getline(ss, stime, ',');
            std::getline(ss, sposX, '\n');

            data temp;
            temp.time = std::stod(stime);
            temp.posX = std::stod(sposX);

            out.emplace_back(temp);
        }
    }
}

/*********************************************************
 * param in  - standard deviation of noise to be added, reference 
 *             of vector with timestamp and data
 * param out - vector with added noise in posX
 * remarks   - 
 * *******************************************************/
void addNoise(std::vector<data> &in, const double stdDev, std::vector<data> &out)
{
    const double mean = 0.0;
    std::default_random_engine generator;
    std::normal_distribution<double> dist(mean,stdDev);

    for (auto& x : in)
    {
        data temp;
        temp.time = x.time;
        temp.posX = x.posX + dist(generator); 
        out.emplace_back(temp);
    }
}

int main()
{
    std::vector<data> camData;
    std::vector<data> noisyCamData;
    std::string camFile1 = "cam_data1.txt";
    loadData(camData, camFile1);

    double sensorStdDev = 2.0;
    addNoise(camData, sensorStdDev, noisyCamData);

    KalmanFilter kf;
    Eigen::Vector2d x0;             // initial state estimate
    x0 << noisyCamData[0].posX, 0;
    Eigen::Matrix2d P0;             // initial estimate covariance
    P0 << 10, 0,
          0, 10;

    double sigmaA = 3.4;            // std dev of external acceleration - to be used for process noise co-variance
    

    kf.init(x0, P0, sigmaA, sensorStdDev);

    double kfSquaredErrorSum = std::pow((noisyCamData[0].posX - camData[0].posX),2.0);
    double measSquaredErrorSum = std::pow((noisyCamData[0].posX - camData[0].posX),2.0);

    for (int i = 1; i<noisyCamData.size(); i++)
    {
        double dt = (noisyCamData[i].time - noisyCamData[i-1].time)/1000.0;
        Eigen::Vector<double, 1> z;   // measurement
        z << noisyCamData[i].posX;
        kf.update(dt, z);

        kfSquaredErrorSum += std::pow((kf.getFilteredPosX() - camData[i].posX),2.0);

        measSquaredErrorSum += std::pow((noisyCamData[i].posX - camData[i].posX),2.0);
    }

    double kfMeanSquaredError = kfSquaredErrorSum/camData.size();
    double measMeanSquaredError = measSquaredErrorSum/camData.size();

    std::cout << " Filter output mean squared error is " << kfMeanSquaredError << std::endl;
    std::cout << " Measurement mean squared error is " << measMeanSquaredError << std::endl;
}