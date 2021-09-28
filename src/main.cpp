#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include <random>

#include "pbPlots/pbPlots.hpp"
#include "pbPlots/supportLib.hpp"

#include "kf.h"


/*************************************************************
 * param in  - file to load data from
 * param out - vector containing timestamp and position
 * remarks   - data is assumed to be stored in csv format - timestamp, posX (in m)
 * **********************************************************/
void loadData(std::vector<double> &time, std::vector<double> &data, const std::string& filename)
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

            time.emplace_back(std::stod(stime));
            data.emplace_back(std::stod(sposX));
        }
    }
    stream.close();
}



/*********************************************************
 * param in  - standard deviation of noise to be added, reference 
 *             of vector with timestamp and data
 * param out - vector with added noise in posX
 * remarks   - 
 * *******************************************************/
void addNoise(std::vector<double> &in, const double stdDev, std::vector<double> &out)
{
    const double mean = 0.0;
    std::default_random_engine generator;
    std::normal_distribution<double> dist(mean,stdDev);

    for (auto& x : in)
    {
        out.emplace_back(x + dist(generator));
    }
}


int main()
{
    std::vector<double> noisyCamTime;
    std::vector<double> noisyCamData;
    std::vector<double> kfOutData;
    
    std::vector<double> camData;
    std::string camFile = "cam_data1.txt";
    loadData(noisyCamTime, camData, camFile);

    addNoise(camData, 2.0, noisyCamData);

    KalmanFilter kf;
    Eigen::Vector2d x0;             // initial state estimate
    Eigen::Matrix2d P0; 
 

    x0 << noisyCamData[0], 0;
    // initial estimate covariance
    P0 << 10, 0,
          0, 10;

    double sigmaA = 3.4;  
    double sensorStdDev = 1.0;



    kf.init(x0, P0, sigmaA, sensorStdDev);
    kfOutData.emplace_back(x0[0]);


    double kfSquaredErrorSum = std::pow((kf.getFilteredPosX() - camData[0]),2.0);
    double measSquaredErrorSum = std::pow((noisyCamData[0] - camData[0]),2.0);

    for (int i = 1; i<noisyCamData.size(); i++)
    {
        double dt = (noisyCamTime[i] - noisyCamTime[i-1])/1000.0;
        Eigen::Vector<double, 1> z;   // measurement
        z << noisyCamData[i];
        kf.update(dt, z);
        kfOutData.emplace_back(kf.getFilteredPosX());

        kfSquaredErrorSum += std::pow((kf.getFilteredPosX() - camData[i]),2.0);
        measSquaredErrorSum += std::pow((noisyCamData[i] - camData[i]),2.0);

    }

    double kfMeanSquaredError = kfSquaredErrorSum/noisyCamData.size();
    double measMeanSquaredError = measSquaredErrorSum/noisyCamData.size();

    std::cout << " Filter output mean squared error is " << kfMeanSquaredError << std::endl;
    std::cout << " Measurement mean squared error is " << measMeanSquaredError << std::endl;

    RGBABitmapImageReference *imageReference = CreateRGBABitmapImageReference();

	ScatterPlotSeries *series = GetDefaultScatterPlotSeriesSettings();

    series->xs = &noisyCamTime;
	series->ys = &camData;
	series->linearInterpolation = true;
	series->lineType = toVector(L"solid");
	series->lineThickness = 2;
	series->color = CreateRGBColor(0, 0, 1);

	ScatterPlotSeries *series2 = GetDefaultScatterPlotSeriesSettings();
	series2->xs = &noisyCamTime;
	series2->ys = &kfOutData;
	series2->linearInterpolation = true;
	series2->pointType = toVector(L"dots");
	series2->color = CreateRGBColor(1, 0, 0);

    ScatterPlotSeries *series3 = GetDefaultScatterPlotSeriesSettings();
	series3->xs = &noisyCamTime;
	series3->ys = &noisyCamData;
	series3->linearInterpolation = false;
	series3->pointType = toVector(L"circles");
	series3->color = CreateRGBColor(0, 0, 0);

	ScatterPlotSettings *settings = GetDefaultScatterPlotSettings();
	settings->width = 1200;
	settings->height = 800;
	settings->autoBoundaries = true;
	settings->autoPadding = true;
	settings->title = toVector(L"");
	settings->xLabel = toVector(L"");
	settings->yLabel = toVector(L"");
    settings->xLabel = toVector(L"Timsetamp");
    settings->yLabel = toVector(L"posX");
    settings->scatterPlotSeries->push_back(series3);
	settings->scatterPlotSeries->push_back(series);
	settings->scatterPlotSeries->push_back(series2);

	DrawScatterPlotFromSettings(imageReference, settings);

	std::vector<double> *pngdata = ConvertToPNG(imageReference->image);

	WriteToFile(pngdata, "plot.png");

	DeleteImage(imageReference->image);

	return 0;
    
}