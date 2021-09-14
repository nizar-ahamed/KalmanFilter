#include <iostream>
#include <vector>
#include <fstream>
#include <sstream>
#include <random>

struct data{
    double time;
    double posX;
};

/*************************************************************
 * param in  - file to load data from (to be implemented)
 * param out - vector containing timestamp and position
 * remarks -  
 * **********************************************************/
void loadData(std::vector<data> &out)
{
    std::string line;
    std::ifstream stream("../data/cam_data1.txt"); 
    if (stream.is_open())
    {
        std::cout<< "file opened"<<std::endl;
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
 *             of vector to which noise is to be added
 * param out - vector with added noise in posX
 * remarks - 
 * *******************************************************/
void addNoise(std::vector<data> &out, const double stdDev)
{
    const double mean = 0.0;
    std::default_random_engine generator;
    std::normal_distribution<double> dist(mean,stdDev);

    for (auto& x : out)
    {
        x.posX += dist(generator); 
    }

}

int main()
{
    std::vector<data> camData;
    loadData(camData);
    addNoise(camData, 0.1);

    for (int i=0;i<20;i++)
    {
        std::cout << camData[i].posX<<std::endl;
    }
}