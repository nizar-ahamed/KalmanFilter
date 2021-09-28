# KalmanFilter

## Dependencies for Running Locally
* cmake >= 3.0
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory in the top level directory: `mkdir build && cd build`
5. Compile: `cmake .. && make`
6. Run it: `./kalmanFilter`
7. png plots will be generated in the build folder
	- legend for the plots: solid blue line - ground truth data
				solid red line  - kalman filter output
				black squares   - measurement data


## References

Kalman equations are picked from [wikipedia](https://en.wikipedia.org/wiki/Kalman_filter) and [towardsdatascience](https://towardsdatascience.com/kalman-filter-an-algorithm-for-making-sense-from-the-insights-of-various-sensors-fused-together-ddf67597f35e)

![image](https://user-images.githubusercontent.com/80982593/134109472-f47cf53d-1a78-4642-b470-261972966174.png)

[EigenLibrary](https://eigen.tuxfamily.org/index.php?title=Main_Page)

[pbPlots](https://github.com/InductiveComputerScience/pbPlots)


