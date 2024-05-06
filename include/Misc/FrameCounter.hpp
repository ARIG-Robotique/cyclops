#pragma once

#include <chrono>
#include <string>
#include <opencv2/core.hpp>

//Helper class to count frames per second
class FrameCounter
{
	typedef std::chrono::steady_clock deltaclock;
	typedef std::chrono::time_point<deltaclock> deltapoint;
	typedef std::chrono::duration<double> deltatype;
private:
	deltapoint LastTime;
	deltapoint StartTime;
	deltatype LastDelta;
public:
	FrameCounter(/* args */);
	~FrameCounter();

	//Get the last value of GetDeltaTime
	double GetLastDelta();

	// Returns the time elapsed since last call of this function
	double GetDeltaTime();

	//Return time since creation of this object
	double GetAbsoluteTime();

	static std::string GetFPSString(double DeltaTime);

	//helper function to add a fps counter
	static void AddFpsToImage(cv::InputOutputArray img, double DeltaTime);
};
