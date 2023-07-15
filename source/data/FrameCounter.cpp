#include "data/FrameCounter.hpp"

#include <string>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

using namespace cv;

FrameCounter::FrameCounter()
{
	LastTime = deltaclock::now();
	StartTime = LastTime;
}

FrameCounter::~FrameCounter()
{

}

double FrameCounter::GetDeltaTime()
{
	deltapoint dt2 = deltaclock::now();
	deltatype deltaTime = (dt2-LastTime);
	LastTime = dt2;
	return deltaTime.count();
}

double FrameCounter::GetAbsoluteTime()
{
	deltapoint dt2 = deltaclock::now();
	deltatype deltaTime = dt2-StartTime;
	return deltaTime.count();
}

std::string FrameCounter::GetFPSString(double DeltaTime)
{
	char buffer[20];
	snprintf(buffer, 20, "fps : %.1f", 1/DeltaTime);
	return std::string(buffer);
}

void FrameCounter::AddFpsToImage(InputOutputArray img, double DeltaTime)
{
	std::string strfps = GetFPSString(DeltaTime);
	putText(img, strfps, Point2i(0,img.rows()-20), FONT_HERSHEY_SIMPLEX, 2, Scalar(255, 255, 255), 5);
	putText(img, strfps, Point2i(0,img.rows()-20), FONT_HERSHEY_SIMPLEX, 2, Scalar(0, 0, 0), 2);
}