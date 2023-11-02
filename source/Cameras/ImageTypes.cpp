#include "Cameras/ImageTypes.hpp"

using namespace cv;
using namespace std;

bool CameraSettings::IsValidCalibration()
{
	return CameraMatrix.size() == Size(3,3) && distanceCoeffs.size().area() > 0;
}

bool CameraSettings::IsValid()
{
	return Framerate >0 && Resolution.width >0 && Resolution.height >0 && FramerateDivider > 0;
}