#include "Cameras/ImageTypes.hpp"

using namespace cv;
using namespace std;

bool LensSettings::IsValid()
{
	if (CameraMatrix.size() != Size(3,3))
	{
		return false;
	}
	if (distanceCoeffs.size().area() <= 0)
	{
		return false;
	}
	if (ROI.area() <= 0)
	{
		return false;
	}
	return true;
}

bool CameraSettings::IsValidCalibration()
{
	if (Lenses.size() == 0)
	{
		return false;
	}
	for (auto &&i : Lenses)
	{
		if (!i.IsValid())
		{
			return false;
		}
	}
	return true;
}

bool CameraSettings::IsValid()
{
	return Framerate >0 && Resolution.width >0 && Resolution.height >0 && FramerateDivider > 0;
}