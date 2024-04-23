#include "Cameras/ImageSource.hpp"
#include <Misc/GlobalConf.hpp>

using namespace cv;
using namespace std;

ImageSource::ImageSource(/* args */)
{
}

ImageSource::~ImageSource()
{
}

void ImageSource::SetFrame(const CameraImageData& frame, bool Distorted)
{
	(void)frame;
	(void)Distorted;
}

CameraImageData ImageSource::GetFrame(bool Distorted) const
{
	(void)Distorted;
	return CameraImageData();
}

Size findSplit(Size screensize, Size targetAspect, int numscreen)
{
	Size splits(1,1);
	double AspectRatioTarget = (double)targetAspect.width/targetAspect.height;
	double AspectRatioScreen = (double)screensize.width/screensize.height;
	while (splits.height * splits.width < numscreen)
	{
		double arc = AspectRatioScreen/(splits.width+1.0)*splits.height;
		double arr = AspectRatioScreen/splits.width*(splits.height+1.0);
		if (abs(arc - AspectRatioTarget) < abs(arr - AspectRatioTarget))
		{
			splits.width++;
		}
		else
		{
			splits.height++;
		}
	}
	return splits;
}