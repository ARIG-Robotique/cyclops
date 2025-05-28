#include "Misc/math2d.hpp"

using namespace cv;
using namespace std;

Size ScaleToFit(Size original, Size target)
{
	if (original.area() == 0)
	{
		original = Size(1,1);
	}
	
	Size &insize = target;
	Size &bssize = original;
	int wmax = bssize.width*insize.height/bssize.height;
	wmax = min(insize.width, wmax);
	int hmax = bssize.height*insize.width/bssize.width;
	hmax = min(insize.height, hmax);
	
	Size outsize(wmax, hmax);
	return outsize;
}

Rect ScaleToFit(Size original, Rect target)
{
	Size targetsz = ScaleToFit(original, target.size());
	Size szdiff = target.size()-targetsz;
	Rect roi(szdiff.width/2 + target.x,szdiff.height/2 + target.y, targetsz.width, targetsz.height);
	return roi;
}

vector<Rect> DistributeViewports(Size ImageSize, Size ViewportSize, int NumSources)
{
	Size cuts(1,1);
	while (cuts.area() < NumSources)
	{
		double cutratio = (ImageSize.width*ViewportSize.height*cuts.width)/(double)(ImageSize.height*ViewportSize.width*cuts.height);
		if (cutratio > 1)
		{
			cuts.height++;
		}
		else
		{
			cuts.width++;
		}
	}
	Size2d TileSize(ViewportSize.width/(double)cuts.width, ViewportSize.height/(double)cuts.height);
	vector<Rect> Tiles;
	for (int i = 0; i < NumSources; i++)
	{
		Size pos(i/cuts.height, i%cuts.height);
		Size start(TileSize.width*pos.width, TileSize.height*pos.height);
		Rect roi = Rect(Point(start), Size(TileSize));
		Tiles.push_back(ScaleToFit(ImageSize, roi));
	}
	return Tiles;
}

Size2d GetCameraFOV(Size Resolution, Mat CameraMatrix)
{
	if (CameraMatrix.size().width != 3 || CameraMatrix.size().height != 3)
	{
		return Size2d(M_PI/2, M_PI/2);
	}
	double fx = CameraMatrix.at<double>(0,0), fy = CameraMatrix.at<double>(1,1);
	double fovx = 2*atan(Resolution.width/(2*fx)), fovy = 2*atan(Resolution.height/(2*fy));
	return Size2d(fovx, fovy);
} 