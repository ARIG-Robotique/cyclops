#include <PostProcessing/Jardinieres.hpp>
#include <EntryPoints/CDFRExternal.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>

using namespace std;
using namespace cv;

PostProcessJardinieres::PostProcessJardinieres(CDFRExternal* InOwner)
	:PostProcess(InOwner)
{
	array<string, 6> names = {"Jaune Milieu", "Bleu Sud", "Bleu Milieu", "Jaune Sud", "Jaune Nord", "Bleu Nord"};
	size_t stockidx;
	for (size_t stockidx = 0; stockidx < Stocks.size(); stockidx++)
	{
		auto& stock = Stocks[stockidx];
		stock.name = names[stockidx];
	}
	for (stockidx = 0; stockidx < 4; stockidx++)
	{
		auto& stock = Stocks[stockidx];
		Vec3d CenterPosition(1.575,0.3875, 0.092);
		bool west = stockidx>=2;
		bool blue = stockidx%2==west;
		bool center = (stockidx%2)==0;
		if (!center)
		{
			CenterPosition[1] *=-1;
		}
		if (west)
		{
			CenterPosition[0] *=-1;
		}
		stock.team = blue ? CDFRTeam::Blue : CDFRTeam::Yellow;
		stock.Corners.reserve(4);
		Vec3d Size(0.150, 0.325, 0.0);
		for (size_t corneridx = 0; corneridx < 4; corneridx++)
		{
			Vec3d corneroffset = Size/2.0;
			int ymul = corneridx>=2 ? -1:1;
			corneroffset[1] *= ymul;
			corneroffset[0] *= ymul * (corneridx%2==0 ? 1 : -1);
			stock.Corners.emplace_back(CenterPosition + corneroffset);
		}
	}
	for (stockidx = 4; stockidx < 6; stockidx++)
	{
		auto& stock = Stocks[stockidx];
		Vec3d CenterPosition(0.7375,1.075, 0.092);
		bool west = stockidx==5;
		bool blue = west;
		if (west)
		{
			CenterPosition[0] *=-1;
		}
		stock.team = blue ? CDFRTeam::Blue : CDFRTeam::Yellow;
		stock.Corners.reserve(4);
		Vec3d Size(0.325, 0.150, 0.0);
		for (size_t corneridx = 0; corneridx < 4; corneridx++)
		{
			Vec3d corneroffset = Size/2.0;
			int ymul = corneridx>=2 ? -1:1;
			corneroffset[0] *= ymul;
			corneroffset[1] *= ymul * (corneridx%2==0 ? 1 : -1);
			stock.Corners.emplace_back(CenterPosition + corneroffset);
		}
	}
}

void PostProcessJardinieres::Process(std::vector<CameraImageData> &ImageData, std::vector<CameraFeatureData> &FeatureData, std::vector<ObjectData> &Objects)
{
	(void) ImageData;
	if (FeatureData.size() == 0)
	{
		return;
	}
	assert(FeatureData.size() == 1);
	for (auto &zone : Stocks)
	{
		zone.NumPlants = 0;
	}
	const auto &ThisImageData = ImageData[0];
	const auto &ThisFeatureData = FeatureData[0];
	const auto InvCameraMatrix = ThisFeatureData.CameraTransform.inv();
	Size WantedImageSize(65,30);
	vector<Vec2f> AffineTarget{{0,0}, {0, (float)WantedImageSize.height}, Vec2f(WantedImageSize.width, WantedImageSize.height)};
	for (auto &zone : Stocks)
	{
		vector<Vec2d> ImagePointsDouble;
		projectPoints(zone.Corners, InvCameraMatrix.rvec(), InvCameraMatrix.translation(), 
			ThisFeatureData.CameraMatrix, ThisFeatureData.DistanceCoefficients, ImagePointsDouble);
		vector<Vec2f> ImagePoints(ImagePointsDouble.begin(), ImagePointsDouble.end()-1);
		
		Mat WarpedImage; 
		Mat AffineMatrix = getAffineTransform(ImagePoints, AffineTarget);
		warpAffine(ThisImageData.Image, WarpedImage, AffineMatrix, WantedImageSize);
		//imshow(zone.name, WarpedImage);
	}
	//waitKey(1);
}