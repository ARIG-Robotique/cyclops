#include <PostProcessing/Jardinieres.hpp>
#include <EntryPoints/CDFRExternal.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>

#include <iostream>

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
		stock.TimeSpentContacting = chrono::seconds(0);
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
		stock.BuzzingPoint = Vec2d(CenterPosition.val) + Vec2d(west ? 0.075 : -0.075, 0);
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
		stock.BuzzingPoint = Vec2d(CenterPosition.val) + Vec2d(0, -0.075);
	}
}

void PostProcessJardinieres::Process(std::vector<CameraImageData> &ImageData, std::vector<CameraFeatureData> &FeatureData, std::vector<ObjectData> &Objects)
{
	(void) ImageData;
	#if 1
	(void) FeatureData;
	(void) Objects;
	#else
	if (FeatureData.size() != 1)
	{
		return;
	}
	for (auto &zone : Stocks)
	{
		zone.NumPlants = 0;
	}
	const auto &ThisImageData = ImageData[0];
	if (!ThisImageData.Valid)
	{
		return;
	}
	const auto &ThisFeatureData = FeatureData[0];
	const auto InvCameraMatrix = ThisFeatureData.CameraTransform.inv();
	Size WantedImageSize(65,30);
	vector<Vec2f> AffineTarget{{0,0}, {0, (float)WantedImageSize.height}, Vec2f(WantedImageSize.width, WantedImageSize.height)};
	int adaptive_threshold_size = 5;
	int dilation_amount = adaptive_threshold_size/2, erosion_amount=dilation_amount+1;
	Mat dilation_kernel = getStructuringElement(MORPH_ELLIPSE, Size(dilation_amount*2+1,dilation_amount*2+1), Point(dilation_amount,dilation_amount));
	Mat erosion_kernel = getStructuringElement(MORPH_ELLIPSE, Size(erosion_amount*2+1,erosion_amount*2+1), Point(erosion_amount,erosion_amount));
	vector<ObjectData> EnemyRobots = GetEnemyRobots(Objects);
	
	for (auto &zone : Stocks)
	{
		vector<Vec2d> ImagePointsDouble;
		projectPoints(zone.Corners, InvCameraMatrix.rvec(), InvCameraMatrix.translation(), 
			ThisFeatureData.CameraMatrix, ThisFeatureData.DistanceCoefficients, ImagePointsDouble);
		vector<Vec2f> ImagePoints(ImagePointsDouble.begin(), ImagePointsDouble.end()-1);
		
		Mat WarpedImage, HSV; vector<Mat> BGRComponents, HSVComponents;
		Mat AffineMatrix = getAffineTransform(ImagePoints, AffineTarget);
		warpAffine(ThisImageData.Image, WarpedImage, AffineMatrix, WantedImageSize);
		split(WarpedImage, BGRComponents);
		cvtColor(WarpedImage, HSV, COLOR_BGR2HSV);
		split(WarpedImage, HSVComponents);
		Mat mask, green, notblue, notred, adaptive_value, adaptive_dilated, adaptive_eroded, saturated;
		threshold(BGRComponents[1], green, 32, 255, THRESH_BINARY);
		threshold(BGRComponents[0], notblue, 128, 255, THRESH_BINARY_INV);
		threshold(BGRComponents[2], notred, 128, 255, THRESH_BINARY_INV);
		adaptiveThreshold(HSVComponents[2], adaptive_value, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY_INV, 3, 10);
		dilate(adaptive_value, adaptive_dilated, dilation_kernel);
		erode(adaptive_dilated, adaptive_eroded, erosion_kernel);
		threshold(HSVComponents[1], saturated, 32, 255, THRESH_BINARY);
		mask = green & notblue & notred & adaptive_eroded;
		/*if (zone.name == "Bleu Sud")
		{
			Mat concat;
			vector<Mat> masks{green, notblue, notred, adaptive_value, adaptive_dilated, adaptive_eroded, saturated, mask};
			vconcat(masks, concat);
			cvtColor(concat, concat, COLOR_GRAY2BGR);
			masks = {WarpedImage, concat};
			vconcat(masks, concat);
			imshow(zone.name, concat);
		}*/
		int NumWhitePixels = countNonZero(mask);
		zone.NumPlants = NumWhitePixels * 12 / WantedImageSize.area();
		//cout << zone.NumPlants << " plants in " << zone.name << endl;

		//track enemy robot contact
		zone.ContactThisTick = false;
		for (auto &obj : EnemyRobots)
		{
			auto pos = obj.GetPos2D();
			auto delta = pos-zone.BuzzingPoint;
			double distance = sqrt(delta.ddot(delta));
			bool contact = distance < zone.BuzzingRadius;
			if (contact)
			{
				if (!zone.Contacting)
				{
					zone.LastContactStart = obj.LastSeen;
				}
				zone.ContactThisTick = true;
			}
		}
		if (!zone.ContactThisTick && zone.Contacting)
		{
			zone.LastContactEnd = ThisImageData.GrabTime;
			zone.TimeSpentContacting += zone.LastContactEnd - zone.LastContactStart;
		}
		zone.Contacting = zone.ContactThisTick;

		ObjectData obj(ObjectType::Jardiniere, zone.name, Affine3d::Identity(), ThisImageData.GrabTime);
		obj.metadata["numPlantes"] = zone.NumPlants;
		obj.metadata["whitePixels"] = NumWhitePixels;
		if (zone.LastContactStart != ObjectData::TimePoint())
		{
			obj.metadata["lastContactStartAge"] = chrono::duration_cast<chrono::milliseconds>(ObjectData::Clock::now() - zone.LastContactStart).count();
		}
		if (zone.LastContactEnd != ObjectData::TimePoint())
		{
			obj.metadata["lastContactStopAge"] = chrono::duration_cast<chrono::milliseconds>(ObjectData::Clock::now() - zone.LastContactEnd).count();
		}
		obj.metadata["contacting"] = zone.Contacting;
		obj.metadata["timeSpentNear"] = chrono::duration_cast<chrono::milliseconds>(zone.TimeSpentContacting).count();
		Objects.push_back(obj);
	}
	//waitKey(2);
	#endif

}