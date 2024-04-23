#pragma once

//File that provides most includes for both CDFR scenarios

#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/photo.hpp>

#include <GlobalConf.hpp>

#include <Cameras/ImageTypes.hpp>
#include <Cameras/Camera.hpp>
#include <ArucoPipeline/ObjectTracker.hpp>
#include <DetectFeatures/ArucoDetect.hpp>
#include <DetectFeatures/YoloDetect.hpp>
#include <DetectFeatures/ColorDetect.hpp>

#include <Misc/FrameCounter.hpp>
#include <Misc/ManualProfiler.hpp>


using namespace cv;
using namespace std;

namespace CDFRCommon
{
	struct Settings
	{
	public:
		bool direct = true;
		bool v3d = false;
		bool record = false;
		int RecordInterval = 10;


		bool SegmentedDetection = true;
		bool POIDetection = false;
		bool YoloDetection = false;
		bool Denoising = false;
		bool DistortedDetection = true;

		Settings(bool External)
			:direct(External),
			SegmentedDetection(External),
			DistortedDetection(External)
		{

		}
	};

	extern Settings ExternalSettings;
	extern Settings InternalSettings;

	void MakeTrackedObjects(bool Internal, std::map<CDFRTeam, ObjectTracker&> Trackers);

	template<bool ProfilerEnabled> 
	bool Detection(const CDFRCommon::Settings &Settings, ManualProfiler<ProfilerEnabled> &profiler, Camera* cam, const CameraImageData& ImData, CameraFeatureData& FeatData, ObjectTracker& Tracker, uint64_t GrabTick)
	{
		if (Settings.Denoising)
		{
			profiler.EnterSection("Denoise");
			cv::fastNlMeansDenoising(ImData.Image, ImData.Image, 10);
		}
		FeatData.CopyEssentials(ImData);
		if (Settings.YoloDetection)
		{
			profiler.EnterSection("Detect Yolo");
			DetectYolo(ImData, FeatData);
		}
		else
		{
			//profiler.EnterSection("Detect Color");
			//DetectColor(ImData, FeatData);
		}
		profiler.EnterSection("Detect Aruco");
		if (Settings.SegmentedDetection)
		{
			DetectArucoSegmented(ImData, FeatData, 200, Size(4,3));
		}
		else
		{
			DetectAruco(ImData, FeatData);
		}
		
		if (cam)
		{
			profiler.EnterSection("3D Solve Camera");
			bool HasPosition = Tracker.SolveCameraLocation(FeatData);
			if (HasPosition)
			{
				cam->SetLocation(FeatData.CameraTransform, GrabTick);
				//cout << "Camera has location" << endl;
			}
			FeatData.CameraTransform = cam->GetLocation();
			
			if (Settings.POIDetection)
			{
				profiler.EnterSection("Detect Aruco POIs");
				const auto &POIs = Tracker.GetPointsOfInterest();
				DetectArucoPOI(ImData, FeatData, POIs);
			}
			return HasPosition;
		}
		else
		{
			FeatData.CameraTransform = Affine3d::Identity();
		}
		return false;
	}
};

string TimeToStr();