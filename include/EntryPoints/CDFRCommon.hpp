#pragma once

//File that provides most includes for both CDFR scenarios

#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/photo.hpp>

#include <Misc/GlobalConf.hpp>

#include <Cameras/ImageTypes.hpp>
#include <Cameras/Camera.hpp>
#include <ArucoPipeline/ObjectTracker.hpp>
#include <DetectFeatures/ArucoDetect.hpp>
#include <DetectFeatures/YoloDetect.hpp>
#include <DetectFeatures/ColorDetect.hpp>
#include <DetectFeatures/StereoDetect.hpp>

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
		int RecordInterval = 0;

		bool ArucoDetection = true;
		bool SegmentedDetection = true;
		bool POIDetection = false;
		bool YoloDetection = false;
		bool Denoising = false;
		bool DistortedDetection = true;
		bool SolveCameraLocation = true;

		Settings(bool External)
			:direct(External),
			SegmentedDetection(External),
			DistortedDetection(External),
			SolveCameraLocation(External)
		{

		}
	};

	extern Settings ExternalSettings;
	extern Settings InternalSettings;

	void MakeTrackedObjects(bool Internal, std::map<CDFRTeam, ObjectTracker&> Trackers);

	bool ImageToFeatureData(const CDFRCommon::Settings &Settings,  
		Camera* cam, const CameraImageData& ImData, CameraFeatureData& FeatData, 
		ObjectTracker& Tracker, std::chrono::steady_clock::time_point GrabTick, YoloDetect *YoloDetector = nullptr);
};

string TimeToStr();