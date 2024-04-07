#pragma once

//File that provides most includes for both CDFR scenarios

#include <fstream>
#include <iostream>
#include <opencv2/core.hpp>


#include <GlobalConf.hpp>

#include <Cameras/VideoCaptureCamera.hpp>
#include <Cameras/CameraManagerV4L2.hpp>
#include <Cameras/CameraManagerSimulation.hpp>
#include <ArucoPipeline/TrackerCube.hpp>
#include <ArucoPipeline/TopTracker.hpp>
#include <ArucoPipeline/StaticObject.hpp>
#include <ArucoPipeline/ObjectTracker.hpp>
#include <ArucoPipeline/SolarPanel.hpp>

#include <Misc/FrameCounter.hpp>


using namespace cv;
using namespace std;

namespace CDFRCommon
{
	extern bool direct;
	extern bool v3d;
	extern bool record;
	extern int RecordInterval;


	extern bool SegmentedDetection;
	extern bool POIDetection;
	extern bool YoloDetection;
	extern bool Denoising;
	extern bool DistortedDetection;

	void MakeTrackedObjects(bool Internal, std::map<CDFRTeam, ObjectTracker&> Trackers);
};

string TimeToStr();