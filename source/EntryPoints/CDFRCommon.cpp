#include "EntryPoints/CDFRCommon.hpp"
#include <set>
#include <thread>

#include <Misc/ManualProfiler.hpp>

#include <ArucoPipeline/StaticObject.hpp>
#include <ArucoPipeline/TopTracker.hpp>
#include <ArucoPipeline/SolarPanel.hpp>

namespace CDFRCommon
{
	Settings ExternalSettings(true);
	Settings InternalSettings(false);
};


void CDFRCommon::MakeTrackedObjects(bool Internal, map<CDFRTeam, ObjectTracker&> Trackers)
{
	set<shared_ptr<TrackedObject>> GlobalObjects;
	GlobalObjects.emplace(make_shared<StaticObject>(Internal, "Board"));
	GlobalObjects.emplace(make_shared<SolarPanel>());
	for (int i = 1; i < 11; i++)
	{
		optional<double> height = 0.450;
		CDFRTeam team = CDFRTeam::Unknown;

		if (i < 6) 
		{
			team = CDFRTeam::Blue;
		} 
		else  
		{
			team = CDFRTeam::Yellow;
		}

		GlobalObjects.emplace(make_shared<TopTracker>(i, 0.07, TeamNames.at(team).JavaName + String(" ") + to_string(i), height, true));
	}
	for (auto &Tracker : Trackers)
	{
		for (auto &Object : GlobalObjects)
		{
			Tracker.second.RegisterTrackedObject(Object);
		}
	}
	//TrackerCube* robot1 = new TrackerCube({51, 52, 54, 55}, 0.06, 0.0952, "Robot1");
	//TrackerCube* robot2 = new TrackerCube({57, 58, 59, 61}, 0.06, 0.0952, "Robot2");
	//BlueTracker.RegisterTrackedObject(robot1);
	//BlueTracker.RegisterTrackedObject(robot2);
	
#ifdef USE_TRACKER_CUBES
	auto blue1 = make_shared<TrackerCube>(vector<int>({51, 52, 53, 54, 55}), 0.05, 85.065/1000.0, "blue1");
	auto blue2 = make_shared<TrackerCube>(vector<int>({56, 57, 58, 59, 60}), 0.05, 85.065/1000.0, "blue2");
	auto yellow1 = make_shared<TrackerCube>(vector<int>({71, 72, 73, 74, 75}), 0.05, 85.065/1000.0, "yellow1");
	auto yellow2 = make_shared<TrackerCube>(vector<int>({76, 77, 78, 79, 80}), 0.05, 85.065/1000.0, "yellow2");

	BlueTracker.RegisterTrackedObject(blue1);
	BlueTracker.RegisterTrackedObject(blue2);

	YellowTracker.RegisterTrackedObject(yellow1);
	YellowTracker.RegisterTrackedObject(yellow2);
#endif
	vector<string> PAMINames = {"Triangle", "Square", "Circle"};
	for (size_t i = 0; i < 2; i++)
	{
		auto &tracker = i==0 ? Trackers.at(CDFRTeam::Blue) : Trackers.at(CDFRTeam::Yellow);
		for (size_t j = 0; j < PAMINames.size(); j++)
		{
			auto pamitracker = make_shared<TopTracker>(51+i*20+j, 0.0695, PAMINames[j], 0.112, false);
			tracker.RegisterTrackedObject(pamitracker);
		}
	}
	

}


bool CDFRCommon::ImageToFeatureData(const CDFRCommon::Settings &Settings,  
		Camera* cam, const CameraImageData& ImData, CameraFeatureData& FeatData, 
		ObjectTracker& Tracker, std::chrono::steady_clock::time_point GrabTick, YoloDetect *YoloDetector)
{
	constexpr bool use_threads = false;
	FeatData.Clear();
	FeatData.CopyEssentials(ImData);
	bool doYolo = Settings.YoloDetection && YoloDetector;
	bool doAruco = Settings.ArucoDetection;
	unique_ptr<thread> yoloThread;
	unique_ptr<thread> arucoThread;
	if (doYolo)
	{
		if (use_threads)
		{
			yoloThread = make_unique<thread>(&YoloDetect::Detect, YoloDetector, 
				ImData, &FeatData);
		}
		else
		{
			YoloDetector->Detect(ImData, &FeatData);
		}
	}
	if (doAruco)
	{
		if (use_threads)
		{
			if (Settings.SegmentedDetection)
			{
				arucoThread = make_unique<thread>(DetectArucoSegmented, ImData, &FeatData, 200, Size(4,3));
			}
			else
			{
				arucoThread = make_unique<thread>(DetectAruco, ImData, &FeatData);
			}
		}
		else
		{
			if (Settings.SegmentedDetection)
			{
				DetectArucoSegmented(ImData, &FeatData, 200, Size(4,3));
			}
			else
			{
				DetectAruco(ImData, &FeatData);
			}
		}
	}
	
	if (cam)
	{
		if (Settings.SolveCameraLocation && !cam->PositionLocked)
		{
			if (arucoThread)
			{
				arucoThread->join();
				arucoThread.reset();
			}
			
			bool HasPosition = Tracker.SolveCameraLocation(FeatData);
			if (HasPosition)
			{
				cam->SetLocation(FeatData.CameraTransform, GrabTick);
				//cout << "Camera has location" << endl;
			}
		}
		else
		{
			cam->SetLocation(cam->GetLocation(), GrabTick); //update grabtick
		}
		
		
		FeatData.CameraTransform = cam->GetLocation();
		
		if (Settings.POIDetection)
		{
			if (arucoThread)
			{
				arucoThread->join();
				arucoThread.reset();
			}
			const auto &POIs = Tracker.GetPointsOfInterest();
			DetectArucoPOI(ImData, &FeatData, POIs);
		}
	}
	else
	{
		FeatData.CameraTransform = Affine3d::Identity();
	}
	if (yoloThread)
	{
		yoloThread->join();
		yoloThread.reset();
	}
	if (arucoThread)
	{
		arucoThread->join();
		arucoThread.reset();
	}
	
	
	return false;
}

string TimeToStr()
{
	auto now = chrono::system_clock::to_time_t(chrono::system_clock::now());
	char timestr[64] = {0};
	strftime(timestr, sizeof(timestr), "%m-%d-%H:%M:%S", std::localtime(&now));
	return timestr;
}