#include "EntryPoints/CDFRInternal.hpp"
#include <EntryPoints/CDFRCommon.hpp>

#include <DetectFeatures/ArucoDetect.hpp>
#include <DetectFeatures/YoloDetect.hpp>

#include <iostream>
#include <sstream>
#include <thread>

using namespace std;

CDFRInternal::CDFRInternal()
{
}

CDFRInternal::~CDFRInternal()
{
}

shared_future<CDFRInternal::InternalResult> CDFRInternal::Inject(CameraImageData &InData, CDFRTeam Team)
{
	return std::async(std::launch::async, &CDFRInternal::Process, this, InData, Team);
}

CDFRInternal::InternalResult CDFRInternal::Process(CameraImageData InData, CDFRTeam Team)
{
	cout << "Started internal processing thread " << this_thread::get_id() << endl;
	ObjectTracker tracker;
	CDFRCommon::MakeTrackedObjects(true, {{Team, tracker}});

	InternalResult response;

	auto GrabTick = TrackedObject::Clock::now();

	CDFRCommon::ImageToFeatureData(CDFRCommon::InternalSettings, nullptr, InData, response.FeatureData, tracker, GrabTick);

	std::vector FDArray({response.FeatureData});

	tracker.SolveLocationsPerObject(FDArray, GrabTick);
	response.ObjData = tracker.GetObjectDataVector(GrabTick);

	cout << "Done internal processing thread " << this_thread::get_id() << endl;

	return response;
}