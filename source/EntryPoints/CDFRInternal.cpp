#include "EntryPoints/CDFRInternal.hpp"
#include <EntryPoints/CDFRCommon.hpp>

#include <DetectFeatures/ArucoDetect.hpp>
#include <DetectFeatures/YoloDetect.hpp>

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

	ManualProfiler<false> profiler;

	InternalResult response;

	CDFRCommon::Detection(CDFRCommon::InternalSettings, profiler, nullptr, InData, response.FeatureData, tracker, 0);

	std::vector FDArray({response.FeatureData});

	tracker.SolveLocationsPerObject(FDArray, 0);
	response.ObjData = tracker.GetObjectDataVector(0);

	cout << "Done internal processing thread " << this_thread::get_id() << endl;

	return response;
}