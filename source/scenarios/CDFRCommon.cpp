#include "Scenarios/CDFRCommon.hpp"
#include "data/ManualProfiler.hpp"

typedef ManualProfiler<false> pipelineprof;

pipelineprof mp("Pipeline ", {"Read", "Undistort", "Rescale", "ArucoDetect", "ArucoSolve"});

void BufferedPipeline(vector<Camera*> Cameras, aruco::ArucoDetector& Detector, ObjectTracker* registry)
{
	int numCams = Cameras.size();
	vector<uint8_t> BufToCamMap;
	vector<uint8_t> BufIdxMap;
	for (int i = 0; i < Cameras.size(); i++)
	{
		Cameras[i]->Grab();
		for (size_t j = 0; j < 1/*Cameras[i]->GetCameraSettings().BufferSize*/; j++)
		{
			BufToCamMap.push_back(i);
			BufIdxMap.push_back(j);
		}
		
	}
	//Range range(0, numCams*Camera::FrameBufferSize);
	vector<pipelineprof> mps;
	mps.resize(BufToCamMap.size());
	parallel_for_(Range(0, BufToCamMap.size()), [&](const Range& range)
	{
		for (int i = range.start; i < range.end; i++)
		{
			pipelineprof& localprof = mps[i];
			int pf = 0;
			localprof.EnterSection(pf++);
			int CamIdx = BufToCamMap[i];
			Camera* cam = Cameras[CamIdx];
			int Buffer0 = BufIdxMap[i];
			//Detect aruco
			cam->Read();
			localprof.EnterSection(pf++);
			cam->Undistort();
			localprof.EnterSection(pf++);
			cam->RescaleFrames();
			localprof.EnterSection(pf++);
			cam->detectMarkers(Detector);
			localprof.EnterSection(-1);
			
			
		}
		//cout << "Aruco stripe from " << range.start << " to " << range.end << endl;
	}, BufToCamMap.size());
	for (int i = 0; i < BufToCamMap.size(); i++)
	{
		mp += mps[i];
	}
	mp.PrintIfShould();
	
} 

map<PacketType, bool> GetDefaultAllowMap()
{
	map<PacketType, bool> allowmap = {
		{PacketType::Null, false},
		{PacketType::Camera, false},
		{PacketType::ReferenceAbsolute, false},
		{PacketType::ReferenceRelative, true},
		{PacketType::Robot, true},
		{PacketType::TrackerCube, true},
		{PacketType::TopTracker, true},
		{PacketType::Puck, true},
		{PacketType::Tag, false},
		{PacketType::Team, true}
	};
	return allowmap;
}