#include "EntryPoints/CDFRInternal.hpp"
#include <EntryPoints/CDFRCommon.hpp>

#include <Visualisation/BoardGL.hpp>
#include <Communication/Transport/TCPTransport.hpp>
#include <Communication/Transport/UDPTransport.hpp>
#include <Communication/Transport/SerialTransport.hpp>

void CDFRInternalMain(bool direct, bool v3d)
{
	assert(0);
	/*CameraManager CameraMan(GetCaptureMethod(), GetCaptureConfig().filter, false);

	auto& Detector = GetArucoDetector();

	BoardGL OpenGLBoard;
	OpenGLBoard.Start();
	
	FrameCounter fps;

	ObjectTracker tracker;
	
	PositionDataSender sender;
	{
		WebsocketConfig wscfg = GetWebsocketConfig();
		sender.encoder = new MinimalEncoder(GetDefaultAllowMap());
		sender.transport = new TCPTransport(wscfg.Server, wscfg.IP, wscfg.Port, wscfg.Interface);
		sender.StartReceiveThread();
	}

	StaticObject* boardobj = new StaticObject(true, "board");
	tracker.RegisterTrackedObject(boardobj);

	TrackerCube* robot1 = new TrackerCube({51, 52, 54, 55}, 0.06, 0.0952, "Robot1");
	TrackerCube* robot2 = new TrackerCube({57, 58, 59, 61}, 0.06, 0.0952, "Robot2");
	tracker.RegisterTrackedObject(robot1);
	tracker.RegisterTrackedObject(robot2);
	
	int hassent = 0;
	for (;;)
	{
		double deltaTime = fps.GetDeltaTime();
		CameraMan.Tick<VideoCaptureCamera>();
		int64 GrabTick = getTickCount();
		BufferedPipeline(vector<Camera*>(physicalCameras.begin(), physicalCameras.end()), Detector, &tracker);

		//cout << "Pipeline took " << TimePipeline << "s to run" << endl;
		
		vector<CameraFeatureData> arucoDatas;
		int NumCams = physicalCameras.size();
		arucoDatas.resize(NumCams);

		for (int i = 0; i < physicalCameras.size(); i++)
		{
			Camera* cam = physicalCameras[i];
			if (!cam->GetMarkerData(arucoDatas[i]))
			{
				continue;
			}
		}
		
		tracker.SolveLocationsPerObject(arucoDatas, GrabTick);
		vector<ObjectData> ObjData = tracker.GetObjectDataVector(GrabTick);

		if (!OpenGLBoard.Tick(ObjectData::ToGLObjects(ObjData)))
		{
			break;
		}

		if (waitKey(1) == '\e')
		{
			break;
		}
	}*/
}