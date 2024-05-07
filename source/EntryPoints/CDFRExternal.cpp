
#include "EntryPoints/CDFRExternal.hpp"
#include <EntryPoints/CDFRCommon.hpp>

#include <opencv2/photo.hpp> //for denoising


#include <Cameras/Calibfile.hpp>
#include <DetectFeatures/ArucoDetect.hpp>
#include <DetectFeatures/YoloDetect.hpp>

#include <Visualisation/BoardGL.hpp>
#include <Visualisation/ImguiWindow.hpp>

#include <Misc/ManualProfiler.hpp>
#include <Misc/math2d.hpp>
#include <Misc/path.hpp>

#include <Communication/Transport/TCPTransport.hpp>
#include <Communication/Transport/UDPTransport.hpp>
#include <Cameras/CameraManagerV4L2.hpp>
#include <Cameras/CameraManagerSimulation.hpp>
#include <Cameras/VideoCaptureCamera.hpp>

#include <PostProcessing/YoloDeflicker.hpp>
#include <PostProcessing/StockPlants.hpp>
#include <PostProcessing/Jardinieres.hpp>

#include <thread>
#include <memory>

#include <thirdparty/HsvConverter.h>


CDFRExternal::CDFRExternal()
{

	assert(ObjData.size() == FeatureData.size());
	assert(FeatureData.size() > 0);

	CDFRCommon::MakeTrackedObjects(false, 
		{
			{CDFRTeam::Blue, BlueTracker},
			{CDFRTeam::Yellow, YellowTracker},
			{CDFRTeam::Unknown, UnknownTracker}
		}	
	);

	Start();
}

CDFRTeam CDFRExternal::GetTeamFromCameraPosition(vector<Camera*> Cameras)
{
	if (Cameras.size() == 0)
	{
		return CDFRTeam::Unknown;
	}
	const static double ydist = 0.95;
	const static double xdist = 1.594;
	const static map<CDFRTeam, vector<Vec2d>> CameraPos = 
	{
		{CDFRTeam::Blue, {{0, -ydist}, {-xdist, ydist}, {-xdist, -ydist}, {-0.225, 1.122}}},
		{CDFRTeam::Yellow, {{0, ydist}, {xdist, ydist}, {xdist, -ydist}, {0.225, 1.122}}}
	};
	map<CDFRTeam, int> TeamScores;
	for (Camera* cam : Cameras)
	{
		auto campos = cam->GetLocation().translation();
		Vec2d pos2d(campos[0], campos[1]);
		CDFRTeam bestTeam = CDFRTeam::Unknown;
		double bestdist = 0.5; //tolerance of 50cm
		for (const auto& [team, positions] : CameraPos)
		{
			for (const auto &position : positions)
			{
				Vec2d delta = position-pos2d;
				double dist = sqrt(delta.ddot(delta));
				if (dist < bestdist)
				{
					bestTeam = team;
					bestdist = dist;
				}
			}
			
		}
		auto position = TeamScores.find(bestTeam);
		if (position == TeamScores.end())
		{
			TeamScores[bestTeam] = 1;
		}
		else
		{
			TeamScores[bestTeam] += 1;
		}
	}
	CDFRTeam bestTeam = CDFRTeam::Unknown;
	int mostCount = 0;
	for (const auto [team, count] : TeamScores)
	{
		if (count > mostCount)
		{
			bestTeam = team;
			mostCount = count;
		}
	}
	return bestTeam;
}

void CDFRExternal::SetCameraLock(bool value)
{
	CDFRCommon::ExternalSettings.SolveCameraLocation = !value;
}

CDFRTeam CDFRExternal::GetTeam()
{
	auto Cameras = CameraMan->GetCameras();
	CDFRTeam Team = LockedTeam == CDFRTeam::Unknown ? GetTeamFromCameraPosition(Cameras) : LockedTeam;
	return Team;
}

using ExternalProfType = ManualProfiler<false>;

void CDFRExternal::ThreadEntryPoint()
{
	ExternalProfType prof("External Global Profile");
	ExternalProfType ParallelProfiler("Parallel Cameras Detail");
	
	if (GetScenario().size())
	{
		auto basepath = GetCyclopsPath() / "sim";
		CameraMan = make_unique<CameraManagerSimulation>(basepath/GetScenario());
	}
	else
	{
		Idle = true;
		CameraMan = make_unique<CameraManagerV4L2>(GetCaptureMethod(), GetCaptureConfig().filter, false);
	}

	YoloDetector = make_unique<YoloDetect>("cdfr", 4);

	PostProcesses.emplace_back(make_unique<PostProcessYoloDeflicker>(this));
	PostProcesses.emplace_back(make_unique<PostProcessStockPlants>(this));
	PostProcesses.emplace_back(make_unique<PostProcessJardinieres>(this));

	//display/debug section
	FrameCounter fps;
	
	//OpenGLBoard.InspectObject(blue1);
	if (CDFRCommon::ExternalSettings.v3d)
	{
		Open3DVisualizer();
	}
	if (CDFRCommon::ExternalSettings.direct)
	{
		OpenDirectVisualizer();
	}
	
	RecordRootPath = GetCyclopsPath() / "Recordings" / TimeToStr();
	
	
	//track and untrack cameras dynamically
	CameraMan->StartCamera = [](VideoCaptureCameraSettings settings) -> shared_ptr<Camera>
	{
		auto cam = make_shared<VideoCaptureCamera>(make_shared<VideoCaptureCameraSettings>(settings));
		if(!cam->StartFeed())
		{
			cerr << "Failed to start feed @" << settings.DeviceInfo.device_description << endl;
			return nullptr;
		}
		return cam;
	};
	
	CameraMan->RegisterCamera = [this](shared_ptr<Camera> cam) -> void
	{
		if (!CDFRCommon::ExternalSettings.SolveCameraLocation)
		{
			CDFRCommon::ExternalSettings.SolveCameraLocation = true;
			cerr << "New camera registered, but the cameras were locked, removing the lock..." << endl;
		}
		BlueTracker.RegisterTrackedObject(cam);
		YellowTracker.RegisterTrackedObject(cam);
		cout << "Registering new camera @" << cam << ", name " << cam->GetName() << endl;
	};
	CameraMan->StopCamera = [this](shared_ptr<Camera> cam) -> bool
	{
		BlueTracker.UnregisterTrackedObject(cam);
		YellowTracker.UnregisterTrackedObject(cam);
		cout << "Unregistering camera @" << cam << endl;
		
		return true;
	};

	CameraMan->Start();

	
	while (!killed)
	{
		DetectionFrameCounter.GetDeltaTime();
		bool LowPower = false;
		if(Idle)
		{
			//this_thread::sleep_for(chrono::milliseconds(10));
			if (!LastIdle)
			{
				cout << "Entering idle..." << endl;
			}
			LastIdle = true;
			LowPower = true;
		}
		else if (LastIdle)
		{
			cout << "Exiting idle..." << endl;
			LastIdle = false;
		}
		vector<Camera*> Cameras;
		double deltaTime = fps.GetDeltaTime();
		prof.EnterSection("CameraManager Tick");
		Cameras = CameraMan->Tick();
		bool HasNoData = Cameras.size() == 0;
		bool IsUnseen = HasNoClients && !DirectImage && !OpenGLBoard;
		if (HasNoData || IsUnseen)
		{
			prof.EnterSection("Sleep");
			if (!LastSleep)
			{
				LastSleep = true;
				cout << "Entering sleep..." << endl;
			}
			LowPower = true;
			//
			//continue;
		}
		else if (LastSleep)
		{
			cout << "Exiting sleep..." << endl;
			LastSleep = false;
		}

		if (LowPower)
		{
			Cameras.clear();
			this_thread::sleep_for(chrono::milliseconds(500));
			//continue;
		}
		
		
		
		CDFRTeam Team = GetTeam();
		if (Team != LastTeam && Cameras.size() > 0 && LockedTeam == CDFRTeam::Unknown)
		{
			cout << "Detected team change : to " << Team << endl;
			LastTeam = Team;
		}
		ObjectTracker* TrackerToUse;
		switch (Team)
		{
		case CDFRTeam::Blue:
			TrackerToUse = &BlueTracker;
			break;
		case CDFRTeam::Yellow:
			TrackerToUse = &YellowTracker;
			break;
		default:
			if (!LowPower)
			{
				cout << "Warning : Using unknown tracker" << endl;
			}
			TrackerToUse = &UnknownTracker;
			break;
		}

		bool RecordThisTick = ForceRecordNext;
		if (RecordTick >= CDFRCommon::ExternalSettings.RecordInterval-1)
		{
			RecordTick = 0;
			RecordThisTick |= CDFRCommon::ExternalSettings.record;
		}
		else if (Cameras.size() > 0 && CDFRCommon::ExternalSettings.record)
		{
			RecordTick++;
		}
		
		
		
		prof.EnterSection("Camera Gather Frames");
		auto GrabTick = chrono::steady_clock::now();
		
		for (size_t i = 0; i < Cameras.size(); i++)
		{
			Cameras[i]->Grab();
		}

		int NumCams = Cameras.size();
		vector<CameraImageData> &ImageDataLocal = ImageData[BufferIndex];
		vector<CameraFeatureData> &FeatureDataLocal = FeatureData[BufferIndex];
		vector<ExternalProfType> ParallelProfilers;
		ImageDataLocal.resize(NumCams);
		FeatureDataLocal.resize(NumCams);
		ParallelProfilers.resize(NumCams);
		prof.EnterSection("Parallel Cameras");

		//grab frames
		//read frames
		//undistort
		//detect aruco and yolo

		/*parallel_for_(Range(0, Cameras.size()), 
		[&Cameras, &FeatureDataLocal, &CamerasWithPosition, TrackerToUse, GrabTick, &ParallelProfilers]
		(Range InRange)*/
		{
			Range InRange(0, Cameras.size());
			for (int i = InRange.start; i < InRange.end; i++)
			{
				auto &thisprof = ParallelProfilers[i];
				Camera* cam = Cameras[i];
				CameraFeatureData &FeatData = FeatureDataLocal[i];
				thisprof.EnterSection("CameraRead");
				if(!cam->Read())
				{
					FeatData.Clear();
					continue;
				}
				if (!CDFRCommon::ExternalSettings.DistortedDetection)
				{
					thisprof.EnterSection("CameraUndistort");
					cam->Undistort();
				}
				thisprof.EnterSection("CameraGetFrame");
				CameraImageData &ImData = ImageDataLocal[i];
				ImData = cam->GetFrame(CDFRCommon::ExternalSettings.DistortedDetection);
				//cout << "Frame " << BufferIndex << " at " << ImData.Image.u << endl;
				if (GetScenario().size() && false)
				{
					thisprof.EnterSection("Add simulation noise");
					cv::UMat noise(ImData.Image.size(),ImData.Image.type());
					float m = 0;
					float sigma = 20;
					cv::randn(noise, m, sigma);
					add(ImData.Image, noise, ImData.Image);
					//imwrite("noised.jpg", ImData.Image);
					break;
				}
				CDFRCommon::ImageToFeatureData(CDFRCommon::ExternalSettings, cam, ImData, FeatData, *TrackerToUse, GrabTick, YoloDetector.get());

				if (RecordThisTick)
				{
					cam->Record(RecordRootPath, RecordIndex);
				}
				
				thisprof.EnterSection("");
			}
		}
		//);

		for (auto &pprof : ParallelProfilers)
		{
			ParallelProfiler += pprof;
		}

		prof.EnterSection("3D Solve");
		TrackerToUse->SolveLocationsPerObject(FeatureDataLocal, GrabTick);
		vector<ObjectData> &ObjDataLocal = ObjData[BufferIndex]; 
		ObjDataLocal = TrackerToUse->GetObjectDataVector(GrabTick);
		for (size_t camidx = 0; camidx < Cameras.size(); camidx++)
		{
			auto YoloObjects = YoloDetector->Project(ImageDataLocal[camidx], FeatureDataLocal[camidx]);
			ObjDataLocal.insert(ObjDataLocal.end(), YoloObjects.begin(), YoloObjects.end());
		}

		for (auto &i : PostProcesses)
		{
			i->Process(ImageDataLocal, FeatureDataLocal, ObjDataLocal);
		}
		
		


		BufferIndex = (BufferIndex+1)%ObjData.size();
		if (RecordThisTick)
		{
			RecordIndex++;
		}
		
		
		if (OpenGLBoard.get())
		{
			prof.EnterSection("Visualisation 3D");
			if (OpenGLBoard->IsThreaded())
			{
				if (OpenGLBoard->GetClosed())
				{
					killed = true;
					cout << "3D visualizer closed, shutting down..." << endl;
					return;
				}
				if (OpenGLBoard->IsKilled())
				{
					OpenGLBoard.reset();
				}
			}
			else
			{
				if(!OpenGLBoard->Tick(ObjectData::ToGLObjects(ObjDataLocal)))
				{
					killed = true;
					cout << "3D visualizer closed, shutting down..." << endl;
					return;
				}
			}
		}
		
		if (DirectImage.get())
		{
			prof.EnterSection("Visualisation 2D");
			if (DirectImage->IsThreaded())
			{
				if (DirectImage->GetClosed())
				{
					killed = true;
					cout << "2D visualizer closed, shutting down..." << endl;
					return;
				}
				if (DirectImage->IsKilled())
				{
					DirectImage.reset();
				}				
			}
		}

		prof.EnterSection("");
		
		if (prof.ShouldPrint())
		{
			cout << fps.GetFPSString(deltaTime) << endl;
			prof.PrintProfile();
			ParallelProfiler.PrintProfile();
		}
	}
}

int CDFRExternal::GetReadBufferIndex() const
{
	return (BufferIndex+FeatureData.size()-1)%FeatureData.size();
}

vector<CameraImageData> CDFRExternal::GetImage() const
{
	return ImageData[GetReadBufferIndex()];
}

std::vector<CameraFeatureData> CDFRExternal::GetFeatureData() const
{
	return FeatureData[GetReadBufferIndex()];
}

std::vector<ObjectData> CDFRExternal::GetObjectData() const
{
	return ObjData[GetReadBufferIndex()];
}

void CDFRExternal::Open3DVisualizer()
{
	OpenGLBoard = make_unique<BoardGL>("Cyclops", this);
	if (OpenGLBoard->IsThreaded())
	{
		
	}
	else if (OpenGLBoard->HasWindow())
	{
		OpenGLBoard->LoadTags();
		OpenGLBoard->Tick({});
	}
	else
	{
		cout << "No 3D visualizer created: No window" << endl;
		OpenGLBoard.reset();
	}
}

void CDFRExternal::OpenDirectVisualizer()
{
	DirectImage = make_unique<ImguiWindow>("External Direct Visualizer", this);
	if (!DirectImage->IsThreaded() && !DirectImage->HasWindow())
	{
		cout << "No 2D visualizer created: No window" << endl;
		DirectImage.reset();
	}
}

CDFRExternal::~CDFRExternal()
{
	cout << "External runner shutting down..." << endl;
	DirectImage.reset();
	OpenGLBoard.reset();
}