
#include "EntryPoints/CDFRExternal.hpp"
#include <EntryPoints/CDFRCommon.hpp>

#include <DetectFeatures/ArucoDetect.hpp>

#include <Visualisation/BoardGL.hpp>
#include <Visualisation/ImguiWindow.hpp>
#include <Misc/ManualProfiler.hpp>
#include <Misc/math2d.hpp>
#include <Cameras/Calibfile.hpp>

#include <Communication/Transport/TCPTransport.hpp>
#include <Communication/Transport/UDPTransport.hpp>
#include <thread>
#include <memory>

CDFRTeam CDFRExternal::GetTeamFromCameraPosition(vector<Camera*> Cameras)
{
	if (Cameras.size() == 0)
	{
		return CDFRTeam::Unknown;
	}
	int blues = 0, greens = 0;
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
		for (const auto [team, positions] : CameraPos)
		{
			for (const auto position : positions)
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

using ExternalProfType = ManualProfiler<false>;

void CDFRExternal::ThreadEntryPoint()
{
	ExternalProfType prof("External Global Profile");
	ExternalProfType ParallelProfiler("Parallel Cameras Detail");
	CameraManager CameraMan(GetCaptureMethod(), GetCaptureConfig().filter, false);

	auto& Detector = GetArucoDetector();

	//display/debug section
	FrameCounter fps;
	
	ObjectTracker bluetracker, yellowtracker;

	unique_ptr<StaticObject> boardobj = make_unique<StaticObject>(false, "board");
	bluetracker.RegisterTrackedObject(boardobj.get()); 
	yellowtracker.RegisterTrackedObject(boardobj.get());
	//TrackerCube* robot1 = new TrackerCube({51, 52, 54, 55}, 0.06, 0.0952, "Robot1");
	//TrackerCube* robot2 = new TrackerCube({57, 58, 59, 61}, 0.06, 0.0952, "Robot2");
	//bluetracker.RegisterTrackedObject(robot1);
	//bluetracker.RegisterTrackedObject(robot2);
	
	BoardGL OpenGLBoard;
	std::unique_ptr<ImguiWindow> DirectImage;
	vector<Texture> DirectTextures;
	unique_ptr<TrackerCube> blue1 = make_unique<TrackerCube>(vector<int>({51, 52, 53, 54, 55}), 0.05, 85.065/1000.0, "blue1");
	unique_ptr<TrackerCube> blue2 = make_unique<TrackerCube>(vector<int>({56, 57, 58, 59, 60}), 0.05, 85.065/1000.0, "blue2");
	unique_ptr<TrackerCube> yellow1 = make_unique<TrackerCube>(vector<int>({71, 72, 73, 74, 75}), 0.05, 85.065/1000.0, "yellow1");
	unique_ptr<TrackerCube> yellow2 = make_unique<TrackerCube>(vector<int>({76, 77, 78, 79, 80}), 0.05, 85.065/1000.0, "yellow2");
	bluetracker.RegisterTrackedObject(blue1.get());
	bluetracker.RegisterTrackedObject(blue2.get());

	yellowtracker.RegisterTrackedObject(yellow1.get());
	yellowtracker.RegisterTrackedObject(yellow2.get());

	vector<unique_ptr<TopTracker>> TopTrackers;
	for (int i = 1; i < 11; i++)
	{
		TopTracker* tt = new TopTracker(i, 0.07, "top tracker " + std::to_string(i));
		TopTrackers.emplace_back(tt);
		bluetracker.RegisterTrackedObject(tt);
		yellowtracker.RegisterTrackedObject(tt);
	}
	
	
	//OpenGLBoard.InspectObject(blue1);
	if (v3d)
	{
		OpenGLBoard.Start();
		OpenGLBoard.LoadTags();
		OpenGLBoard.Tick({});
	}
	if (direct)
	{
		DirectImage = make_unique<ImguiWindow>();
	}
	

	
	
	//track and untrack cameras dynamically
	if (GetRunType() == RunType::Simulate)
	{
		CameraMan.StartCamera = [](VideoCaptureCameraSettings settings) -> Camera*
		{
			settings.StartType = CameraStartType::PLAYBACK;
			settings.StartPath = "../sim/sim1.mp4";
			readCameraParameters("../sim/cal", settings.CameraMatrix, settings.distanceCoeffs, settings.Resolution);

			Camera* cam = new VideoCaptureCamera(make_shared<VideoCaptureCameraSettings>(settings));
			if(!cam->StartFeed())
			{
				cerr << "Failed to start feed @" << settings.DeviceInfo.device_description << endl;
				delete cam;
				return nullptr;
			}
			
			return cam;
		};
	}
	else
	{
		CameraMan.StartCamera = [](VideoCaptureCameraSettings settings) -> Camera*
		{
			Camera* cam = new VideoCaptureCamera(make_shared<VideoCaptureCameraSettings>(settings));
			if(!cam->StartFeed())
			{
				cerr << "Failed to start feed @" << settings.DeviceInfo.device_description << endl;
				delete cam;
				return nullptr;
			}
			
			return cam;
		};
	}
	
	
	CameraMan.RegisterCamera = [&bluetracker, &yellowtracker](Camera* cam) -> void
	{
		bluetracker.RegisterTrackedObject(cam);
		yellowtracker.RegisterTrackedObject(cam);
		cout << "Registering new camera @" << cam << ", name " << cam->GetName() << endl;
	};
	CameraMan.StopCamera = [&bluetracker, &yellowtracker](Camera* cam) -> bool
	{
		bluetracker.UnregisterTrackedObject(cam);
		yellowtracker.UnregisterTrackedObject(cam);
		cout << "Unregistering camera @" << cam << endl;
		
		return true;
	};

	CameraMan.StartScanThread();

	int lastmarker = 0;
	CDFRTeam LastTeam = CDFRTeam::Unknown;
	while (!killed)
	{
		if(Idle)
		{
			this_thread::sleep_for(chrono::milliseconds(10));
			continue;
		}
		double deltaTime = fps.GetDeltaTime();
		prof.EnterSection("CameraManager Tick");
		vector<Camera*> Cameras = CameraMan.Tick();
		CDFRTeam Team = GetTeamFromCameraPosition(Cameras);
		if (Team != LastTeam)
		{
			const string teamname = TeamNames.at(Team);
			cout << "Detected team change : to " << teamname <<endl; 
			LastTeam = Team;
		}
		ObjectTracker* TrackerToUse = &bluetracker;
		if (Team == CDFRTeam::Yellow)
		{
			TrackerToUse = &yellowtracker;
		}
		
		prof.EnterSection("Camera Gather Frames");
		int64 GrabTick = getTickCount();
		
		for (int i = 0; i < Cameras.size(); i++)
		{
			Cameras[i]->Grab();
		}

		int NumCams = Cameras.size();
		vector<CameraFeatureData> &FeatureDataLocal = FeatureData[BufferIndex];
		vector<bool> CamerasWithPosition;
		vector<ExternalProfType> ParallelProfilers;
		FeatureDataLocal.resize(NumCams);
		CamerasWithPosition.resize(NumCams);
		ParallelProfilers.resize(NumCams);
		prof.EnterSection("Parallel Cameras");

		//grab frames
		//read frames
		//undistort
		//detect aruco and yolo

		parallel_for_(Range(0, Cameras.size()), 
		[&Cameras, &FeatureDataLocal, &CamerasWithPosition, TrackerToUse, GrabTick, &ParallelProfilers]
		(Range InRange)
		{
			for (int i = InRange.start; i < InRange.end; i++)
			{
				auto &thisprof = ParallelProfilers[i];
				thisprof.EnterSection("CameraRead");
				Camera* cam = Cameras[i];
				cam->Read();
				thisprof.EnterSection("CameraUndistort");
				cam->Undistort();
				thisprof.EnterSection("CameraGetFrame");
				CameraImageData ImData = cam->GetFrame(false);
				thisprof.EnterSection("DetectAruco");
				CameraFeatureData &FeatData = FeatureDataLocal[i];
				FeatData.CopyEssentials(ImData);
				DetectAruco(ImData, FeatData);
				CamerasWithPosition[i] = TrackerToUse->SolveCameraLocation(FeatData);
				if (CamerasWithPosition[i])
				{
					cam->SetLocation(FeatData.CameraTransform, GrabTick);
					//cout << "Camera has location" << endl;
				}
				FeatData.CameraTransform = cam->GetLocation();
			}
		});

		for (auto &pprof : ParallelProfilers)
		{
			ParallelProfiler += pprof;
		}

		int viewsidx = 0;

		prof.EnterSection("3D Solve");
		TrackerToUse->SolveLocationsPerObject(FeatureDataLocal, GrabTick);
		vector<ObjectData> &ObjDataLocal = ObjData[BufferIndex]; 
		ObjDataLocal = TrackerToUse->GetObjectDataVector(GrabTick);
		ObjectData TeamPacket(ObjectType::Team, TeamNames.at(Team));
		ObjDataLocal.insert(ObjDataLocal.begin(), TeamPacket); //insert team as the first object

		prof.EnterSection("Visualisation");
		
		if (v3d)
		{
			if(!OpenGLBoard.Tick(ObjectData::ToGLObjects(ObjDataLocal)))
			{
				killed = true;
				return;
			}
		}
		if (direct)
		{
			DirectImage->StartFrame();
			int DisplaysPerCam = 2;
			int NumDisplays = Cameras.size()*DisplaysPerCam;
			if (DirectTextures.size() != NumDisplays)
			{
				DirectTextures.resize(NumDisplays);
			}
			Size WindowSize = DirectImage->GetWindowSize();
			auto tiles = DistributeViewports(GetCaptureConfig().FrameSize, WindowSize, NumDisplays);
			for (int camidx = 0; camidx < Cameras.size(); camidx++)
			{
				if (Cameras[camidx]->errors != 0)
				{
					continue;
				}
				auto ImData = Cameras[camidx]->GetFrame(true);
				DirectTextures[camidx*DisplaysPerCam].LoadFromUMat(ImData.Image);
				DirectImage->AddImageToBackground(DirectTextures[camidx*DisplaysPerCam], tiles[camidx*DisplaysPerCam]);

				if (DisplaysPerCam > 1)
				{
					UMat gray, thresholded, tcol;
					gray = PreprocessArucoImage(ImData.Image);
					auto& DetParams = GetArucoDetector().getDetectorParameters();
					adaptiveThreshold(gray, thresholded, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY_INV, DetParams.adaptiveThreshWinSizeMax, DetParams.adaptiveThreshConstant);
					cvtColor(thresholded, tcol, COLOR_GRAY2BGR);
					DirectTextures[camidx*DisplaysPerCam+1].LoadFromUMat(tcol);
					DirectImage->AddImageToBackground(DirectTextures[camidx*DisplaysPerCam+1], tiles[camidx*DisplaysPerCam+1]);
				}

				//draw aruco
				auto DrawList = ImGui::GetForegroundDrawList();
				CameraFeatureData &FeatData = FeatureDataLocal[camidx];
				Rect SourceRemap(Point(0,0), GetCaptureConfig().FrameSize);
				Rect DestRemap = tiles[camidx];
				for (int arucoidx = 0; arucoidx < FeatData.ArucoIndices.size(); arucoidx++)
				{
					auto& corners = FeatData.ArucoCorners[arucoidx];
					uint32_t color = IM_COL32(0,255,0,255);
					if (FeatData.ArucoCornersReprojected[arucoidx].size() != 0)
					{
						//cout << arucoidx << " is reprojected" << endl;
						corners = FeatData.ArucoCornersReprojected[arucoidx];
						color = IM_COL32(0,0,255,255);
					}
					
					Point2d textpos(0,0);
					for (auto cornerit = corners.begin(); cornerit != corners.end(); cornerit++)
					{
						auto vizpos = ImageRemap<double>(SourceRemap, DestRemap, *cornerit);
						textpos.x = max<double>(textpos.x, vizpos.x);
						textpos.y = max<double>(textpos.y, vizpos.y);
						if (cornerit == corners.begin())
						{
							Point2d size = Point2d(2,2);
							DrawList->AddRect(vizpos-size, vizpos+size, color);
						}
						
						DrawList->PathLineTo(vizpos);
					}
					DrawList->PathStroke(color, ImDrawFlags_Closed, 2);
					string text = to_string(FeatData.ArucoIndices[arucoidx]);
					DrawList->AddText(textpos, color, &*text.begin(), &*text.end());
				}
			}
			if(!DirectImage->EndFrame())
			{
				killed = true;
				return;
			}
		}
		
		
		
		prof.EnterSection("");
		
		
		
		if (prof.ShouldPrint())
		{
			cout << fps.GetFPSString(deltaTime) << endl;
			prof.PrintProfile();
			ParallelProfiler.PrintProfile();
		}
		BufferIndex = (BufferIndex+1)%ObjData.size();
	}
}

void CDFRExternal::GetData(std::vector<CameraFeatureData> &OutFeatureData, std::vector<ObjectData> &OutObjectData)
{
	int SelectedBuffer = (BufferIndex+FeatureData.size()-1)%FeatureData.size();
	cout << "Copying " << FeatureData[SelectedBuffer].size() << " features and " << ObjData[SelectedBuffer].size() << " objects" << endl;
	OutFeatureData = FeatureData[SelectedBuffer];
	OutObjectData = ObjData[SelectedBuffer];
}

CDFRExternal::CDFRExternal(bool InDirect, bool InV3D)
	:direct(InDirect), v3d(InV3D)
{
	ThreadHandle = make_unique<thread>(&CDFRExternal::ThreadEntryPoint, this);
	assert(ObjData.size() == FeatureData.size());
	assert(FeatureData.size() > 0);
}

CDFRExternal::~CDFRExternal()
{
	killed = true;
	if (ThreadHandle)
	{
		ThreadHandle->join();
	}
}