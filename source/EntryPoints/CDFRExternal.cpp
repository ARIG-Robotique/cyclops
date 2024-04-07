
#include "EntryPoints/CDFRExternal.hpp"
#include <EntryPoints/CDFRCommon.hpp>

#include <DetectFeatures/ArucoDetect.hpp>
#include <DetectFeatures/YoloDetect.hpp>

#include <Visualisation/BoardGL.hpp>
#include <Visualisation/ImguiWindow.hpp>
#include <Misc/ManualProfiler.hpp>
#include <Misc/math2d.hpp>
#include <Cameras/Calibfile.hpp>
#include <opencv2/photo.hpp>

#include <Communication/Transport/TCPTransport.hpp>
#include <Communication/Transport/UDPTransport.hpp>
#include <thread>
#include <memory>


CDFRExternal::CDFRExternal(bool InDirect, bool InV3D)
	:direct(InDirect), v3d(InV3D)
{

	assert(ObjData.size() == FeatureData.size());
	assert(FeatureData.size() > 0);

	auto boardobj = make_shared<StaticObject>(false, "board");
	BlueTracker.RegisterTrackedObject(boardobj); 
	YellowTracker.RegisterTrackedObject(boardobj);
	UnknownTracker.RegisterTrackedObject(boardobj);
	auto SolarPanels = make_shared<SolarPanel>();
	BlueTracker.RegisterTrackedObject(SolarPanels); 
	YellowTracker.RegisterTrackedObject(SolarPanels);
	UnknownTracker.RegisterTrackedObject(SolarPanels);
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
		auto &tracker = i==0 ? BlueTracker : YellowTracker;
		for (size_t j = 0; j < PAMINames.size(); j++)
		{
			auto pamitracker = make_shared<TopTracker>(51+i*20+j, 0.07, PAMINames[j], 0.112);
			tracker.RegisterTrackedObject(pamitracker);
		}
	}
	

	for (int i = 1; i < 11; i++)
	{
		auto tt = make_shared<TopTracker>(i, 0.07, "Robot " + std::to_string(i), 0.450);
		BlueTracker.RegisterTrackedObject(tt);
		YellowTracker.RegisterTrackedObject(tt);
		UnknownTracker.RegisterTrackedObject(tt);
	}

	ThreadHandle = make_unique<thread>(&CDFRExternal::ThreadEntryPoint, this);
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

using ExternalProfType = ManualProfiler<false>;

void CDFRExternal::ThreadEntryPoint()
{
	ExternalProfType prof("External Global Profile");
	ExternalProfType ParallelProfiler("Parallel Cameras Detail");
	
	
	switch (GetRunType())
	{
		case RunType::Normal:
			CameraMan = make_unique<CameraManagerV4L2>(GetCaptureMethod(), GetCaptureConfig().filter, false);
			break;
		case RunType::Simulate:
			CameraMan = make_unique<CameraManagerSimulation>("../sim/scenario0.json");
			break;
	}

	//display/debug section
	FrameCounter fps;
	
	//OpenGLBoard.InspectObject(blue1);
	if (v3d)
	{
		Open3DVisualizer();
	}
	if (direct)
	{
		OpenDirectVisualizer();
	}
	
	RecordRootPath = TimeToStr();
	
	
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

	CameraMan->StartScanThread();

	
	while (!killed)
	{
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
		
		
		
		CDFRTeam Team = GetTeamFromCameraPosition(Cameras);
		if (Team != LastTeam && Cameras.size() > 0)
		{
			cout << "Detected team change : to " << Team <<endl;
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

		bool RecordThisTick = false;
		if (RecordTick >= RecordInterval-1)
		{
			RecordTick = 0;
			RecordThisTick = record;
		}
		else if (Cameras.size() > 0 && record)
		{
			RecordTick++;
		}
		
		
		
		prof.EnterSection("Camera Gather Frames");
		int64 GrabTick = getTickCount();
		
		for (size_t i = 0; i < Cameras.size(); i++)
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
					CamerasWithPosition[i] = false;
					continue;
				}
				//thisprof.EnterSection("CameraUndistort");
				//cam->Undistort();
				thisprof.EnterSection("CameraGetFrame");
				CameraImageData ImData = cam->GetFrame(true);
				switch (GetRunType())
				{
					case RunType::Normal:
						break;
					//add simulated noise
					case RunType::Simulate:
						break;
						thisprof.EnterSection("Add simulation noise");
						cv::UMat noise(ImData.Image.size(),ImData.Image.type());
						float m = 0;
						float sigma = 20;
						cv::randn(noise, m, sigma);
						add(ImData.Image, noise, ImData.Image);
						//imwrite("noised.jpg", ImData.Image);
						break;
				}
				if (Denoising)
				{
					thisprof.EnterSection("Denoise");
					fastNlMeansDenoising(ImData.Image, ImData.Image, 10);
					imwrite("denoised.jpg", ImData.Image);
				}
				FeatData.CopyEssentials(ImData);
				if (YoloDetection)
				{
					thisprof.EnterSection("Detect Yolo");
					DetectYolo(ImData, FeatData);
				}
				thisprof.EnterSection("Detect Aruco");
				if (SegmentedDetection)
				{
					DetectArucoSegmented(ImData, FeatData, 200, Size(4,3));
				}
				else
				{
					DetectAruco(ImData, FeatData);
				}
				thisprof.EnterSection("3D Solve Camera");
				CamerasWithPosition[i] = TrackerToUse->SolveCameraLocation(FeatData);
				if (CamerasWithPosition[i])
				{
					cam->SetLocation(FeatData.CameraTransform, GrabTick);
					//cout << "Camera has location" << endl;
				}
				FeatData.CameraTransform = cam->GetLocation();
				if (POIDetection)
				{
					thisprof.EnterSection("Detect Aruco POIs");
					const auto &POIs = TrackerToUse->GetPointsOfInterest();
					DetectArucoPOI(ImData, FeatData, POIs);
				}
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
		ObjectData TeamPacket(ObjectType::Team, TeamNames.at(Team));
		ObjDataLocal.insert(ObjDataLocal.begin(), TeamPacket); //insert team as the first object


		BufferIndex = (BufferIndex+1)%ObjData.size();
		if (RecordThisTick)
		{
			RecordIndex++;
		}
		
		
		if (OpenGLBoard.get())
		{
			prof.EnterSection("Visualisation 3D");
			if(!OpenGLBoard->Tick(ObjectData::ToGLObjects(ObjDataLocal)))
			{
				killed = true;
				cout << "3D visualizer closed, shutting down..." << endl;
				return;
			}
		}
		if (DirectImage.get())
		{
			prof.EnterSection("Visualisation 2D");
			UpdateDirectImage(Cameras, FeatureDataLocal);
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

void CDFRExternal::GetData(std::vector<CameraFeatureData> &OutFeatureData, std::vector<ObjectData> &OutObjectData)
{
	int SelectedBuffer = (BufferIndex+FeatureData.size()-1)%FeatureData.size();
	cout << "Copying " << FeatureData[SelectedBuffer].size() << " features and " << ObjData[SelectedBuffer].size() << " objects" << endl;
	OutFeatureData = FeatureData[SelectedBuffer];
	OutObjectData = ObjData[SelectedBuffer];
}

void CDFRExternal::Open3DVisualizer()
{
	OpenGLBoard = make_unique<BoardGL>();
	if (OpenGLBoard->HasWindow())
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
	DirectImage = make_unique<ImguiWindow>("External Direct Visualizer");
	if (!DirectImage->HasWindow())
	{
		cout << "No 2D visualizer created: No window" << endl;
		DirectImage.reset();
	}
}

void CDFRExternal::UpdateDirectImage(const vector<Camera*> &Cameras, const vector<CameraFeatureData> &FeatureDataLocal)
{
	bool selfkill = false;
	DirectImage->StartFrame();
	int DisplaysPerCam = 1;
	int NumDisplays = Cameras.size()*DisplaysPerCam;
	if ((int)DirectTextures.size() != NumDisplays)
	{
		DirectTextures.resize(NumDisplays);
	}
	Size WindowSize = DirectImage->GetWindowSize();
	Size ImageSize = WindowSize;
	if (Cameras.size() > 0)
	{
		ImageSize = Cameras[0]->GetCameraSettings()->Resolution;
	}

	if (ImGui::Begin("Settings"))
	{
		ImGui::InputInt("Record interval", &RecordInterval);
		if(ImGui::Checkbox("Record", &record))
		{
		}
		if (!OpenGLBoard)
		{
			if (ImGui::Button("Open 3D vizualiser"))
			{
				Open3DVisualizer();
			}
		}
		else
		{
			if (ImGui::Button("Close 3D vizualiser"))
			{
				OpenGLBoard.reset();
			}
		}
		
		if (ImGui::Button("Close this window"))
		{
			selfkill=true;
		}
		//ImGui::Checkbox("Freeze camera position", nullptr);
		ImGui::Checkbox("Segmented detection", &SegmentedDetection);
		ImGui::Checkbox("POI Detection", &POIDetection);
		ImGui::Checkbox("Yolo detection", &YoloDetection);
		ImGui::Checkbox("Denoising", &Denoising);
		ImGui::Checkbox("Idle", &Idle);
	}
	ImGui::End();
	
	
	auto tiles = DistributeViewports(ImageSize, WindowSize, NumDisplays);
	assert(tiles.size() == Cameras.size());
	//show cameras and arucos
	for (size_t camidx = 0; camidx < Cameras.size(); camidx++)
	{
		if (Cameras[camidx]->errors != 0)
		{
			continue;
		}
		const auto &thisTile = tiles[camidx*DisplaysPerCam];
		const Camera* thisCamera = Cameras[camidx];
		auto ImData = thisCamera->GetFrame(true);
		Size Resolution(ImData.Image.cols, ImData.Image.rows);
		DirectTextures[camidx*DisplaysPerCam].LoadFromUMat(ImData.Image);
		DirectImage->AddImageToBackground(DirectTextures[camidx*DisplaysPerCam], thisTile);

		auto DrawList = ImGui::GetForegroundDrawList();
		{
			ostringstream CameraTextStream;
			CameraTextStream << thisCamera->GetLocation().translation() <<endl;
			CameraTextStream << LastTeam << endl;
			string CameraText = CameraTextStream.str();
			DrawList->AddText(NULL, 12, ImVec2(thisTile.x, thisTile.y), IM_COL32(255,255,255,255), CameraText.c_str());
		}
		//draw aruco
		const CameraFeatureData &FeatData = FeatureDataLocal[camidx];
		Rect SourceRemap(Point(0,0), Resolution);
		Rect DestRemap = tiles[camidx];
		for (size_t arucoidx = 0; arucoidx < FeatData.ArucoIndices.size(); arucoidx++)
		{
			auto corners = FeatData.ArucoCorners[arucoidx];
			uint32_t color = IM_COL32(255, 128, 255, 128);
			if (FeatData.ArucoCornersReprojected[arucoidx].size() != 0)
			{
				//cout << arucoidx << " is reprojected" << endl;
				corners = FeatData.ArucoCornersReprojected[arucoidx];
				color = IM_COL32(128, 255, 255, 128);
			}
			
			Point2d textpos(0,0);
			for (auto cornerit = corners.cbegin(); cornerit != corners.cend(); cornerit++)
			{
				auto vizpos = ImageRemap<double>(SourceRemap, DestRemap, *cornerit);
				textpos.x = max<double>(textpos.x, vizpos.x);
				textpos.y = max<double>(textpos.y, vizpos.y);
				if (cornerit == corners.begin())
				{
					//corner0 square
					Point2d size = Point2d(2,2);
					DrawList->AddRect(vizpos-size, vizpos+size, color);
				}
				//start aruco contour
				DrawList->PathLineTo(vizpos);
			}
			//finish aruco contour
			DrawList->PathStroke(color, ImDrawFlags_Closed, 2);
			//text with aruco number
			string text = to_string(FeatData.ArucoIndices[arucoidx]);
			DrawList->AddText(nullptr, 16, textpos, color, text.c_str());
		}
		//display segments of segmented detection
		for (size_t segmentidx = 0; segmentidx < FeatData.ArucoSegments.size(); segmentidx++)
		{
			auto &segment = FeatData.ArucoSegments[segmentidx];
			auto tl = ImageRemap<double>(SourceRemap, DestRemap, segment.tl());
			auto br = ImageRemap<double>(SourceRemap, DestRemap, segment.br());
			DrawList->AddRect(tl, br, IM_COL32(255, 255, 255, 64));
		}
		
	}
	if(!DirectImage->EndFrame())
	{
		killed = true;
		cout << "2D visualizer closed, shutting down..." << endl;
		return;
	}
	if (selfkill)
	{
		DirectImage.reset();
	}
	
}


CDFRExternal::~CDFRExternal()
{
	killed = true;
	if (ThreadHandle)
	{
		ThreadHandle->join();
	}
	for (auto &&i : DirectTextures)
	{
		i.Release();
	}
	
}