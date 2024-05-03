
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

#include <Communication/Transport/TCPTransport.hpp>
#include <Communication/Transport/UDPTransport.hpp>
#include <Cameras/CameraManagerV4L2.hpp>
#include <Cameras/CameraManagerSimulation.hpp>
#include <Cameras/VideoCaptureCamera.hpp>

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

void CDFRExternal::SetCameraLock(bool value)
{
	LockedCamera = value;
	CDFRCommon::ExternalSettings.SolveCameraLocation = !value;
}

using ExternalProfType = ManualProfiler<false>;

void CDFRExternal::ThreadEntryPoint()
{
	ExternalProfType prof("External Global Profile");
	ExternalProfType ParallelProfiler("Parallel Cameras Detail");
	
	if (GetScenario().size())
	{
		std::filesystem::path basepath("../sim");
		CameraMan = make_unique<CameraManagerSimulation>(basepath/GetScenario());
	}
	else
	{
		CameraMan = make_unique<CameraManagerV4L2>(GetCaptureMethod(), GetCaptureConfig().filter, false);
	}

	YoloDetector = make_unique<YoloDetect>("cdfr", 4);

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
		if (LockedCamera)
		{
			LockedCamera = false;
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
		
		
		
		CDFRTeam Team = LockedTeam == CDFRTeam::Unknown ? GetTeamFromCameraPosition(Cameras) : LockedTeam;
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
		int64 GrabTick = getTickCount();
		
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

		map<const char *, CDFRCommon::Settings&> settingsmap({{"External", CDFRCommon::ExternalSettings}, {"Internal", CDFRCommon::InternalSettings}});
		ForceRecordNext = ImGui::Button("Capture next frame");

		for (auto &entry : settingsmap)
		{
			if (!ImGui::CollapsingHeader(entry.first))
			{
				continue;
			}
			ImGui::InputInt("Record interval", &entry.second.RecordInterval);
			if(ImGui::Checkbox("Record", &entry.second.record))
			{
			}
			//ImGui::Checkbox("Freeze camera position", nullptr);
			ImGui::Checkbox("Aruco Detection", &entry.second.ArucoDetection);
			ImGui::Checkbox("Distorted detection", &entry.second.DistortedDetection);
			ImGui::Checkbox("Segmented detection", &entry.second.SegmentedDetection);
			ImGui::Checkbox("POI Detection", &entry.second.POIDetection);
			ImGui::Checkbox("Yolo detection", &entry.second.YoloDetection);
			ImGui::Checkbox("Denoising", &entry.second.Denoising);
			ImGui::Spacing();
		}
		
		
		ImGui::Checkbox("Idle", &Idle);

		ImGui::Checkbox("Show Aruco", &ShowAruco);
		ImGui::Checkbox("Show Yolo", &ShowYolo);

		ImGui::Checkbox("Focus peeking", &FocusPeeking);
		
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
		auto &thisTile = tiles[camidx*DisplaysPerCam];
		const Camera* thisCamera = Cameras[camidx];
		auto ImData = thisCamera->GetFrame(CDFRCommon::ExternalSettings.DistortedDetection);
		Size Resolution = ImData.Image.size();
		if (FocusPeeking)
		{
			auto POIs = UnknownTracker.GetPointsOfInterest();
			auto POIRects = GetPOIRects(POIs, Resolution, GetFeatureData()[camidx].CameraTransform, 
				ImData.CameraMatrix, ImData.DistanceCoefficients);
			auto POI = POIRects[POIs.size()/2];
			thisTile.height = ImageSize.height;
			thisTile.width = ImageSize.width;
			thisTile.x = -POI.x+(WindowSize.width-POI.width)/2;
			thisTile.y = -POI.y+(WindowSize.height-POI.height)/2;
		}
		
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
		const CameraFeatureData &FeatData = FeatureDataLocal[camidx];
		Rect SourceRemap(Point(0,0), Resolution);
		Rect DestRemap = tiles[camidx];
		//draw aruco
		if (ShowAruco && !FocusPeeking)
		{
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
		if (ShowYolo && !FocusPeeking)
		{
			for (size_t detidx = 0; detidx < FeatData.YoloDetections.size(); detidx++)
			{
				auto det = FeatData.YoloDetections[detidx];
				uint8_t r,g,b;
				HsvConverter::getRgbFromHSV(1530*det.Class/YoloDetector->GetNumClasses(), 255, 255, r, g, b);
				uint32_t color = IM_COL32(r,g,b,det.Confidence*255);
				
				Point2d textpos(0,0);
				auto tl = ImageRemap<double>(SourceRemap, DestRemap, det.Corners.tl());
				auto br = ImageRemap<double>(SourceRemap, DestRemap, det.Corners.br());
				DrawList->AddRect(tl, br, color);
				//text with class and confidence
				string text = YoloDetector->GetClassName(det.Class) + string("/") + to_string(det.Confidence);
				DrawList->AddText(nullptr, 16, tl, color, text.c_str());
			}
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