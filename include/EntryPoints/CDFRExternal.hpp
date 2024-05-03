#pragma once

#include <thread>
#include <vector>
#include <array>
#include <memory>
#include <filesystem>

#include <Communication/ProcessedTypes.hpp>
#include <ArucoPipeline/ObjectIdentity.hpp>
#include <ArucoPipeline/ObjectTracker.hpp>

class CDFRExternal
{
private:
	//State
	bool killed = false;
	bool HasNoClients = true;
	//Low power modes : disables aruco detection and reduces visualizers to 2fps
	//Idle : manual low power mode
	bool Idle = false, LastIdle = false;
	//Sleep : If we have no data or noone is seeing (no clients + no visualizers), enter sleep
	bool Sleep = false, LastSleep = false;

	//Settings
	bool FocusPeeking = false;
	int RecordTick = 0, RecordIndex=0;
	bool ForceRecordNext = false;
	std::filesystem::path RecordRootPath;

	std::unique_ptr<class YoloDetect> YoloDetector;

	//Camera manager
	std::unique_ptr<class CameraManager> CameraMan;
	//Running thread handle
	std::unique_ptr<std::thread> ThreadHandle;

	//3D viz
	std::unique_ptr<class BoardGL> OpenGLBoard;

	//2D viz
	std::vector<struct Texture> DirectTextures;
	std::unique_ptr<class ImguiWindow> DirectImage;

	//data
	int BufferIndex = 0;
	CDFRTeam LastTeam = CDFRTeam::Unknown, LockedTeam = CDFRTeam::Unknown;
	bool LockedCamera = false; //lock camera position
	ObjectTracker BlueTracker, YellowTracker, UnknownTracker;
	std::array<std::vector<CameraImageData>, 3> ImageData;
	std::array<std::vector<CameraFeatureData>, 3> FeatureData;
	std::array<std::vector<ObjectData>, 3> ObjData;

	//Show Aruco or Yolo
	bool ShowAruco = true, ShowYolo = true;

	CDFRTeam GetTeamFromCameraPosition(std::vector<class Camera*> Cameras);

	void UpdateDirectImage(const std::vector<class Camera*> &Cameras, const std::vector<CameraFeatureData> &FeatureDataLocal);

	void Open3DVisualizer();

	void OpenDirectVisualizer();
	
public:
	void SetHasNoClients(bool value)
	{
		HasNoClients = value;
	}

	void SetIdle(bool value)
	{
		Idle = value;
	}

	void SetCameraLock(bool value);

	void SetTeamLock(CDFRTeam value)
	{
		LockedTeam = value;
	}

	void ThreadEntryPoint();

	int GetReadBufferIndex() const;

	std::vector<CameraImageData> GetImage() const;

	std::vector<CameraFeatureData> GetFeatureData() const;

	std::vector<ObjectData> GetObjectData() const;

	bool IsKilled()
	{
		return killed;
	}

	CDFRExternal();
	~CDFRExternal();

};




