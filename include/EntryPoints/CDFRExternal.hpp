#pragma once

#include <thread>
#include <vector>
#include <array>
#include <memory>

#include <Communication/ProcessedTypes.hpp>
#include <ArucoPipeline/ObjectIdentity.hpp>
#include <ArucoPipeline/ObjectTracker.hpp>
#include <Cameras/CameraManager.hpp>
#include <Visualisation/BoardGL.hpp>
#include <Visualisation/ImguiWindow.hpp>

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
	bool direct;
	bool v3d;
	bool record = false;
	bool SegmentedDetection = true;
	bool POIDetection = false;
	bool YoloDetection = false;
	bool Denoising = false;

	//Camera manager
	std::unique_ptr<CameraManager> CameraMan;
	//Running thread handle
	std::unique_ptr<std::thread> ThreadHandle;

	//3D viz
	std::unique_ptr<BoardGL> OpenGLBoard;

	//2D viz
	std::vector<Texture> DirectTextures;
	std::unique_ptr<ImguiWindow> DirectImage;

	//data
	int BufferIndex = 0;
	CDFRTeam LastTeam = CDFRTeam::Unknown;
	ObjectTracker BlueTracker, YellowTracker, UnknownTracker;

	std::array<std::vector<CameraFeatureData>, 3> FeatureData;
	std::array<std::vector<ObjectData>, 3> ObjData;

	CDFRTeam GetTeamFromCameraPosition(std::vector<class Camera*> Cameras);

	void UpdateDirectImage(const std::vector<Camera*> &Cameras, const std::vector<CameraFeatureData> &FeatureDataLocal);

	void Open3DVisualizer();

	void OpenDirectVisualizer();
	
public:
	void SetHasNoClients(bool value)
	{
		HasNoClients = value;
	}

	void ThreadEntryPoint();

	void GetData(std::vector<CameraFeatureData> &OutFeatureData, std::vector<ObjectData> &OutObjectData);

	bool IsKilled()
	{
		return killed;
	}

	CDFRExternal(bool InDirect, bool InV3D);
	~CDFRExternal();

};




