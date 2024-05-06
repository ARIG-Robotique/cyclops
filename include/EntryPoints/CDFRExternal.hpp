#pragma once

#include <thread>
#include <vector>
#include <array>
#include <memory>
#include <filesystem>

#include <Communication/ProcessedTypes.hpp>
#include <ArucoPipeline/ObjectIdentity.hpp>
#include <ArucoPipeline/ObjectTracker.hpp>
#include <Cameras/ImageTypes.hpp>
#include <Misc/FrameCounter.hpp>
#include <Misc/Task.hpp>
#include <PostProcessing/PostProcess.hpp>

class CDFRExternal : public Task
{
private:
	//State
	bool HasNoClients = true;
	//Low power modes : disables aruco detection and reduces visualizers to 2fps
	//Idle : manual low power mode
	bool Idle = false, LastIdle = false;
	//Sleep : If we have no data or noone is seeing (no clients + no visualizers), enter sleep
	bool Sleep = false, LastSleep = false;

	//Settings
	int RecordTick = 0, RecordIndex=0;
protected:
	bool ForceRecordNext = false;
	FrameCounter DetectionFrameCounter;
private:
	std::filesystem::path RecordRootPath;

	std::unique_ptr<class YoloDetect> YoloDetector;

	//Camera manager
	std::unique_ptr<class CameraManager> CameraMan;

	std::vector<std::unique_ptr<PostProcess>> PostProcesses;

protected:
	//3D viz
	std::unique_ptr<class BoardGL> OpenGLBoard;

	//2D viz
	std::unique_ptr<class ImguiWindow> DirectImage;

private:
	//data
	int BufferIndex = 0;
	CDFRTeam LastTeam = CDFRTeam::Unknown, LockedTeam = CDFRTeam::Unknown;
	ObjectTracker BlueTracker, YellowTracker, UnknownTracker;
	std::array<std::vector<CameraImageData>, 3> ImageData;
	std::array<std::vector<CameraFeatureData>, 3> FeatureData;
	std::array<std::vector<ObjectData>, 3> ObjData;

	CDFRTeam GetTeamFromCameraPosition(std::vector<class Camera*> Cameras);

	void UpdateDirectImage(const std::vector<class Camera*> &Cameras, const std::vector<CameraFeatureData> &FeatureDataLocal);

protected:
	void Open3DVisualizer();

	void OpenDirectVisualizer();
	
public:
	void SetHasNoClients(bool value)
	{
		HasNoClients = value;
	}

	bool GetIdle() const 
	{
		return Idle;
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

	CDFRTeam GetTeam();

	virtual void ThreadEntryPoint() override;

	int GetReadBufferIndex() const;

	std::vector<CameraImageData> GetImage() const;

	std::vector<CameraFeatureData> GetFeatureData() const;

	std::vector<ObjectData> GetObjectData() const;

	CDFRExternal();
	virtual ~CDFRExternal();

	friend class ImguiWindow;
};




