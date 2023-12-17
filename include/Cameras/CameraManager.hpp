#pragma once

#include <vector>
#include <set>
#include <string>
#include <iostream>
#include <shared_mutex>
#include <thread>
#include <memory>

#include <Cameras/Camera.hpp>
#include <GlobalConf.hpp>


//Overseer class for the cameras
//Creates a camera objet when a new camera is detected
//Deletes a camera object when too many errors have been accumulated
//Could be made so that the creation and detection is run on another thread
class CameraManager
{
private:
	CameraStartType Start;
	std::string Filter;
	bool AllowNoCalib;
	//Boths paths are protected by pathmutex
	std::set<std::string> usedpaths; //Paths used by the cameras
	std::set<std::string> blockedpaths; //paths that have been tried but failed at some point during init, so that we don't try again

	std::shared_mutex pathmutex, cammutex;
	std::unique_ptr<std::thread> scanthread;
	bool killmutex;

	std::vector<std::shared_ptr<Camera>> Cameras, NewCameras; //List of cameras. NewCameras is protected by cammutex, Cameras only belongs to Tick

public:
	//Function called when a new camera is to be created. Return nullptr if you want to veto that creation
	//Will be called in a separate thread 
	std::function<Camera*(VideoCaptureCameraSettings)> StartCamera; 
	//When a new camera is added, called when Tick is called
	std::function<void(Camera*)> RegisterCamera;
	std::function<bool(Camera*)> StopCamera; //Function called before a camera is going to be deleted (as a head's up)


	CameraManager(CameraStartType InStart, std::string InFilter, bool InAllowNoCalib = false)
		:Start(InStart), Filter(InFilter), AllowNoCalib(InAllowNoCalib)
	{
		scanthread = nullptr;
		killmutex = false;
	}
	
	~CameraManager()
	{
		killmutex = true;
		scanthread->join();
	}
	//Check if the name can fit the filter to blacklist or whitelist certain cameras based on name
	static bool DeviceInFilter(v4l2::devices::DEVICE_INFO device, std::string Filter);

	//Gather calibration info and start setting (fps, resolution, method...) before starting the camera
	static VideoCaptureCameraSettings DeviceToSettings(v4l2::devices::DEVICE_INFO device, CameraStartType Start);

	static std::vector<VideoCaptureCameraSettings> autoDetectCameras(CameraStartType Start, std::string Filter, bool silent = true);

public:
	//Call Tick to remove misbehaving cameras and get the current cameras
	std::vector<Camera*> Tick();

	void StartScanThread();

private:
	void ScanWorker();
};
