#pragma once

#include <vector>
#include <set>
#include <string>
#include <iostream>
#include <shared_mutex>
#include <thread>
#include <memory>

#include <Cameras/Camera.hpp>


//Overseer class for the cameras
//Creates a camera objet when a new camera is detected
//Deletes a camera object when too many errors have been accumulated
//Could be made so that the creation and detection is run on another thread
class CameraManager
{
protected:
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
	std::function<std::shared_ptr<Camera>(VideoCaptureCameraSettings)> StartCamera; 
	//When a new camera is added, called when Tick is called
	std::function<void(std::shared_ptr<Camera>)> RegisterCamera;
	std::function<bool(std::shared_ptr<Camera>)> StopCamera; //Function called before a camera is going to be deleted (as a head's up)


	CameraManager()
	{
		scanthread = nullptr;
		killmutex = false;
	}
	
	virtual ~CameraManager()
	{
		killmutex = true;
		scanthread->join();
	}

public:
	//Call Tick to remove misbehaving cameras and get the current cameras
	virtual std::vector<Camera*> Tick();

	std::vector<Camera*> GetCameras();

	void StartScanThread();

protected:
	virtual void ScanWorker();
};
