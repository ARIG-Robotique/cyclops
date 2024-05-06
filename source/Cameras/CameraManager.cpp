#include "Cameras/CameraManager.hpp"

using namespace std;

vector<Camera*> CameraManager::Tick()
{
	for (size_t i = 0; i < Cameras.size(); i++)
	{
		if (Cameras[i]->errors >= 20)
		{
			std::cerr << "Detaching camera @ " << Cameras[i]->GetName() << std::endl;
			std::string pathtofind = dynamic_cast<const VideoCaptureCameraSettings*>(Cameras[i]->GetCameraSettings())->DeviceInfo.device_paths[0];
			StopCamera(Cameras[i]);
			
			unique_lock lock(pathmutex);
			usedpaths.erase(pathtofind);
			Cameras.erase(std::next(Cameras.begin(), i));
			i--;
		}
	}
	{
		unique_lock lock(cammutex);
		for (auto &Camera : NewCameras)
		{
			RegisterCamera(Camera);
			Cameras.emplace_back(Camera);
		}
		NewCameras.clear();
	}
	return GetCameras();
}

std::vector<Camera*> CameraManager::GetCameras()
{
	vector<Camera*> OutCams;
	OutCams.reserve(Cameras.size());
	for (auto &cam : Cameras)
	{
		OutCams.push_back(cam.get());
	}
	return OutCams;
}

void CameraManager::ThreadEntryPoint()
{
	cout << "Called base CameraManager ScanWorker, this is bad..." << endl;
	while (!killed)
	{
		this_thread::sleep_for(std::chrono::milliseconds(1000));
	}
}