#include "Cameras/CameraManager.hpp"

#include <GlobalConf.hpp>
#include <Cameras/Calibfile.hpp>

#include <filesystem>


using namespace std;

bool CameraManager::DeviceInFilter(v4l2::devices::DEVICE_INFO device, std::string Filter)
{
	bool invertedFilter = false;
	if (Filter.length() > 1 && Filter[0] == '!')
	{
		invertedFilter = true;
		Filter = Filter.substr(1);
	}
	if (Filter.length() == 0)
	{
		return true;
	}
	
	return (device.device_description.find(Filter) != string::npos) ^ invertedFilter;
}

VideoCaptureCameraSettings CameraManager::DeviceToSettings(v4l2::devices::DEVICE_INFO device, CameraStartType Start)
{
	VideoCaptureCameraSettings settings;
	CaptureConfig cfg = GetCaptureConfig();
	settings.Resolution = cfg.FrameSize;
	settings.Framerate = cfg.CaptureFramerate;
	settings.FramerateDivider = cfg.FramerateDivider;
	settings.DeviceInfo = device;
	settings.StartType = Start;

	//these only get populated when StartFeed is called
	settings.StartPath = "";
	settings.ApiID = -1;

	string CalibrationRoot = string("../calibration/");
	string CalibrationPath = CalibrationRoot + settings.DeviceInfo.device_description;

	/*if (!filesystem::exists(CalibrationPath))
	{
		return settings;
	}*/
	

	readCameraParameters(CalibrationPath, settings.CameraMatrix, settings.distanceCoeffs, settings.Resolution);
	//cout << "Camera matrix : " << cam->CameraMatrix << " / Distance coeffs : " << cam->distanceCoeffs << endl;
	return settings;
}

vector<VideoCaptureCameraSettings> CameraManager::autoDetectCameras(CameraStartType Start, string Filter, string CalibrationFile, bool silent)
{
	vector<v4l2::devices::DEVICE_INFO> devices;

	v4l2::devices::list(devices);

	if (!silent)
	{
		for (const auto & device : devices) 
		{
			cout << device.device_description <<  " at " << device.bus_info << " is attached to :\n";
			for (const auto & path : device.device_paths) {
				cout << "\t" << path << "\n";
			}
		}
	}
	vector<VideoCaptureCameraSettings> detected;
	for (const auto & device : devices)
	{
		//v4l2-ctl --list-formats-ext
		//gst-launch-1.0 v4l2src device="/dev/video0" io-mode=2 ! "image/jpeg, width=1920, height=1080, framerate=30/1" ! nvjpegdec ! "video/x-raw" ! nvoverlaysink -e
		//gst-launch-1.0 v4l2src device="/dev/video0" ! "video/x-h264, format=H264, width=1920, height=1080, framerate=30/1" ! h264parse ! omxh264dec ! nvvidconv ! "video/x-raw(memory:NVMM), format=NV12" ! nvoverlaysink -e
		
		//gst-launch-1.0 v4l2src device="/dev/video0" io-mode=2 ! "image/jpeg, width=1920, height=1080, framerate=60/1" ! nvdec ! glimagesink -e
		
		//nvv4l2decoder ?
		//string capname = string("v4l2src device=/dev/video") + to_string(i)
		//+ String(" io-mode=2 do-timestamp=true ! image/jpeg, width=1920, height=1080, framerate=30/2 ! nvv4l2decoder mjpeg=1 ! nvvidconv ! video/x-raw,format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink");
		
		//jpegdec + videoconvert marche
		//nvdec ! glcolorconvert ! gldownload
		if (DeviceInFilter(device, Filter))
		{
			detected.push_back(DeviceToSettings(device, Start));
		}
	}
	return detected;
}


vector<Camera*> CameraManager::Tick()
{
	for (int i = 0; i < Cameras.size(); i++)
	{
		if (Cameras[i]->errors >= 20)
		{
			std::cerr << "Detaching camera @ " << Cameras[i]->GetName() << std::endl;
			std::string pathtofind = dynamic_cast<const VideoCaptureCameraSettings*>(Cameras[i]->GetCameraSettings())->DeviceInfo.device_paths[0];
			StopCamera(Cameras[i]);
			
			unique_lock lock(pathmutex);
			usedpaths.erase(pathtofind);
			delete Cameras[i];
			Cameras.erase(std::next(Cameras.begin(), i));
			i--;
		}
	}
	{
		unique_lock lock(cammutex);
		for (auto Camera : NewCameras)
		{
			RegisterCamera(Camera);
			Cameras.push_back(Camera);
		}
		NewCameras.clear();
	}
	return Cameras;
}

void CameraManager::StartScanThread()
{
	if (scanthread)
	{
		return;
	}
	scanthread = new thread(&CameraManager::ScanWorker, this);
}

void CameraManager::ScanWorker()
{
	while (!killmutex)
	{
		std::vector<v4l2::devices::DEVICE_INFO> devices;
		v4l2::devices::list(devices);
		std::set<std::string> knownpaths, unknownpaths;
		{
			shared_lock lock(pathmutex);
			std::copy(usedpaths.begin(), usedpaths.end(), std::inserter(knownpaths, knownpaths.end()));
			std::copy(blockedpaths.begin(), blockedpaths.end(), std::inserter(knownpaths, knownpaths.end()));
		}

		for (auto &device : devices)
		{
			std::string pathtofind = device.device_paths[0];
			auto pos = std::find(knownpaths.begin(), knownpaths.end(), pathtofind);
			if (pos == knownpaths.end()) //new camera
			{
				continue;
			}

			VideoCaptureCameraSettings settings = DeviceToSettings(device, Start);
			if (!settings.IsValid()) //no valid settings
			{
				std::cerr << "Failed to open camera " << device.device_description << " @ " << pathtofind << " : Invalid settings" << std::endl;
				unique_lock lock(pathmutex);
				blockedpaths.emplace(pathtofind);
				continue;
			}

			bool HasCalib = settings.IsValidCalibration();
			if (!AllowNoCalib && !HasCalib)
			{
				std::cerr << "Did not open camera " << device.device_description << " @ " << pathtofind << " : Camera has no calibration" << std::endl;
				unique_lock lock(pathmutex);
				blockedpaths.emplace(pathtofind);
				continue;
			}

			Camera* cam = StartCamera(settings);
			if (!cam)
			{
				std::cerr << "Did not open camera " << device.device_description << " @ " << pathtofind << " : StartCamera returned null" << std::endl;
				unique_lock lock(pathmutex);
				blockedpaths.emplace(pathtofind);
				continue;
			}

			{
				{
					unique_lock lock(pathmutex);
					usedpaths.emplace(pathtofind);
				}
				{
					unique_lock lock(cammutex);
					NewCameras.push_back(cam);
				}
				
			}
		}

		this_thread::sleep_for(std::chrono::milliseconds(1000));
	}
}