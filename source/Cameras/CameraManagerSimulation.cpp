#include <Cameras/CameraManagerSimulation.hpp>

#include <filesystem>
#include <fstream>
#include <nlohmann/json.hpp>
#include <Cameras/Calibfile.hpp>
#include <Misc/GlobalConf.hpp>

using namespace std;

void CameraManagerSimulation::ThreadEntryPoint()
{
	while (!killed)
	{
		{
			shared_lock lock(pathmutex);
			if (usedpaths.size() != 0)
			{
				this_thread::sleep_for(std::chrono::milliseconds(1000));
				continue;
			}
		}
		if (!filesystem::exists(ScenarioPath))
		{
			cerr << "Scenario could not be found ! Path " << ScenarioPath << endl;
			break;
		}
		
		auto rootPath = filesystem::weakly_canonical(ScenarioPath).parent_path();
		
		ifstream scenario(ScenarioPath);
		nlohmann::json decoded;
		try
		{
			scenario >> decoded;
		}
		catch(const std::exception& e)
		{
			std::cerr << e.what() << '\n';
			break;
		}
		if (!decoded.is_array())
		{
			cerr << "Scenario root is not array !" << endl;
			break;
		}
		
		for (auto &i : decoded.items())
		{
			filesystem::path calibpath, videopath;
			try
			{
				auto value = i.value();
				calibpath = rootPath / value["calibration"];
				videopath = rootPath / value["video"];
			}
			catch(const std::exception& e)
			{
				std::cerr << e.what() << '\n';
				continue;
			}
			
			VideoCaptureCameraSettings settings;
			readCameraParameters(calibpath, settings.CameraMatrix, settings.distanceCoeffs, settings.Resolution);
			settings.StartType = CameraStartType::PLAYBACK;
			settings.StartPath = videopath;
			settings.record = RecordVideo;
			settings.DeviceInfo.device_paths.push_back(videopath);
			settings.DeviceInfo.device_paths.push_back(calibpath);
			settings.DeviceInfo.device_description = videopath.filename().replace_extension("");
			auto cam = StartCamera(settings);
			if (!cam)
			{
				std::cerr << "Did not open virtual camera " << videopath << " @ " << calibpath << " : StartCamera returned null" << std::endl;
				unique_lock lock(pathmutex);
				continue;
			}
			{
				{
					unique_lock lock(pathmutex);
					usedpaths.emplace(videopath);
				}
				{
					unique_lock lock(cammutex);
					NewCameras.emplace_back(cam);
				}
			}
		}
	}
}