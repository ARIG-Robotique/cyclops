#include <Cameras/CameraManagerSimulation.hpp>

#include <filesystem>
#include <fstream>
#include <regex>
#include <nlohmann/json.hpp>
#include <Cameras/Calibfile.hpp>
#include <Misc/path.hpp>
#include <Misc/GlobalConf.hpp>
#include <Transport/thread-rename.hpp>
#include <opencv2/imgcodecs.hpp>

using namespace std;

int extractNumber(const std::string& filename) {
    std::regex numberRegex(R"(\d+)"); // Regular expression to match digits
    std::smatch match;

    if (std::regex_search(filename, match, numberRegex)) {
        return std::stoi(match.str()); // Return the first number found as an integer
    }
    return -1; // Return -1 if no number is found
}

void TurnFolderToVideo(std::filesystem::path folderPath)
{
	std::string outputVideo = folderPath; // Output video file
	outputVideo.append(".avi");
	int frameRate = 24; // Frames per second for the video

	try
	{
		std::filesystem::remove(outputVideo);
	}
	catch(const std::exception& e)
	{
		std::cerr << e.what() << '\n';
	}

	// Store image paths
	std::vector<std::filesystem::path> imagePaths;

	// Read all image files from the folder
	for (auto& entry : std::filesystem::directory_iterator(folderPath)) {
		if (entry.is_regular_file()) {
			imagePaths.push_back(entry.path());
		}
	}

	// Sort the image paths alphabetically to ensure correct order
	std::sort(imagePaths.begin(), imagePaths.end(), 
		[](const std::filesystem::path& a, const std::filesystem::path& b) 
		{
        	return extractNumber(a.filename().string()) < extractNumber(b.filename().string());
		}
	);

	if (imagePaths.empty()) {
		std::cerr << "No valid image files found in the folder." << std::endl;
		return;
	}

	// Read the first image to determine the frame size
	cv::Mat firstFrame = cv::imread(imagePaths[0]);
	if (firstFrame.empty()) {
		std::cerr << "Failed to read the first image." << std::endl;
		return;
	}

	cv::Size frameSize = firstFrame.size();
	int codec = cv::VideoWriter::fourcc('H', '2', '6', '4'); // Codec for AVI format

	// Initialize the video writer
	cv::VideoWriter videoWriter(outputVideo, codec, frameRate, frameSize);

	if (!videoWriter.isOpened()) {
		std::cerr << "Could not open the video file for writing." << std::endl;
		return;
	}

	// Process and write each image to the video
	for (const auto& imagePath : imagePaths) {
		cv::Mat frame = cv::imread(imagePath);
		if (frame.empty()) {
			std::cerr << "Skipping invalid image: " << imagePath << std::endl;
			continue;
		}

		// Resize frame to match the first image's size, if needed
		if (frame.size() != frameSize) {
			cv::resize(frame, frame, frameSize);
		}

		// Write the frame to the video
		videoWriter.write(frame);
	}
	
	try
	{
		std::filesystem::remove_all(folderPath);
	}
	catch(const std::exception& e)
	{
		std::cerr << e.what() << '\n';
	}
	
	
}

void CameraManagerSimulation::ThreadEntryPoint()
{
	SetThreadName("CameraManagerSimulation");
	//number of times the cameras have been restarted. Limited to 1 when recording the screens
	int NumberOfInvocation = 0;
	while (!killed)
	{
		if (Idle)
		{
			this_thread::sleep_for(chrono::milliseconds(1000));
			continue;
		}
		
		{
			shared_lock lock(pathmutex);
			if (usedpaths.size() != 0)
			{
				this_thread::sleep_for(std::chrono::milliseconds(1000));
				continue;
			}
			if (NumberOfInvocation > 0 && DoScreenCapture())
			{
				for (auto &&folder : std::filesystem::directory_iterator(GetScreenCapturePath()))
				{
					if (folder.is_directory())
					{
						TurnFolderToVideo(folder.path());
					}
				}
				exit(0);
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
			vector<unsigned int> CameraLockToggles;
			try
			{
				auto value = i.value();
				calibpath = rootPath / value["calibration"];
				videopath = rootPath / value["video"];
				if (value.contains("locks") && value["locks"].is_array())
				{
					for (auto &&i : value["locks"])
					{
						if (i.is_number_integer())
						{
							CameraLockToggles.push_back(i);
						}
					}
				}
			}
			catch(const std::exception& e)
			{
				std::cerr << e.what() << '\n';
				continue;
			}
			
			VideoCaptureCameraSettings settings;
			readCameraParameters(calibpath, settings);
			settings.StartType = CameraStartType::PLAYBACK;
			settings.StartPath = videopath;
			settings.DeviceInfo.device_paths.push_back(videopath);
			settings.DeviceInfo.device_paths.push_back(calibpath);
			settings.DeviceInfo.device_description = videopath.filename().replace_extension("");
			settings.CameraLockToggles = CameraLockToggles;
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
		NumberOfInvocation++;
	}
}