#pragma once

#include <string>
#include <thread>
#include <vector>
#include <cstdint>
#include <sstream>

class TCPTransport;

//Get Cameras
//Get image from camera #
//	-jpeg, base64
//Send image to be processed (2D, 3D, yolo)

//Get external data (with masking)
//	-full data (matrix)
//	-2D simplified
//	-stated

class JsonListener
{
private:
	std::thread *ListenThread = nullptr;
	bool killed = false;
	std::vector<char> ReceiveBuffer;
public:
	TCPTransport *Transport;
	std::string ClientName;

	JsonListener(TCPTransport* InTransport, std::string InClientName);

	~JsonListener();

	bool IsKilled() const
	{
		return killed;
	}

private:

	void HandleJson(const std::string &command);

	void ThreadEntryPoint();
};
