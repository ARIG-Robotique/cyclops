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
	std::stringstream ReceiveBuffer;
public:
	TCPTransport *Transport;
	std::string ClientName;

	void StartListenThread();

	~JsonListener();

private:
	void ThreadEntryPoint();
};
