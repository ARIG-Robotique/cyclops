#pragma once

#include <string>
#include <thread>
#include <vector>
#include <cstdint>
#include <sstream>
#include <chrono>
#include <opencv2/core/affine.hpp>
#include <nlohmann/json.hpp>

class TCPTransport;

//Get Cameras
//Get image from camera #
//	-jpeg, base64
//Send image to be processed (2D, 3D, yolo)

//Get external data (with masking)
//	-full data (matrix)
//	-2D simplified
//	-stated

class TCPJsonHost;

class JsonListener
{
private:
	std::thread *ListenThread = nullptr;
	bool killed = false;
	std::vector<char> ReceiveBuffer;
public:
	TCPTransport *Transport = nullptr;
	std::string ClientName = "none";
	TCPJsonHost *Parent = nullptr;
	uint64 sendIndex = 0;
	std::chrono::steady_clock::time_point LastAliveSent, LastAliveReceived;
	enum class TransformMode
	{
		Float2D,
		Millimeter2D,
		Float3D
	};
	TransformMode ObjectMode = TransformMode::Millimeter2D;


	JsonListener(TCPTransport* InTransport, std::string InClientName, class TCPJsonHost* InParent);

	~JsonListener();

	bool IsKilled() const
	{
		return killed;
	}

private:

	nlohmann::json ObjectToJson(const struct ObjectData& Object);

	//Get data from external monitor
	nlohmann::json GetData(nlohmann::json filter);

	void HandleQuery(const nlohmann::json &Query);

	void HandleResponse(const nlohmann::json &Response);

	bool IsQuery(const nlohmann::json &object);

	void HandleJson(const std::string &command);

	void SendJson(const nlohmann::json &object);

	void CheckAlive();

	void ThreadEntryPoint();
};
