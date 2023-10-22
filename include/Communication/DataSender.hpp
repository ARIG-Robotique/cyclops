#pragma once

#include <vector>
#include <fstream>
#include <thread>
#include <mutex>
#include "ArucoPipeline/TrackedObject.hpp"
#include "Communication/Encoders/GenericEncoder.hpp"
#include "Communication/Transport/GenericTransport.hpp"

//Class that chains an encoder and a transport layer.
class PositionDataSender
{
protected:

	int64 StartTick;

	std::thread *ReceiveThread;
	std::mutex ReceiveKillMutex;

public:
	GenericEncoder *encoder;
	GenericTransport *transport; 


	PositionDataSender();

	virtual ~PositionDataSender();

	int64 GetTick();

	virtual void SendPacket(int64 GrabTick, std::vector<ObjectData> &Data);

	void ThreadRoutine();

	void StartReceiveThread();
	void StopReceiveThread();
};