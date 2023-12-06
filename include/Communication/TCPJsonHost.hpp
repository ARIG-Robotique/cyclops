#pragma once

#include <memory>
#include <set>
#include <thread>
#include <Communication/Transport/GenericTransport.hpp>

class TCPJsonHost
{
private:
	std::set<std::thread*> ThreadHandles;
	int Port;
	bool killed;
public:
	class CDFRExternal* ExternalRunner;
	TCPJsonHost(int InPort);
	~TCPJsonHost();

	bool IsKilled() const
	{
		return killed;
	}

private:
	void ThreadEntryPoint(GenericTransport::NetworkInterface interface);
};