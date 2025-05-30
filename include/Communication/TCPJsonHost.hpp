#pragma once

#include <memory>
#include <set>
#include <thread>
#include <atomic>
#include <optional>
#include <Transport/GenericTransport.hpp>

class TCPJsonHost
{
private:
	std::set<std::unique_ptr<std::thread>> ThreadHandles;
	int Port;
	bool killed;
	std::atomic<int> NumClients = 0;
public:
	class CDFRExternal* ExternalRunner = nullptr;
	class CDFRInternal* InternalRunner = nullptr;
	TCPJsonHost(int InPort);
	~TCPJsonHost();

	bool IsKilled() const
	{
		return killed;
	}

private:
	void ThreadEntryPoint(std::optional<GenericTransport::NetworkInterface> interface);
};