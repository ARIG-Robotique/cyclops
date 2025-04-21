#pragma once

#include <memory>
#include <thread>
#include <Transport/UDPTransport.hpp>

class AdvertiseMV
{
private:
	bool killmutex = false;
	std::vector<std::unique_ptr<std::thread>> threads;
	void ThreadStartPoint(GenericTransport::NetworkInterface interface);
public:
	AdvertiseMV();
	~AdvertiseMV();
};