#pragma once

#include <utility>
#include <vector>
#include <string>

//Generic class to send data to other programs
class GenericTransport
{
private:
	/* data */
public:

	virtual ~GenericTransport() {};

	static const std::string BroadcastClient;

	static void printBuffer(const void *buffer, int length);

	virtual std::vector<std::string> GetClients() const;

	//receive from clients
	//return number of bytes read
	virtual int Receive(void *buffer, int maxlength, std::string client, bool blocking=false);

	virtual bool Send(const void* buffer, int length, std::string client);

	struct NetworkInterface
	{
		std::string name;
		std::string address;
		std::string mask;
		std::string broadcast;
	};

	static std::vector<NetworkInterface> GetInterfaces();
};
