#pragma once

#include <Communication/Transport/GenericTransport.hpp>

#include <thread>
#include <memory>
#include <shared_mutex>
#include <vector>
#include <functional>
#include <netinet/in.h>

//TCP transport layer



class TCPTransport : public GenericTransport
{
private:
	struct TCPConnection
	{
		int filedescriptor;
		sockaddr_in address;
		std::string name;
	};

	bool Server;
	std::string IP, Interface;
	int Port;
	int sockfd;
	bool Connected;
	std::unique_ptr<std::thread> ReceiveThreadHandle;
	mutable std::shared_mutex listenmutex; //protects connections
	std::vector<TCPConnection> connections;
public:

	TCPTransport(bool inServer, std::string inIP, int inPort, std::string inInterface);

	virtual ~TCPTransport();

private:
	void CreateSocket();
	bool Connect();
	void CheckConnection();
	void LowerLatency(int fd);
	void DeleteSocket(int fd);
	void ServerDeleteSocket(int clientidx);
public:

	virtual std::vector<std::string> GetClients() const override;

	virtual int Receive(void *buffer, int maxlength, std::string client, bool blocking=false) override;

	virtual bool Send(const void* buffer, int length, std::string client) override;

	std::vector<std::string> AcceptNewConnections();

	void DisconnectClient(std::string client);
	
	void receiveThread();
};
