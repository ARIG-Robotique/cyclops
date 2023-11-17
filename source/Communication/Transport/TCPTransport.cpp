#include "Communication/Transport/TCPTransport.hpp"


#include <iostream>
#include <filesystem>
#include <sstream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/un.h>
#include <sys/types.h>
#include <unistd.h>
#include <arpa/inet.h>

#include <mutex>

using namespace std;

TCPTransport::TCPTransport(bool inServer, string inIP, int inPort, string inInterface)
	: GenericTransport()
{	
	Server = inServer;
	IP = inIP;
	Port = inPort;
	Interface = inInterface;
	sockfd = -1;
	Connected = false;
	CreateSocket();
	Connect();

	ReceiveThreadHandle = new thread(&TCPTransport::receiveThread, this);
}

TCPTransport::~TCPTransport()
{
	cout << "Destroying TCP transport " << IP << ":" << Port << " @ " << Interface <<endl;
	for (int i = 0; i < connections.size(); i++)
	{
		shutdown(connections[i].filedescriptor, SHUT_RDWR);
		close(connections[i].filedescriptor);
	}
	if (sockfd != -1)
	{
		shutdown(sockfd, SHUT_RDWR);
		close(sockfd);
	}
}

void TCPTransport::CreateSocket()
{
	if (sockfd != -1)
	{
		return;
	}
	int type = Server ? SOCK_STREAM : SOCK_STREAM | SOCK_NONBLOCK;
	sockfd = socket(AF_INET, type, 0);
	if (sockfd == -1)
	{
		cerr << "TCP Failed to create socket, port " << Port << endl;
	}
	
	if(setsockopt(sockfd, SOL_SOCKET, SO_BINDTODEVICE, Interface.c_str(), Interface.size()))
	{
		cerr << "TCP Failed to bind to interface : " << strerror(errno) << endl;
	}
	//LowerLatency(sockfd);
	
}

bool TCPTransport::Connect()
{
	struct sockaddr_in serverAddress;
	serverAddress.sin_family = AF_INET;
	serverAddress.sin_port = htons(Port);

	string ip = Server ? "0.0.0.0" : IP;

	if (inet_pton(AF_INET, ip.c_str(), &serverAddress.sin_addr) <= 0) {
		cerr << "TCP ERROR : Invalid address/ Address not supported \n" << endl;
	}

	if (Server)
	{
		//cout << "TCP Binding socket..." << endl;
		if (bind(sockfd, (struct sockaddr *)&serverAddress, sizeof(serverAddress)) == -1) 
		{
			cerr << "TCP Can't bind to IP/port, " << strerror(errno) << endl;
		}
		//cout << "TCP Marking socket for listening" << endl;
		if (listen(sockfd, SOMAXCONN) == -1)
		{
			cerr << "TCP Can't listen !" << endl;
		}
		Connected = true;
		return true;
	}
	else
	{
		// communicates with listen
		if(connect(sockfd, (struct sockaddr *)&serverAddress, sizeof(serverAddress)) == -1)
		{
			//cerr << "Failed to connect to server" << endl;
			return false;
		}
		else
		{
			cout << "TCP connected to server" << endl;
			Connected = true;
			return true;
		}
		
	}
}

void TCPTransport::CheckConnection()
{
	if (sockfd == -1)
	{
		CreateSocket();
	}
	if (!Connected)
	{
		Connect();
	}
}

void TCPTransport::LowerLatency(int fd)
{
	int corking = 0;
	if (setsockopt(fd, IPPROTO_TCP, TCP_CORK, &corking, sizeof(corking)))
	{
		cerr << "TCP Failed to disable corking : " << errno << endl;
	}
	int nodelay = 1;
	if (setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay)))
	{
		cerr << "TCP Failed to disable corking : " << errno << endl;
	}
}

void TCPTransport::DeleteSocket(int fd)
{
	close(fd);
}

void TCPTransport::ServerDeleteSocket(int clientidx)
{
	DeleteSocket(connections[clientidx].filedescriptor);
	connections.erase(next(connections.begin(), clientidx));
}


vector<string> TCPTransport::GetClients() const
{
	if (Server)
	{
		vector<string> clients;
		shared_lock lock(listenmutex);
		clients.reserve(connections.size());
		for (int i = 0; i < connections.size(); i++)
		{
			clients.push_back(connections[i].name);
		}
		return clients;
	}
	else
	{
		return {BroadcastClient};
	}
}

int TCPTransport::Receive(void *buffer, int maxlength, string client, bool blocking)
{
	int n = -1;
	//memset(buffer, '0' , maxlength);
	int flags = blocking && (client != GenericTransport::BroadcastClient) ? 0 : MSG_DONTWAIT;

	if (Server)
	{
		shared_lock lock(listenmutex);
		for (int i = 0; i < connections.size(); i++)
		{
			if (client != connections[i].name && client != BroadcastClient)
			{
				continue;
			}
			if ((n = recv(connections[i].filedescriptor, buffer, maxlength, flags)) > 0)
			{
				//cout << "Received " << n << " bytes..." << endl;
				//printBuffer(dataReceived, n);
				return n;
			}
		}
	}
	else
	{
		if ((n = recv(sockfd, buffer, maxlength, flags)) > 0)
		{
			//cout << "Received " << n << " bytes..." << endl;
			//printBuffer(dataReceived, n);
			return n;
		}
	}
	return -1;
}

bool TCPTransport::Send(const void* buffer, int length, string client)
{
	if (!Connected)
	{
		return false;
	}
	//cout << "Sending " << length << " bytes..." << endl;
	//printBuffer(buffer, length);
	/*if (length > 1000)
	{
		cerr << "WARNING : Packet length over 1000, packet may be dropped" << endl;
	}*/

	if (Server)
	{
		shared_lock lock(listenmutex);
		bool failed = false;
		for (int i = 0; i < connections.size(); i++)
		{
			if (client != connections[i].name && client != BroadcastClient)
			{
				continue;
			}
			int err = send(connections[i].filedescriptor, buffer, length, MSG_NOSIGNAL);
			int errnocp = errno;
			if (err ==-1 && (errnocp != EAGAIN && errnocp != EWOULDBLOCK))
			{
				if (errnocp == EPIPE)
				{
					DisconnectClient(connections[i].name);
					i--;
				}
				else
				{
					cerr << "TCP Server failed to send data to client " << connections[i].name << " : " << errnocp << " (" << strerror(errnocp) << ")" << endl;
					failed = true;
				}
				
			}
		}
		return !failed;
	}
	else
	{
		int err = send(sockfd, buffer, length, MSG_NOSIGNAL);
		int errnocp = errno;
		if (err ==-1 && (errnocp != EAGAIN && errnocp != EWOULDBLOCK))
		{
			if (errnocp == EPIPE)
			{
				cout << "TCP Server has disconnected" << endl;
				DeleteSocket(sockfd);
				sockfd = -1;
				Connected = false;
			}
			else
			{
				cerr << "TCP Failed to send data : " << errnocp << "(" << strerror(errnocp) << ")" << endl;
				return false;
			}
			
		}
		return true;
	}
}

vector<string> TCPTransport::AcceptNewConnections()
{
	if (!Server)
	{
		return {};	
	}
	vector<string> newconnections;
	CheckConnection();
	while (1)
	{
		TCPConnection connection;
		socklen_t clientSize = sizeof(connection.address);
		bzero(&connection.address, clientSize);
		connection.filedescriptor = accept4(sockfd, (struct sockaddr *)&connection.address, &clientSize, 0);
		if (connection.filedescriptor > 0)
		{
			//LowerLatency(ret);
			char buffer[16];
			inet_ntop(AF_INET, &connection.address.sin_addr, buffer, sizeof(buffer));
			buffer[sizeof(buffer)-1] = 0;
			connection.name = string(buffer, strlen(buffer));
			cout << "TCP Client connecting from " << connection.name << " fd=" << connection.filedescriptor << endl;
			unique_lock lock(listenmutex);
			connections.push_back(connection);
			newconnections.push_back(connection.name);
		}
		else 
		{
			switch (errno)
			{
			case EAGAIN:
				break;
			
			default:
				cerr << "TCP Unhandled error on accept: " << errno << "(" << strerror(errno) << ")" << endl;
				break;
			}
			break;
		}
	}
	return newconnections;
}

void TCPTransport::DisconnectClient(std::string client)
{
	unique_lock lock(listenmutex);
	for (int i = 0; i < connections.size(); i++)
	{
		if (client != connections[i].name && client != BroadcastClient)
		{
			continue;
		}
		cout << "TCP Client " << connections[i].name << " disconnected." <<endl;
		ServerDeleteSocket(i);
		i--;
	}
}

void TCPTransport::receiveThread()
{
	cout << "TCP Webserver thread started..." << endl;
	int n;
	while (1)
	{
		this_thread::sleep_for(chrono::milliseconds(100));
		CheckConnection();
		AcceptNewConnections();
	}
	
}