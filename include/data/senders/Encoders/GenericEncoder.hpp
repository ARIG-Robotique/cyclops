#pragma once

#include <vector>
#include <cstdint>

#include "ArucoPipeline/ObjectIdentity.hpp"
#include "ArucoPipeline/TrackedObject.hpp"

struct EncodedData
{
	bool valid;
	size_t length;
	char* buffer;

	EncodedData()
	:valid(false), length(0), buffer(nullptr)
	{}

	EncodedData(size_t inlength, char* inbuffer)
	:valid(true), length(inlength), buffer(inbuffer)
	{}
};


//Base encoder class : The encoders transform the object data into bytes that can be sent to other clients though transports, following strict rules
//They should live at least as long as the connection, as they keep their own time.
//They also filter what type of object to send on the network
class GenericEncoder
{
	int64 CreationTick;
public:
    GenericEncoder();
	virtual ~GenericEncoder(){}
    int64 GetTick();
	virtual EncodedData Encode(int64 GrabTime, const std::vector<ObjectData> &objects) = 0;
};
