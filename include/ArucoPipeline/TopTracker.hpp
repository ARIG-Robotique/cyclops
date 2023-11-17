#pragma once

#include <ArucoPipeline/TrackedObject.hpp>

//A cube with 4 tags, one on each side
class TopTracker : public TrackedObject
{
private:

public:
	TopTracker(int MarkerIdx, double MarkerSize, std::string InName);
	~TopTracker();

	virtual std::vector<ObjectData> ToObjectData() const override;
};