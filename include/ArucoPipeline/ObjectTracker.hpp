#pragma once

#include <opencv2/core.hpp>
#include <ArucoPipeline/TrackedObject.hpp>
#include <Misc/ArucoDictSize.hpp>
#include <array>

//Class that handles the objects, and holds information about each tag's size
//Registered objects will have their locations solved and turned into a vector of ObjectData for display and data sending
class ObjectTracker
{
private:
	std::vector<TrackedObject*> objects;
	std::array<int, ARUCO_DICT_SIZE> ArucoMap; //Which object owns the tag at index i ? objects[ArucoMap[TagID]]
	std::array<float, ARUCO_DICT_SIZE> ArucoSizes; //Size of the aruco tag

public:
	ObjectTracker(/* args */);
	~ObjectTracker();

	void RegisterTrackedObject(TrackedObject* object);

	void UnregisterTrackedObject(TrackedObject* object);

	bool SolveCameraLocation(CameraFeatureData& CameraData);

	void SolveLocationsPerObject(std::vector<CameraFeatureData>& CameraData, uint64_t Tick);


	std::vector<ObjectData> GetObjectDataVector(uint64_t Tick);

	//only needed for the center
	void SetArucoSize(int number, float SideLength);

	float GetArucoSize(int number);

private:

	void RegisterArucoRecursive(TrackedObject* object, int index);
};