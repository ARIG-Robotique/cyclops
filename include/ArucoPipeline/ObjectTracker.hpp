#pragma once

#include <opencv2/core.hpp>
#include <ArucoPipeline/TrackedObject.hpp>
#include <ArucoPipeline/ArucoTypes.hpp>
#include <array>

struct ResolvedLocation
{
	float score;
	cv::Affine3d WorldToObject;
	cv::Affine3d WorldToCamera;

	ResolvedLocation(float InScore, cv::Affine3d InObjLoc, cv::Affine3d InCamLoc)
	:score(InScore), WorldToObject(InObjLoc), WorldToCamera(InCamLoc)
	{}

	bool operator<(ResolvedLocation& other)
	{
		return score < other.score;
	}

	static cv::Affine3d IntersectMultiview(std::vector<ResolvedLocation> Views);
};

//Class that handles the objects, and holds information about each tag's size
//Registered objects will have their locations solved and turned into a vector of ObjectData for display and data sending
class ObjectTracker
{
private:
	std::vector<std::shared_ptr<TrackedObject>> objects;
	std::array<int, ARUCO_DICT_SIZE> ArucoMap; //Which object owns the tag at index i ? objects[ArucoMap[TagID]]
	std::array<double, ARUCO_DICT_SIZE> ArucoSizes; //Size of the aruco tag

public:
	ObjectTracker(/* args */);
	~ObjectTracker();

	void RegisterTrackedObject(std::shared_ptr<TrackedObject> object);

	void UnregisterTrackedObject(std::shared_ptr<TrackedObject> object);

	bool SolveCameraLocation(CameraFeatureData& CameraData);

	void SolveLocationsPerObject(std::vector<CameraFeatureData>& CameraData, TrackedObject::TimePoint Tick);


	std::vector<ObjectData> GetObjectDataVector(TrackedObject::TimePoint Tick);

	//only needed for the center
	void SetArucoSize(int number, double SideLength);

	double GetArucoSize(int number);

	std::vector<std::vector<cv::Point3d>> GetPointsOfInterest() const;

private:

	void RegisterArucoRecursive(std::shared_ptr<TrackedObject> object, int index);
};