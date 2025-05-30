#pragma once

#include <array>
#include <opencv2/core.hpp>				// Basic OpenCV structures (Mat, Scalar)
#include <opencv2/video/tracking.hpp>	//Kalman filter
#include <ArucoPipeline/ObjectIdentity.hpp>
#include <Communication/ProcessedTypes.hpp>

class Camera;
class BoardViz2D;
struct CameraFeatureData;


struct ArucoMarker
{
public:
	double sideLength; //Length of a side of the tags. Tag should be square
	int number; //Number of the tag on it
	cv::Affine3d Pose; //Location relative to it's parent

private:
	mutable std::array<cv::Point3d, 4> ObjectPointsNoOffset;

public:
	static std::array<cv::Point3d, 4> GetObjectPointsNoOffset(double SideLength);

	const std::array<cv::Point3d, 4>& GetObjectPointsNoOffset() const;

	ArucoMarker()
		:sideLength(0.05),
		number(-1),
		Pose(cv::Affine3d::Identity())
	{}

	ArucoMarker(double InSideLength, int InNumber)
		:sideLength(InSideLength),
		number(InNumber),
		Pose(cv::Affine3d::Identity()),
		ObjectPointsNoOffset(GetObjectPointsNoOffset(InSideLength))
	{}

	ArucoMarker(double InSideLength, int InNumber, cv::Affine3d InPose)
		:sideLength(InSideLength),
		number(InNumber),
		Pose(InPose),
		ObjectPointsNoOffset(GetObjectPointsNoOffset(InSideLength))
	{}
};

//Base class for any object that should be tracked in 3D space.
//Can hold other object and/or Aruco tags
//Aruco tags will be registered to this object when added to the object tracker
class TrackedObject
{
public: 
	typedef ObjectData::Clock Clock;
	typedef ObjectData::TimePoint TimePoint;
	struct ArucoViewCameraLocal
	{
		cv::Affine3d AccumulatedTransform; //transform to marker, not including the marker's transform relative to it's parent
		ArucoMarker* Marker; //pointer to source marker
		ArucoCornerArray CameraCornerPositions; //corner positions, in space relative to the calling object's coordinates
		std::vector<cv::Point3d> StereoMarkerCorners; 
		std::vector<cv::Point3d> LocalMarkerCorners; //corner positions in space relative to camera
		int IndexInCameraData; //index where this marker was found in the camera
		int LensIndex;

		bool IsStereo()
		{
			return StereoMarkerCorners.size() > 0;
		}

		bool IsMono()
		{
			return CameraCornerPositions.size() >0;
		}

		float GetSurface();
		float GetVolume();
		cv::Affine3d FitPlane();
	};
public:
	std::vector<ArucoMarker> markers; //Should be populated before adding to the Object Tracker
	std::vector<std::shared_ptr<TrackedObject>> childs; //Should be populated before adding to the Object Tracker
	bool Unique; //Can there be only one ?
	bool CoplanarTags; //Are all tags on the same plane ? If true, then it uses IPPE solve when multiple tags are located
	cv::String Name; //Display name

protected:
	cv::Affine3d Location;
	TimePoint LastSeenTick;
	cv::KalmanFilter LocationFilter;

public:

	TrackedObject();

	//Set location. Bypass kalman filter if tick is UINT64_MAX
	virtual bool SetLocation(cv::Affine3d InLocation, TimePoint Tick);
	TimePoint GetLastSeenTick() const { return LastSeenTick; }

	virtual bool ShouldBeDisplayed(TimePoint Tick) const;
	virtual cv::Affine3d GetLocation() const;

	//Find the parameters and the accumulated transform of the tag in the component and it's childs
	virtual bool FindTag(int MarkerID, ArucoMarker& Marker, cv::Affine3d& TransformToMarker);

	//Returns all the corners in 3D space of this object and it's childs, with the marker ID. Does not clear the array at start.
	virtual void GetObjectPoints(std::vector<std::vector<cv::Point3d>>& MarkerCorners, std::vector<int>& MarkerIDs, 
		cv::Affine3d rootTransform = cv::Affine3d::Identity(), std::vector<int> filter = {});

	//Per-lens functions

	//Returns the surface area, markers that are seen by the lens that belong to this object or it's childs are stored in MarkersSeen
	float GetSeenMarkers2D(const LensFeatureData& LensData, std::vector<ArucoViewCameraLocal> &MarkersSeen, 
		cv::Affine3d AccumulatedTransform = cv::Affine3d::Identity(), bool Skip3D = false, int LensIndex = -1);
	//Reprojects some seen markers, returns the reprojection error
	float ReprojectSeenMarkers(const std::vector<ArucoViewCameraLocal> &MarkersSeen, const cv::Affine3d &LensToMarker, 
		const LensFeatureData &LensData, std::map<std::pair<int, int>, ArucoCornerArray> &ReprojectedCorners, int LensIndex = -1);
	//Given corners, solve this object location using multiple tags at once
	//Output transform is given relative to the lens
	//Does not do any reprojection
	virtual cv::Affine3d GetObjectTransform(const LensFeatureData& LensData, float& Surface, int LensIndex = -1);
	
	//Per camera functions

	//Calls GetSeenMarkers2D on all lenses
	float GetSeenMarkers2D(const CameraFeatureData& CameraData, std::vector<ArucoViewCameraLocal> &MarkersSeen, 
		cv::Affine3d AccumulatedTransform = cv::Affine3d::Identity(), bool Skip3D = false);

	float GetSeenMarkers3D(const CameraFeatureData& CameraData, std::vector<ArucoViewCameraLocal> &MarkersSeen, 
		cv::Affine3d AccumulatedTransform = cv::Affine3d::Identity());

	//Calls ReprojectSeenMarkers on all lenses
	float ReprojectSeenMarkers(const std::vector<ArucoViewCameraLocal> &MarkersSeen, const cv::Affine3d &CameraToMarker, 
		const CameraFeatureData &CameraData, std::map<std::pair<int, int>, ArucoCornerArray> &ReprojectedCorners);

	//Calls GetObjectTransform on all lenses, then merges the data and then reprojects
	virtual cv::Affine3d GetObjectTransform(const CameraFeatureData& CameraData, float& Surface, float& ReprojectionError, 
		std::map<std::pair<int, int>, ArucoCornerArray> &ReprojectedCorners);

	virtual std::vector<ObjectData> GetMarkersAndChilds() const;

	virtual std::vector<ObjectData> ToObjectData() const;

	//Array of world space coordinate points where extra attention should be given using aruco (second pass detection using a smaller window containing those points)
	virtual std::vector<std::vector<cv::Point3d>> GetPointsOfInterest() const;

	void Inspect();
};