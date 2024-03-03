#include "ArucoPipeline/ObjectTracker.hpp"


#include <vector>

#include <math3d.hpp>
#include <ArucoPipeline/StaticObject.hpp>

using namespace cv;
using namespace std;

ObjectTracker::ObjectTracker(/* args */)
{
	assert(ArucoMap.size() == ArucoSizes.size());
	for (size_t i = 0; i < ArucoMap.size(); i++)
	{
		ArucoMap[i] = -1;
		ArucoSizes[i] = 0.05;
	}
}

ObjectTracker::~ObjectTracker()
{
}

void ObjectTracker::RegisterTrackedObject(shared_ptr<TrackedObject> object)
{
	int index = objects.size();
	objects.push_back(object);
	RegisterArucoRecursive(object, index);
}

void ObjectTracker::UnregisterTrackedObject(shared_ptr<TrackedObject> object)
{
	assert(object->markers.size() == 0 && object->childs.size() == 0);
	auto objpos = find(objects.begin(), objects.end(), object);
	if (objpos != objects.end())
	{
		objects.erase(objpos);
	}
	
}

struct ResolvedLocation
{
	float score;
	Affine3d AbsLoc;
	Affine3d CameraLoc;

	ResolvedLocation(float InScore, Affine3d InObjLoc, Affine3d InCamLoc)
	:score(InScore), AbsLoc(InObjLoc), CameraLoc(InCamLoc)
	{}

	bool operator<(ResolvedLocation& other)
	{
		return score < other.score;
	}
};

bool ObjectTracker::SolveCameraLocation(CameraFeatureData& CameraData)
{
	CameraData.CameraTransform = Affine3d::Identity();
	float score = 0;
	map<int, ArucoCornerArray> ReprojectedCorners; //index in array, corners
	for (auto object : objects)
	{
		auto *staticobj = dynamic_cast<StaticObject*>(object.get());
		if (staticobj == nullptr)
		{
			continue;
		}
		if (staticobj->IsRelative())
		{
			cerr << "SolveCameraLocation isn't meant for inside-out tracking !" << endl;
			assert(0);
		}
		float surface, reprojectionError;
		Affine3d NewTransform = staticobj->GetObjectTransform(CameraData, surface, reprojectionError, ReprojectedCorners);
		float newscore = surface;
		if (newscore <= score)
		{
			continue;
		}
		CameraData.CameraTransform = NewTransform.inv();
		score = newscore;
	}

	for (auto it = ReprojectedCorners.begin(); it != ReprojectedCorners.end(); it++)
	{
		CameraData.ArucoCornersReprojected[it->first] = it->second;
	}
	return score >0;
}

void ObjectTracker::SolveLocationsPerObject(vector<CameraFeatureData>& CameraData, uint64_t Tick)
{
	const int NumCameras = CameraData.size();
	//const int NumObjects = objects.size();
	vector<map<int, ArucoCornerArray>> ReprojectedCorners;
	ReprojectedCorners.resize(NumCameras);
	
	/*parallel_for_(Range(0, objects.size()), [&](const Range& range)
	{*/
		Range range(0, objects.size());
		for(int ObjIdx = range.start; ObjIdx < range.end; ObjIdx++)
		{
			auto object = objects[ObjIdx];
			if (object->markers.size() == 0)
			{
				continue;
			}
			StaticObject* objstat = dynamic_cast<StaticObject*>(object.get());
			if (objstat != nullptr)
			{
				if (!objstat->IsRelative()) //Do not solve for static non-relative objects
				{
					continue;
				}
			}
			
			vector<ResolvedLocation> locations;
			for (size_t CameraIdx = 0; CameraIdx < CameraData.size(); CameraIdx++)
			{
				CameraFeatureData& ThisCameraData = CameraData[CameraIdx];
				if (ThisCameraData.ArucoCorners.size() == 0) //Not seen
				{
					continue;
				}
				float AreaThis, ReprojectionErrorThis;
				Affine3d transformProposed = ThisCameraData.CameraTransform * 
					objects[ObjIdx]->GetObjectTransform(ThisCameraData, AreaThis, ReprojectionErrorThis, ReprojectedCorners[CameraIdx]);
				float ScoreThis = AreaThis/(ReprojectionErrorThis + 0.1);
				if (ScoreThis < 1 || ReprojectionErrorThis == INFINITY) //Bad solve or not seen
				{
					continue;
				}
				locations.emplace_back(ScoreThis, transformProposed, ThisCameraData.CameraTransform);
			}
			if (locations.size() == 0)
			{
				continue;
			}
			if (locations.size() == 1)
			{
				object->SetLocation(locations[0].AbsLoc, Tick);
				//cout << "Object " << object->Name << " is at location " << objects[ObjIdx]->GetLocation().translation() << " / score: " << locations[0].score << ", seen by 1 camera" << endl;
				continue;
			}
			std::sort(locations.begin(), locations.end());
			ResolvedLocation &best = locations[locations.size()-1];
			ResolvedLocation &secondbest = locations[locations.size()-2];
			Vec3d l1p = best.AbsLoc.translation();
			Vec3d l2p = secondbest.AbsLoc.translation();
			Vec3d l1d = NormaliseVector(best.CameraLoc.translation() - l1p);
			Vec3d l2d = NormaliseVector(secondbest.CameraLoc.translation() - l2p);
			Vec3d l1i, l2i;
			ClosestPointsOnTwoLine(l1p, l1d, l2p, l2d, l1i, l2i);
			Vec3d locfinal = (l1i*best.score+l2i*secondbest.score)/(best.score + secondbest.score);
			Affine3d combinedloc = best.AbsLoc;
			combinedloc.translation(locfinal);
			object->SetLocation(combinedloc, Tick);
			//cout << "Object " << object->Name << " is at location " << objects[ObjIdx]->GetLocation().translation() << " / score: " << best.score+secondbest.score << ", seen by " << locations.size() << " cameras" << endl;
		}
	//});

	for (int CamIdx = 0; CamIdx < NumCameras; CamIdx++)
	{
		for (auto it = ReprojectedCorners[CamIdx].begin(); it != ReprojectedCorners[CamIdx].end(); it++)
		{
			CameraData[CamIdx].ArucoCornersReprojected[it->first] = it->second;
		}
	}
}

vector<ObjectData> ObjectTracker::GetObjectDataVector(uint64_t Tick)
{
	vector<ObjectData> ObjectDatas;
	ObjectDatas.reserve(objects.size()*2);

	for (size_t i = 0; i < objects.size(); i++)
	{
		if (Tick != 0 && !objects[i]->ShouldBeDisplayed(Tick)) //not seen, do not display
		{
			continue;
		}
		
		vector<ObjectData> lp = objects[i]->ToObjectData();
		for (size_t j = 0; j < lp.size(); j++)
		{
			ObjectDatas.push_back(lp[j]);
		}
	}
	return ObjectDatas;
}

void ObjectTracker::SetArucoSize(int number, double SideLength)
{
	ArucoSizes[number] = SideLength;
}

double ObjectTracker::GetArucoSize(int number)
{
	return ArucoSizes[number];
}

vector<vector<Point3d>> ObjectTracker::GetPointsOfInterest() const
{
	vector<vector<Point3d>> poi;
	for (auto object : objects)
	{
		auto localpoi = object->GetPointsOfInterest();
		for (auto &&i : localpoi)
		{
			poi.push_back(i);
		}
	}
	return poi;
}

void ObjectTracker::RegisterArucoRecursive(shared_ptr<TrackedObject> object, int index)
{
	for (size_t i = 0; i < object->markers.size(); i++)
	{
		const ArucoMarker& marker = object->markers[i];
		int MarkerID = marker.number;
		assert(MarkerID < (int)ArucoMap.size());
		if (ArucoMap[MarkerID] != -1)
		{
			cerr << "WARNING Overwriting Marker Misc/owner for marker index " << MarkerID << " with object " << object->Name << endl;
			assert(0);
		}
		ArucoMap[MarkerID] = index;
		ArucoSizes[MarkerID] = marker.sideLength;
	}
	for (size_t i = 0; i < object->childs.size(); i++)
	{
		RegisterArucoRecursive(object->childs[i], index);
	}
}