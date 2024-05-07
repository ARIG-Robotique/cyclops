#pragma once

#include <PostProcessing/PostProcess.hpp>
#include <vector>

class PostProcessYoloDeflicker : public PostProcess
{
	struct YoloObject : public ObjectData
	{
		bool Associated;
		ObjectData::TimePoint Lifetime;

		YoloObject(ObjectData& obj)
			:ObjectData(obj)
		{
			Associated = false;
			Lifetime = obj.LastSeen;
			*this += obj;
		}

		bool operator==(ObjectData& other)
		{
			if (Associated)
			{
				return false;
			}
			
			if (other.type != type)
			{
				if ((type == ObjectType::Fragile && other.type == ObjectType::Resistant) || (other.type == ObjectType::Fragile && type == ObjectType::Resistant))
				{
				}
				else
				{
					return false;
				}
			}
			cv::Vec3d delta = other.location.translation() - location.translation();
			double distance = std::sqrt(delta.ddot(delta));
			return distance < 0.02;
		} 

		void operator+=(ObjectData& other)
		{
			assert(!Associated);
			cv::Vec3d mean = (other.location.translation() + location.translation())/2;
			location.translation(mean);
			int confidence = other.metadata.at("confidence");
			Lifetime += std::chrono::milliseconds(confidence*10);//if 100% confident, add 1s lifetime
			Lifetime = std::max(Lifetime, ObjectData::Clock::now() + std::chrono::seconds(3)); //max 3s lifetime
			LastSeen = other.LastSeen;
			type = other.type;
		}
	};
	std::vector<YoloObject> CachedObjects;
public:
	PostProcessYoloDeflicker(CDFRExternal* InOwner)
		:PostProcess(InOwner)
	{}

	static bool IsYolo(const ObjectData& obj);

	virtual void Process(std::vector<CameraImageData> &ImageData, std::vector<CameraFeatureData> &FeatureData, std::vector<ObjectData> &Objects) override;
};
