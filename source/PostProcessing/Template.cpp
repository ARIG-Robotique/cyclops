#include <PostProcessing/Template.hpp>
#include <EntryPoints/CDFRExternal.hpp>

void PostProcessTemplate::Process(std::vector<CameraImageData> &ImageData, std::vector<CameraFeatureData> &FeatureData, std::vector<ObjectData> &Objects)
{
	(void) ImageData;
	(void) FeatureData;
	assert(Owner != nullptr);
	Objects.emplace_back(ObjectType::Team, TeamNames.at(Owner->GetTeam()).JavaName);
}