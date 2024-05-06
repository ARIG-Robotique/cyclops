#include "PostProcessing/PostProcess.hpp"

using namespace std;

PostProcess::PostProcess(CDFRExternal* InOwner)
    :Owner(InOwner)
{
}

PostProcess::~PostProcess()
{
}

void PostProcess::Process(vector<CameraImageData> &ImageData, vector<CameraFeatureData> &FeatureData, vector<ObjectData> &Objects)
{
	(void) ImageData;
    (void) FeatureData;
    (void) Objects;
}