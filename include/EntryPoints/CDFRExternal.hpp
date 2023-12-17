#pragma once

#include <thread>
#include <vector>
#include <array>
#include <memory>

#include <Communication/ProcessedTypes.hpp>
#include <ArucoPipeline/ObjectIdentity.hpp>


class CDFRExternal
{
private:
    bool killed = false;
    bool direct;
    bool v3d;

    int BufferIndex = 0;

    std::unique_ptr<std::thread> ThreadHandle;

    std::array<std::vector<CameraFeatureData>, 3> FeatureData;
    std::array<std::vector<ObjectData>, 3> ObjData;

    CDFRTeam GetTeamFromCameraPosition(std::vector<class Camera*> Cameras);
    
public:
    bool Idle = false;

    void ThreadEntryPoint();

    void GetData(std::vector<CameraFeatureData> &OutFeatureData, std::vector<ObjectData> &OutObjectData);

    bool IsKilled()
    {
        return killed;
    }

    CDFRExternal(bool InDirect, bool InV3D);
    ~CDFRExternal();

};




