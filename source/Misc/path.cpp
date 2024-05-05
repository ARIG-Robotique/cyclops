#include "Misc/path.hpp"

using namespace std;

filesystem::path ExecutablePath, CyclopsPath;

filesystem::path GetExecutablePath()
{
    return ExecutablePath;
}
filesystem::path GetCyclopsPath()
{
    return CyclopsPath;
}

void SetExecutablePath(const char* path)
{
    ExecutablePath = filesystem::weakly_canonical(filesystem::path(path));
    CyclopsPath = ExecutablePath.parent_path().parent_path();
}