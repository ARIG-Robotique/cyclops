#include <Visualisation/external/ExternalBoardGL.hpp>
#include <EntryPoints/CDFRExternal.hpp>


ExternalBoardGL::ExternalBoardGL(std::string InName, CDFRExternal* InParent)
	:BoardGL(InName), Parent(InParent)
{
	if (InParent)
	{
		Start();
	}
	else
	{
		Init();
	}
}

ExternalBoardGL::~ExternalBoardGL()
{
}

void ExternalBoardGL::ThreadEntryPoint()
{
	if (!initialized)
	{
		assert(Parent != nullptr);
		Init();
		LoadTags();
		initialized = true;
	}
	if (!killed && !Parent->IsKilled())
	{
		closed = !Tick(ObjectData::ToGLObjects(Parent->GetObjectData()));
		killed |= closed;		
	}
	else
	{
		killed = true;
	}
}