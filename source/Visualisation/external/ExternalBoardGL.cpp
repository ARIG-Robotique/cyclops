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
	assert(Parent != nullptr);
	Init();
	LoadTags();
	while (!killed && !Parent->IsKilled())
	{
		closed = !Tick(ObjectData::ToGLObjects(Parent->GetObjectData()));
		killed |= closed;		
	}
	killed = true;
}