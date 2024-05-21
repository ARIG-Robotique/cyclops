#pragma once

#include <Visualisation/BoardGL.hpp>
#include <Visualisation/OpenGLTask.hpp>

class CDFRExternal;

class ExternalBoardGL : public BoardGL, public OpenGLTask
{
private:
	CDFRExternal *Parent = nullptr;
	bool closed = false;
public:
	ExternalBoardGL(std::string InName = "Cyclops", CDFRExternal* InParent = nullptr);
	virtual ~ExternalBoardGL();

	bool IsThreaded()
	{
		return Parent != nullptr;
	}

	bool GetClosed()
	{
		return closed;
	}

protected:
	virtual void ThreadEntryPoint() override;
};
