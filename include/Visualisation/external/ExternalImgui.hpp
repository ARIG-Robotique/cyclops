#pragma once

#include <Visualisation/ImguiWindow.hpp>
#include <Visualisation/OpenGLTask.hpp>

class CDFRExternal;

class ExternalImgui : public ImguiWindow, public OpenGLTask
{
private:
	CDFRExternal *Parent;
	bool closed = false;
	bool ShowAruco = true, ShowYolo = true;
	bool FocusPeeking = false;
	std::vector<cv::UMatData*> LastMatrices;
public:

	ExternalImgui(std::string InWindowName = "ImGui", CDFRExternal *InParent = nullptr);
	virtual ~ExternalImgui();

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
