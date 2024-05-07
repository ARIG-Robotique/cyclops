#pragma once

#include <memory>
#include <vector>

#include <imgui.h>
#include <Visualisation/GLWindow.hpp>
#include <Visualisation/openGL/Texture.hpp>
#include <Misc/Task.hpp>
#include <opencv2/core.hpp>

class CDFRExternal;

class ImguiWindow : public GLWindow, public Task
{

	static bool ImguiOpenGLInit;
	static std::string ImguiIniPath;
	std::string WindowName;
	CDFRExternal *Parent;
	bool closed = false;
	bool ShowAruco = true, ShowYolo = true;
	bool FocusPeeking = false;

	std::vector<Texture> Textures;

public:
	ImguiWindow(std::string InWindowName = "ImGui", CDFRExternal *InParent = nullptr);
	virtual ~ImguiWindow();

	virtual void WindowSizeCallback(int width, int height) override;

	ImVec2 GetWindowSize();

	bool IsThreaded()
	{
		return Parent != nullptr;
	}

	bool GetClosed()
	{
		return closed;
	}
	
	void Init();

	void StartFrame();
	bool EndFrame();

	void AddImageToBackground(const Texture &Image, cv::Rect impos, 
		cv::Size2f UVmin = cv::Size2f(0,0), cv::Size2f UVmax = cv::Size2f(1,1));

protected:

	virtual void ThreadEntryPoint() override;
};


