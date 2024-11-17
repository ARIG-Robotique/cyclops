#pragma once
#include <string>
#include <map>
#include <glm/glm.hpp>
#include <filesystem>
class GLFWwindow;

//parent class for all openGL stuff
class GLWindow
{
protected:
	enum class GLFWStates
	{
		Unitialised,
		Working,
		InitFail
	};
	static GLFWStates GLFWState;
	static bool GLInit();

protected:
	static std::map<GLFWwindow*, GLWindow*> windowmap;
	GLFWwindow* Window = nullptr;
	int FrameIndex = 0;

public:
	static bool IsOpenGLWorking()
	{
		return GLFWState == GLFWStates::Working;
	}

	GLFWwindow* GetWindow() const
	{
		return Window;
	}

	bool HasWindow() const
	{
		return GetWindow();
	}

	virtual ~GLWindow();

	/*
	Creates a window.
	Context is set to be the newly created window
	*/
	GLFWwindow* GLCreateWindow(int width, int height, std::string name);

	/*
	Capture the OpenGL window and writes to file at path
	*/
	void CaptureWindow(std::filesystem::path path);

protected:
	virtual void WindowSizeCallback(int width, int height);

	friend void window_size_callback_generic(GLFWwindow* window, int width, int height);
};