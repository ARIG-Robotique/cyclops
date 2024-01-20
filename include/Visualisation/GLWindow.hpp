#pragma once
#include <string>
#include <map>
#include <glm/glm.hpp>
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

protected:
	virtual void WindowSizeCallback(int width, int height);

	friend void window_size_callback_generic(GLFWwindow* window, int width, int height);
};