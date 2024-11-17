#include "Visualisation/GLWindow.hpp"

#include <iostream>
#include <vector>
#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <opencv2/opencv.hpp>

using namespace std;

GLWindow::GLFWStates GLWindow::GLFWState = GLWindow::GLFWStates::Unitialised;
map<GLFWwindow*, GLWindow*> GLWindow::windowmap;

void window_size_callback_generic(GLFWwindow* window, int width, int height)
{
	GLWindow* target = GLWindow::windowmap.at(window);
	target->WindowSizeCallback(width, height);
}

bool GLWindow::GLInit()
{
	switch (GLFWState)
	{
	case GLFWStates::Working:
		return true;
	case GLFWStates::InitFail:
		return false;
	default:
		break;
	}
	//cout << "Creating OpenGL context" << endl;

	// Initialise GLFW
	glewExperimental = true; // Needed for core profile
	if( !glfwInit() )
	{
		cerr << "Failed to initialize GLFW" << endl;
		GLFWState = GLFWStates::InitFail;
		return false;
	}
	GLFWState = GLFWStates::Working;
	return true;
}

GLWindow::~GLWindow()
{
	if (Window != nullptr)
	{
		windowmap.erase(Window);
		glfwDestroyWindow(Window);
	}
}

GLFWwindow* GLWindow::GLCreateWindow(int width, int height, std::string name)
{
	if (Window != nullptr)
	{
		return Window;
	}
	if(!GLInit())
	{
		return nullptr;
	}
	glfwWindowHint(GLFW_SAMPLES, 4); // 4x antialiasing
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3); // We want OpenGL 3.3
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE); // To make MacOS happy; should not be needed
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE); // We don't want the old OpenGL 

	// Open a window and create its OpenGL context
	Window = glfwCreateWindow( width, height, name.c_str(), NULL, NULL);
	if( Window == NULL ){
		cerr << "Failed to open GLFW window." << endl;
		glfwTerminate();
		return nullptr;
	}
	glfwMakeContextCurrent(Window); // Initialize GLEW
	if (glewInit() != GLEW_OK) {
		cerr << "Failed to initialize GLEW" << endl;
		return nullptr;
	}

	//glfwSwapInterval( 0 ); //disable vsync


	glfwSetWindowSizeCallback(Window, window_size_callback_generic);
	windowmap[Window] = this;
	return Window;
}

void GLWindow::CaptureWindow(std::filesystem::path path)
{
	cv::Mat frame;
	{
		int width, height;
		glfwGetFramebufferSize(Window, &width, &height);
		std::vector<unsigned char> pixels(3 * width * height);
		glReadPixels(0, 0, width, height, GL_BGR, GL_UNSIGNED_BYTE, pixels.data());
		cv::Mat flipped(height, width, CV_8UC3, pixels.data());
		cv::flip(flipped, frame, 0);
	}
	cv::imwrite(path, frame);
}

void GLWindow::WindowSizeCallback(int width, int height)
{
	(void)width;
	(void)height;
}