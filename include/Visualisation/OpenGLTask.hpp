#pragma once

#include <vector>
#include <thread>
#include <mutex>
#include <memory>

class OpenGLTask
{
private:
	static std::vector<OpenGLTask*> Tasks;
	static std::recursive_mutex TaskMutex;
	static std::unique_ptr<std::thread> TaskThread;
protected:
	bool started = false;
	bool killed = false;
	bool initialized = false;
public:
	OpenGLTask();
	virtual ~OpenGLTask();

	void Start();

	bool IsKilled()
	{
		return killed;
	}

	void Kill()
	{
		killed = true;
	}

private:
	static void GlobalThreadEntryPoint();

protected:
	virtual void ThreadEntryPoint();
};
