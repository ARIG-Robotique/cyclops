#include <Visualisation/OpenGLTask.hpp>
#include <algorithm>
#include <thirdparty/thread-rename.hpp>

using namespace std;

vector<OpenGLTask*> OpenGLTask::Tasks;
mutex OpenGLTask::TaskMutex;
unique_ptr<thread> OpenGLTask::TaskThread;

OpenGLTask::OpenGLTask()
{
}

OpenGLTask::~OpenGLTask()
{
	bool killThread = false;
	if (started)
	{
		std::lock_guard TaskLock(TaskMutex);
		Tasks.erase(find(Tasks.begin(), Tasks.end(), this), Tasks.end());
		if (Tasks.size() == 0)
		{
			killThread = true;
		}
	}
	if (killThread)
	{
		TaskThread->join();
		TaskThread.reset();
	}	
}

void OpenGLTask::Start()
{
	if (started)
	{
		return;
	}
	started = true;
	std::lock_guard TaskLock(TaskMutex);
	Tasks.push_back(this);
	if (!TaskThread)
	{
		TaskThread = make_unique<thread>(GlobalThreadEntryPoint);
	}
}

void OpenGLTask::GlobalThreadEntryPoint()
{
	SetThreadName("OpenGLTask");
	while (1)
	{
		std::lock_guard TaskLock(TaskMutex);
		for (auto &Task : Tasks)
		{
			if (Task->IsKilled())
			{
				continue;
			}
			Task->ThreadEntryPoint();
		}
		if (Tasks.size() == 0)
		{
			break;
		}
	}
}

void OpenGLTask::ThreadEntryPoint()
{
}