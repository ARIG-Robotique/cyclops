#pragma once

#include <vector>
#include <cstdint>
#include <string>
#include <chrono>
#include <iostream>

//Helper class to gather execution times of different parts of code and average over multiple cycles. It's a poor man's profiler.
template<bool enabled>
class ManualProfiler
{
	typedef std::chrono::steady_clock profclock;
	typedef std::chrono::time_point<profclock> proftime;
	typedef std::chrono::duration<double> profdelta;
private:
	int currsection;
	std::vector<profdelta> timings;
	std::string ProfilerName;
	std::vector<std::string> names;
	proftime lastswitchtick;
	proftime lastprinttick;
public:
	ManualProfiler(std::string InProfilerName = "", std::vector<std::string> InNames = {})
		:currsection(-1),
		ProfilerName(InProfilerName),
		names(InNames)
	{
		lastprinttick = profclock::now();
	}

	void EnterSection(int sectionnumber)
	{
		if (!enabled)
		{
			return;
		}
		
		proftime entertick = profclock::now();
		if (currsection != -1)
		{
			if (timings.size() <= currsection)
			{
				timings.resize(currsection +1, profdelta(0));
			}
			timings[currsection] += entertick - lastswitchtick;
		}
		currsection = sectionnumber;
		lastswitchtick = profclock::now();
	}
	void NameSection(int sectionnumber, std::string name)
	{
		if (!enabled)
		{
			return;
		}
		
		if (names.size() <= sectionnumber)
		{
			names.resize(sectionnumber +1, std::string("No Name"));
		}
		names[sectionnumber] = name;
	}

	template<bool otheren>
	void operator+=(ManualProfiler<otheren>& other)
	{
		if (!enabled || !otheren)
		{
			return;
		}
		
		if (other.timings.size() > timings.size())
		{
			timings.resize(other.timings.size(), profdelta(0));
		}
		
		for (int i = 0; i < other.timings.size(); i++)
		{
			timings[i] += other.timings[i];
		}
	}

	void PrintProfile()
	{
		if (!enabled)
		{
			return;
		}
		
		std::cout << "Profiling " << ProfilerName <<":" << std::endl;
		profdelta total(0);
		for (int i = 0; i < timings.size(); i++)
		{
			total += timings[i];
		}
		for (int i = 0; i < timings.size(); i++)
		{
			std::string name = "No Name";
			if (i < names.size())
			{
				name = names[i];
			}
			std::cout << "\tSection " << i << " \"" << name << "\" took " << timings[i].count() << "s, " << timings[i].count()/total.count()*100.0 << "%" << std::endl;
		}
		lastprinttick = profclock::now();
	}

	bool ShouldPrint()
	{
		if (!enabled)
		{
			return false;
		}
		profdelta TimeSinceLastPrint = profclock::now() - lastprinttick;
		return TimeSinceLastPrint.count() > 10;
	}

	void PrintIfShould()
	{
		if (ShouldPrint())
		{
			PrintProfile();
		}
	}
};
