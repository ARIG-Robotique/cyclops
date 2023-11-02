#pragma once

#include <map>
#include <cstdint>
#include <string>
#include <chrono>
#include <iostream>
#include <algorithm>

//Helper class to gather execution times of different parts of code and average over multiple cycles. It's a poor man's profiler.
template<bool enabled>
class ManualProfiler
{
	typedef std::chrono::steady_clock profclock;
	typedef std::chrono::time_point<profclock> proftime;
	typedef std::chrono::duration<double> profdelta;
private:
	std::string currsection;
	std::map<std::string, profdelta> timings;
	std::string ProfilerName;
	proftime lastswitchtick;
	proftime lastprinttick;
public:
	ManualProfiler(std::string InProfilerName = "")
		:currsection(""),
		ProfilerName(InProfilerName)
	{
		lastprinttick = profclock::now();
	}

	void EnterSection(std::string name)
	{
		if (!enabled)
		{
			return;
		}
		
		proftime entertick = profclock::now();
		if (currsection != "")
		{
			timings[currsection] += entertick - lastswitchtick;
		}
		
		currsection = name;
		lastswitchtick = profclock::now();
	}

	template<bool otheren>
	void operator+=(ManualProfiler<otheren>& other)
	{
		if (!enabled || !otheren)
		{
			return;
		}
		
		for (auto it = other.timings.begin(); it != other.timings.end(); it++)
		{
			timings[it->first] += it->second;
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
		for (auto it = timings.begin(); it != timings.end(); it++)
		{
			total += it->second;
		}
		for (auto it = timings.begin(); it != timings.end(); it++)
		{
			const std::string &name = it->first;
			profdelta& timing = it->second;
			std::cout << "\tSection " << name << "\" took " << timing.count() << "s, " << timing.count()/total.count()*100.0 << "%" << std::endl;
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
