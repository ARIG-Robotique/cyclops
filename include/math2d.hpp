#pragma once

#include <cmath>

template <class T>
T wraptwopi(T in)
{
	in = fmod(in, M_PI*2);
	if (in > M_PI)
	{
		in -= M_PI*2;
	}
	if (in < -M_PI)
	{
		in += M_PI*2;
	}
	return in;
}