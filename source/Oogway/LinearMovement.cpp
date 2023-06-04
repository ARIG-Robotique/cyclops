#include "Oogway/LinearMovement.hpp"

#include <cmath>
#include <cassert>
#include "math2d.hpp"

using namespace std;
using namespace Oogway;

double LinearMovement::closestangle(double x, double ref)
{
	double delta = wraptwopi(x-ref);
	return delta + ref;
}

void LinearMovement::SetTarget(double NewTarget)
{
	TargetPos = NewTarget;
}

void LinearMovement::SetTargetAngular(double angle)
{
	Pos = wraptwopi(Pos);
	TargetPos = closestangle(angle, Pos);
}

double LinearMovement::GetBrakingDistance(double v0) 
{
	double v1 = copysign(MinSpeed, v0);
	double dec = -copysign(Deceleration, v0);
	return SpeedDeltaDistance(v0, v1, dec);
}

double LinearMovement::GetStoppingPosition()
{
	return Pos+GetBrakingDistance(Speed);
}

LinearMovement::MoveABResult LinearMovement::MoveAB(double Target, double &TimeBudget)
{
	double dx = Target - Pos;
	bool InitialMoving = abs(Speed) > MinSpeed + __DBL_EPSILON__;
	bool GoingTheRightWay = Speed*dx>0;
	if (InitialMoving && !GoingTheRightWay)
	{
		//cout << "Going the wrong way" << endl;
		return MoveABResult::WrongDirection;
	}
	
	double TimeToMaxSpeed, TimeFromMaxSpeed;
	double DistToMaxSpeed, DistFromMaxSpeed;
	Speed = abs(Speed) < MinSpeed ? copysign(MinSpeed, dx) : Speed; //Start speed
	double acc = copysign(Acceleration, dx);
	double dec = -copysign(Deceleration, dx);
	{ //current speed to full speed
		double v0 = Speed;
		double v1 = copysign(MaxSpeed, dx);
		
		TimeToMaxSpeed = SpeedDeltaTime(v0, v1, acc);
		DistToMaxSpeed = SpeedDeltaDistance(v0, v1, acc);
		//assert(TimeToMaxSpeed >-__DBL_EPSILON__);
	}
	{ //full speed to min speed
		double v0 = copysign(MaxSpeed, dx);
		double v1 = copysign(MinSpeed, dx);
		TimeFromMaxSpeed = SpeedDeltaTime(v0, v1, dec);
		DistFromMaxSpeed = SpeedDeltaDistance(v0, v1, dec);
		assert(TimeFromMaxSpeed >0);
	}
	double BrakingDistance = GetBrakingDistance(Speed);
	/*cout 
	<< "dx = " << dx 
	<< " | Speed = " << Speed 
	<< " | Acc = " << acc 
	<< " | Dec = " << dec 
	<< " | Time to max speed = " << TimeToMaxSpeed 
	<< " | Dist to max speed = " << DistToMaxSpeed 
	<< " | Time from max speed = " << TimeFromMaxSpeed 
	<< " | Dist from max speed = " << DistFromMaxSpeed
	<< " | Braking distance = " << BrakingDistance
	<< endl;*/
	if (abs(BrakingDistance) > abs(dx)) //will overshoot, brake hard
	{
		//cout << "Will overshoot" << endl;
		double v0 = Speed;
		double v1 = copysign(MinSpeed, dx);
		double TimeToStop = SpeedDeltaTime(v0, v1, dec);
		if (TimeBudget < TimeToStop)
		{
			double v1 = Speed + TimeBudget*dec;
			Pos += SpeedDeltaDistance(Speed, v1, dec);
			Speed = v1;
			TimeBudget = 0;
			return MoveABResult::NoTimeLeft;
		}
		TimeBudget-=TimeToStop;
	}
	else if (abs(DistFromMaxSpeed + DistToMaxSpeed) > abs(dx)) //no time for full speed
	{
		//cout << "Speed triangle" << endl;
		double step = copysign(MaxSpeed, dx) - copysign(MinSpeed, dx);
		step /= 2;
		double peakspeed = copysign(MinSpeed, dx) + step;
		step /= 2;
		double TimeToPeak, TimeFromPeak;
		double DistToPeak, DistFromPeak;
		for (; abs(step) > __DBL_EPSILON__;)
		{
			double v0 = Speed;
			double v1 = peakspeed;
			double v2 = copysign(MinSpeed, dx);
			TimeToPeak = SpeedDeltaTime(v0, v1, acc);
			DistToPeak = SpeedDeltaDistance(v0, v1, acc);
			TimeFromPeak = SpeedDeltaTime(v1, v2, dec);
			DistFromPeak = SpeedDeltaDistance(v1, v2, dec);
			if (abs(DistToPeak+DistFromPeak) > abs(dx))
			{
				//peak speed is lower
				peakspeed -= step;
			}
			else
			{
				peakspeed += step;
			}
			step /= 2;
		}
		//assert(TimeToPeak >-__DBL_EPSILON__*8);
		//assert(TimeFromPeak >-__DBL_EPSILON__*8);
		TimeToPeak = max(TimeToPeak, 0.0);
		TimeFromPeak = max(TimeFromPeak, 0.0);
		if (TimeBudget < TimeToPeak) //Not enough time to reach peak
		{
			double v1 = Speed + TimeBudget*acc;
			Pos += SpeedDeltaDistance(Speed, v1, acc);
			Speed = v1;
			TimeBudget = 0;
			return MoveABResult::NoTimeLeft;
		}
		Pos += DistToPeak;
		TimeBudget -= TimeToPeak;
		Speed = peakspeed;
		if (TimeBudget < TimeFromPeak) //Not enough time to fully stop
		{
			double v1 = Speed + TimeBudget*dec;
			Pos += SpeedDeltaDistance(Speed, v1, dec);
			Speed = v1;
			TimeBudget = 0;
			return MoveABResult::NoTimeLeft;
		}
		TimeBudget -= TimeFromPeak;
	}
	else
	{
		//cout << "Constant speed" << endl;
		double DistFullSpeed = copysign(abs(dx) - abs(DistToMaxSpeed + DistFromMaxSpeed), dx);
		double TimeFullSpeed = DistFullSpeed/copysign(MaxSpeed, dx);
		assert(TimeFullSpeed > -__DBL_EPSILON__);

		if (TimeBudget < TimeToMaxSpeed)
		{
			double v1 = Speed + TimeBudget*acc;
			Pos += SpeedDeltaDistance(Speed, v1, acc);
			Speed = v1;
			TimeBudget = 0;
			return MoveABResult::NoTimeLeft;
		}
		Pos += DistToMaxSpeed;
		TimeBudget -= TimeToMaxSpeed;
		Speed = copysign(MaxSpeed, dx);
		if (TimeBudget < TimeFullSpeed)
		{
			Pos += Speed * TimeBudget;
			TimeBudget = 0;
			return MoveABResult::NoTimeLeft;
		}
		Pos += DistFullSpeed;
		TimeBudget -= TimeFullSpeed;
		if (TimeBudget < TimeFromMaxSpeed)
		{
			double v1 = Speed + TimeBudget*dec;
			Pos += SpeedDeltaDistance(Speed, v1, dec);
			Speed = v1;
			TimeBudget = 0;
			return MoveABResult::NoTimeLeft;
		}
		TimeBudget -= TimeFromMaxSpeed;
	}
	Pos = Target;
	Speed = 0;
	return MoveABResult::Done;
	
}

double LinearMovement::Tick(double &TimeBudget)
{
	MoveABResult result;
	int i = 0;
	do
	{
		result = MoveAB(TargetPos, TimeBudget);
		switch (result)
		{
		case MoveABResult::WrongDirection :
			double inttarget = Pos + GetBrakingDistance(Speed);
			//cout << "Wrong direction : new target is " << inttarget << endl;
			result = MoveAB(inttarget, TimeBudget);
			break;
		}
		i++;
	} while (abs(Pos - TargetPos)>__DBL_EPSILON__ && TimeBudget > __DBL_EPSILON__ && i < 10);
	return TimeBudget;
}