

#include "Misc/math3d.hpp"
#include <math.h>
#include <glm/glm.hpp>

using namespace cv;
using namespace std;




Matx33d ImageToWorld()
{
	return Matx33d(1,0,0, 0,-1,0, 0,0,1);
}

double GetVectorLengthSquared(Vec3d x)
{
	return x.ddot(x);
}

Vec3d NormaliseVector(Vec3d x)
{
	return x / sqrt(GetVectorLengthSquared(x));
}

Vec3d MakeVectorOrthogonal(Vec3d base, Vec3d x)
{
	double comp = x.ddot(base);
	Vec3d rem = x-base*comp;
	return NormaliseVector(rem);
}

Matx33d MakeRotationFromXYZ(Vec3d X, Vec3d Y, Vec3d Z)
{
	Matx33d outputmatrix;
	//ensure orthonormal
	X = NormaliseVector(X); 
	Y = MakeVectorOrthogonal(X, Y); 
	Z = MakeVectorOrthogonal(Y, MakeVectorOrthogonal(X, Z));

	Vec3d vectors[3] = {X, Y, Z};
	for (int i = 0; i < 3; i++)
	{
		//outputmatrix.col(i) = vectors[i];
		for (int j = 0; j < 3; j++)
		{
			outputmatrix(j,i) = vectors[i](j);
		}
		
	}
	return outputmatrix;
}

Matx33d MakeRotationFromXY(Vec3d X, Vec3d Y)
{
	X = NormaliseVector(X);
	Y = MakeVectorOrthogonal(X, Y);
	Vec3d Z = X.cross(Y);
	return MakeRotationFromXYZ(X, Y, Z);
}


Matx33d MakeRotationFromXZ(Vec3d X, Vec3d Z)
{
	X = NormaliseVector(X);
	Z = MakeVectorOrthogonal(X, Z);
	Vec3d Y = Z.cross(X);
	return MakeRotationFromXYZ(X, Y, Z);
}

Matx33d MakeRotationFromZX(Vec3d Z, Vec3d X)
{
	Z = NormaliseVector(Z);
	X = MakeVectorOrthogonal(Z, X);
	Vec3d Y = Z.cross(X);
	return MakeRotationFromXYZ(X, Y, Z);
}

Matx33d MakeRotationFromZY(Vec3d Z, Vec3d Y)
{
	Z = NormaliseVector(Z);
	Y = MakeVectorOrthogonal(Z, Y);
	Vec3d X = Y.cross(Z);
	return MakeRotationFromXYZ(X, Y, Z);
}

Matx31d GetAxis(Matx33d rotation, int i)
{
	return rotation.col(i);
}

double GetRotZ(Matx33d rotation)
{
	Matx31d Xaxis = GetAxis(rotation, 0);
	Xaxis(2,0) = 0; //project on XY
	Xaxis /= sqrt(Xaxis.ddot(Xaxis));
	return atan2(Xaxis(1,0), Xaxis(0,0));
}

Vec3d LinePlaneIntersection(Vec3d LineOrigin, Vec3d LineDirection, Vec3d PlaneOrigin, Vec3d PlaneNormal)
{
	LineDirection = NormaliseVector(LineDirection);
	PlaneNormal = NormaliseVector(PlaneNormal);
	double t = (PlaneNormal.ddot(PlaneOrigin) - PlaneNormal.ddot(LineOrigin))/PlaneNormal.ddot(LineDirection); 
	return LineOrigin + LineDirection*t;
}

Vec3d ProjectPointOnLine(Vec3d Point, Vec3d LineOrig, Vec3d LineDir)
{
	Vec3d LineToPoint = Point-LineOrig;
	double dist = LineDir.ddot(LineToPoint);
	Vec3d projected = LineOrig + dist * LineDir;
	return projected;
}

bool ClosestPointsOnTwoLine(Vec3d Line1Orig, Vec3d Line1Dir, Vec3d Line2Orig, Vec3d Line2Dir, Vec3d &Line1Point, Vec3d &Line2Point)
{
	/*https://math.stackexchange.com/questions/846054/closest-points-on-two-line-segments*/

	// Algorithm is ported from the C algorithm of 
	// Paul Bourke at http://local.wasp.uwa.edu.au/~pbourke/geometry/lineline3d/

	Vec3d p13 = Line1Orig - Line2Orig;
	Vec3d p43 = Line2Dir;

	/*if (p43.LengthSq() < Math.Epsilon) {
		return false;
	}*/
	Vec3d p21 = Line1Dir;
	/*if (p21.LengthSq() < Math.Epsilon) {
		return false;
	}*/

	double d1343 = p13.ddot(p43);
	double d4321 = p43.ddot(p21);
	double d1321 = p13.ddot(p21);
	double d4343 = p43.ddot(p43);
	double d2121 = p21.ddot(p21);

	double denom = d2121 * d4343 - d4321 * d4321;
	if (abs(denom) < 1e-9) {
		return false;
	}
	double numer = d1343 * d4321 - d1321 * d4343;

	double mua = numer / denom;
	double mub = (d1343 + d4321 * (mua)) / d4343;

	Line1Point = Line1Orig + mua*p21;
	Line2Point = Line2Orig + mub*p43;

	return true;


	/*Vec3d V21 = Line2Orig - Line1Orig;
	double v11 = Line1Dir.ddot(Line1Dir);
	double v22 = Line2Dir.ddot(Line2Dir);
	double v21 = Line2Dir.ddot(Line1Dir);
	double v21_1 = V21.ddot(Line1Dir);
	double v21_2 = V21.ddot(Line2Dir);
	double denom = v21 * v21 - v22 * v11;
	double s = 0, t = 0;
	if (abs(denom) < 1e-9)
	{
		return false;
	}
	s = (v21_2 * v21 - v22 * v21_1) / denom;
	t = (-v21_1 * v21 + v11 * v21_2) / denom;

	Line1Point = Line1Orig + s * Line1Dir;
	Line2Point = Line2Orig + t * Line2Dir;
	return true;*/
}

glm::mat4 Affine3DToGLM(Affine3d Location)
{
	glm::mat4 outmat;
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			outmat[j][i] = Location.matrix(i,j);
		}
	}
	return outmat;
}

bool SolvePnPUpright(Matx31d UpVector, double MinColinearity,
					InputArray objectPoints, InputArray imagePoints,
					InputArray cameraMatrix, InputArray distCoeffs,
					Mat& rvecs, Mat& tvecs,
					bool useExtrinsicGuess, SolvePnPMethod flags,
					InputArray rvec, InputArray tvec,
					OutputArray reprojectionError)
{
	vector<Mat> rvecsArray, tvecsArray;
	int numsolutions = solvePnPGeneric(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvecsArray, tvecsArray, useExtrinsicGuess, flags, rvec, tvec, reprojectionError);
	int bestidx = -1;
	double bestcos = MinColinearity;
	for (int i = 0; i < numsolutions; i++)
	{
		Mat RotationMatrix;
		Rodrigues(rvecsArray[i], RotationMatrix);
		double Colinearity = GetAxis(RotationMatrix, 2).ddot(UpVector);
		if (Colinearity > bestcos)
		{
			bestcos = Colinearity;
			bestidx = i;
		}
	}
	if (bestidx >= 0)
	{
		rvecs = rvecsArray[bestidx];
		tvecs = tvecsArray[bestidx];
		return true;
	}
	return false;
}