#pragma once

#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/core/affine.hpp>
#include <opencv2/calib3d.hpp>
#include <glm/glm.hpp>



//Matrix to turn an image from it's image space (X right, Y down) to world space
cv::Matx33d ImageToWorld();

template<class AType, class BType>
float ComputeReprojectionError(const std::vector<cv::Point_<AType>> &A, const std::vector<cv::Point_<BType>> &B)
{
	float ReprojectionError = 0;
	for (size_t i = 0; i < A.size(); i++)
	{
		cv::Point_<AType> diff = A[i] - cv::Point_<AType>(B[i]);
		ReprojectionError += sqrt(diff.ddot(diff));
	}
	return ReprojectionError;
}


double GetVectorLengthSquared(cv::Vec3d x);

cv::Vec3d NormaliseVector(cv::Vec3d x);

//removes the component of base from x, then normalises it
//Base must be of unit length
cv::Vec3d MakeVectorOrthogonal(cv::Vec3d base, cv::Vec3d x);

//makes a rotation matrix from identity to those 3 vectors
cv::Matx33d MakeRotationFromXYZ(cv::Vec3d X, cv::Vec3d Y, cv::Vec3d Z);

cv::Matx33d MakeRotationFromXY(cv::Vec3d X, cv::Vec3d Y);

cv::Matx33d MakeRotationFromXZ(cv::Vec3d X, cv::Vec3d Z);

cv::Matx33d MakeRotationFromZX(cv::Vec3d Z, cv::Vec3d X);

cv::Matx33d MakeRotationFromZY(cv::Vec3d Z, cv::Vec3d Y);

cv::Matx31d GetAxis(cv::Matx33d rotation, int i);

//Get the rotation around Z axis
double GetRotZ(cv::Matx33d rotation);

cv::Vec3d LinePlaneIntersection(cv::Vec3d LineOrigin, cv::Vec3d LineDirection, cv::Vec3d PlaneOrigin, cv::Vec3d PlaneNormal);

cv::Vec3d ProjectPointOnLine(cv::Vec3d Point, cv::Vec3d LineOrig, cv::Vec3d LineDir);

bool ClosestPointsOnTwoLine(cv::Vec3d Line1Orig, cv::Vec3d Line1Dir, cv::Vec3d Line2Orig, cv::Vec3d Line2Dir, cv::Vec3d& Line1Point, cv::Vec3d& Line2Point);

glm::mat4 Affine3DToGLM(cv::Affine3d Location);

bool SolvePnPUpright(cv::Matx31d UpVector, double MinColinearity,
					cv::InputArray objectPoints, cv::InputArray imagePoints,
					cv::InputArray cameraMatrix, cv::InputArray distCoeffs,
					cv::Mat& rvecs, cv::Mat& tvecs,
					bool useExtrinsicGuess = false, cv::SolvePnPMethod flags = cv::SOLVEPNP_ITERATIVE,
					cv::InputArray rvec = cv::noArray(), cv::InputArray tvec = cv::noArray(),
					cv::OutputArray reprojectionError = cv::noArray());