#include "Cameras/FakeCamera.hpp"

using namespace cv;
using namespace std;

bool FakeCamera::Grab()
{
	return true;
}

bool FakeCamera::Read()
{
	return true;
}

void FakeCamera::Inject(UMat& image)
{
	FrameBuffer.Reset();
	FrameBuffer.FrameRaw = image;
}
