#include "SiftPipeline/SiftTest.hpp"

#include <string>
#include <iostream>
#include <thread>
#include <filesystem>
#include <vector>
#include <map>


#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>

#include "math3d.hpp"
#include "data/metadata.hpp"
#include "data/ManualProfiler.hpp"
#include "Calibfile.hpp"
#include "GlobalConf.hpp"
#include "BoardGL.hpp"

using namespace cv;
using namespace std;
namespace fs = std::filesystem;

const string SiftImgPath = "Sift";

void SiftTest(void)
{
	ManualProfiler<true> prof("Sift Timing", {"Terrain", "TerrainDraw", "ImageSift", "ImageMatch", "ImageFilter", "ImageDraw"});
	vector<UMat> images;
	if(fs::exists(SiftImgPath) && fs::is_directory(fs::path(SiftImgPath)))
	{
		for (const auto & entry : fs::directory_iterator(SiftImgPath))
		{
			try
			{
				UMat read;
				imread(entry.path().string()).copyTo(read);
				images.push_back(read);
			}
			catch(const std::exception& e)
			{
				std::cerr << "Error loading image at " << entry.path().string() << " : " << e.what() << '\n';
			}
		}
	}
	cout << "Found " << images.size() << " images to try sift on" << endl;
	UMat terrain;
	imread("../assets/board.png").copyTo(terrain);
	auto finder = SIFT::create();
	auto matcher = BFMatcher::create(NORM_HAMMING);
	prof.EnterSection(0);
	vector<KeyPoint> terrainkeypoints; Mat terraindescs;
	finder->detectAndCompute(terrain, noArray(), terrainkeypoints, terraindescs);
	prof.EnterSection(1);
	UMat terrainwithkp;
	drawKeypoints(terrain, terrainkeypoints, terrainwithkp);
	//imshow("terrain, sifted", terrainwithkp);
	imwrite("siftout/terrainsifted.png", terrainwithkp);
	//waitKey();
	/*parallel_for_(Range(0,images.size()), [&](Range InRange)
	{*/
	Range InRange(0, images.size());
		for (int i = InRange.start; i<InRange.end; ++i)
		{
			UMat& image = images[i];
			prof.EnterSection(2);
			vector<KeyPoint> imkeypoints; Mat imdescs;
			finder->detectAndCompute(image, noArray(), imkeypoints, imdescs);
			prof.EnterSection(3);
			vector<vector<DMatch>> matches;
			matcher->knnMatch(terraindescs, imdescs, matches, 2);
			prof.EnterSection(4);
			const float ratio_thresh = 1.f;
			//std::vector<DMatch> good_matches = matches;
			/*for (size_t i = 0; i < matches.size(); i++)
			{
				if (matches[i][0].distance < ratio_thresh * matches[i][1].distance)
				{
					good_matches.push_back(matches[i][0]);
				}
			}*/
			UMat imgmatch;
			prof.EnterSection(5);
			drawMatches(terrain, terrainkeypoints, image, imkeypoints, matches, imgmatch);
			imwrite("siftout/" + to_string(i) + ".png", imgmatch);
		}
	//});
	prof.EnterSection(-1);
	prof.PrintProfile();
	
}