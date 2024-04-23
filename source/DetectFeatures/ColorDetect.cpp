#include "DetectFeatures/ColorDetect.hpp"

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

using namespace cv;
using namespace std;

void DetectColor(const CameraImageData &InData, CameraFeatureData& OutData)
{
	(void) OutData;
	Mat HSVImage;
	cvtColor(InData.Image, HSVImage, COLOR_BGR2HSV);
	MatND Hist;
	// Quantize the hue to 30 levels
	// and the saturation to 32 levels
	const float hwidth = 40;
	const float hcenter = 60;
	const int hbins = hwidth, sbins = 16;
	const int histSize[] = {hbins, sbins};
	// hue varies from 0 to 179, see cvtColor
	const float hranges[] = { hcenter-hwidth/2, hcenter+hwidth/2 };
	// saturation varies from 0 (black-gray-white) to
	// 255 (pure spectrum color)
	const float sranges[] = { 0, 255 };
	const float* ranges[] = { hranges, sranges };
	// we compute the histogram from the 0-th and 1-st channels
	static const int channels[] = {0, 1};
	calcHist(&HSVImage, 1, channels, noArray(), Hist, 2, histSize, ranges, true, false);

	double maxVal=0;
	minMaxLoc(Hist, 0, &maxVal, 0, 0);
	int scale = 10;
	Mat HistImgHSV = Mat::zeros(sbins, hbins, CV_8UC3), HistImgRGB, HistImg;

	for( int h = 0; h < hbins; h++ )
	{
		for( int s = 0; s < sbins; s++ )
		{
			float binVal = Hist.at<float>(h, s);
			int intensity = cvRound(binVal*255/maxVal);
			Vec3b& dest = HistImgHSV.at<Vec3b>(s,h, 0);
			dest[0] = (h*hwidth)/hbins + hcenter-hwidth/2;
			dest[1] = 128 + s*127/sbins;
			dest[2] = intensity;
		}
	}

	cvtColor(HistImgHSV, HistImgRGB, COLOR_HSV2BGR, 3);
	resize(HistImgRGB, HistImg, Size(0,0), scale, scale, INTER_NEAREST);

	imshow("Histogram", HistImg);
	waitKey(1);
}

cv::Mat MultiThreshold(const cv::UMat &Image, const std::vector<cv::Vec3b> &Colors, cv::Vec3b tolerance, float dilateAmount, float erodeAmount)
{
	size_t num_colors = Colors.size();
	//convert colors to BGR
	Mat ColorsBGR(num_colors, 1, CV_8UC3), ColorsHSV(num_colors, 1, CV_8UC3);
	for (size_t colidx = 0; colidx < num_colors; colidx++)
	{
		ColorsHSV.at<Vec3b>(colidx) = Colors[colidx];
	}
	cvtColor(ColorsHSV, ColorsBGR, COLOR_HSV2BGR_FULL, 3);
	Mat ImageHSV, Components[3];

	//convert colors to thresholds
	vector<std::pair<Vec3b, Vec3b>> Thresholds(num_colors);
	for (size_t colidx = 0; colidx < num_colors; colidx++)
	{
		auto hsvcol = ColorsHSV.at<Vec3b>(colidx);
		Vec3b &maxcol = Thresholds[colidx].second, &mincol = Thresholds[colidx].first;
		for (size_t j = 0; j < hsvcol.channels; j++)
		{
			maxcol[j] = min<int>(hsvcol[j] + (int)tolerance[j], UINT8_MAX);
			mincol[j] = max<int>(hsvcol[j] - (int)tolerance[j], 0);
		}
	}

	//convert image to hsv and components
	cvtColor(Image, ImageHSV, COLOR_BGR2HSV_FULL, 3);
	split(ImageHSV, Components);

	const auto image_size = ImageHSV.size();
	Mat Selected(image_size, CV_8UC3);
	Image.copyTo(Selected);

	//make erode/dilate kernels and black backgronud
	Mat black = Mat::zeros(image_size, CV_8UC3);
	Mat dilateElement = getStructuringElement( MORPH_ELLIPSE,
		Size( 2*dilateAmount + 1, 2*dilateAmount+1 ),
		Point( dilateAmount, dilateAmount ) );
	Mat erodeElement = getStructuringElement( MORPH_ELLIPSE,
		Size( 2*erodeAmount + 1, 2*erodeAmount+1 ),
		Point( erodeAmount, erodeAmount ) );
	
	//do the stuff
	for (size_t colidx = 0; colidx < Colors.size(); colidx++)
	{
		Mat mask, acc(image_size, CV_8UC1, Scalar(UINT8_MAX));
		auto& color = Thresholds[colidx];
		for (int i = 0; i < 6; i++)
		{
			bool maxcomp = i&1;
			int compidx = i/2;
			if (tolerance[compidx] == UINT8_MAX)
			{
				continue;
			}
			
			auto& side = maxcomp ? color.second : color.first;
			uint8_t comparison = side[compidx];
			if (maxcomp)
			{
				if (comparison == UINT8_MAX)
				{
					continue;
				}
			}
			else
			{
				if (comparison == 0)
				{
					continue;
				}
			}
			
			

			threshold(Components[compidx], mask, comparison, UINT8_MAX, maxcomp ? THRESH_BINARY_INV : THRESH_BINARY);
			bitwise_and(acc, mask, acc);
		}
		dilate(acc, acc, dilateElement);
		erode(acc, acc, erodeElement);
		Mat colored(image_size, Selected.type(), ColorsBGR.at<Vec3b>(colidx));
		Mat notacc;
		bitwise_and(Selected, black, Selected, acc);
		bitwise_or(Selected, colored, Selected, acc);
	}
	
	return Selected;
}