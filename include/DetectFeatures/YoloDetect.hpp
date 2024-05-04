#pragma once

#include <Cameras/ImageTypes.hpp>
#include <Communication/ProcessedTypes.hpp>
#include <ArucoPipeline/ObjectIdentity.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/core.hpp>
#include <vector>
#include <filesystem>

class YoloDetect
{
private:
	struct Detection
	{
		cv::Rect BoundingBox;
		float Confidence;
		std::vector<float> Classes;

		Detection(const cv::Rect &InBoundingBox, float InConfidence, const std::vector<float> &InClasses)
			:BoundingBox(InBoundingBox), Confidence(InConfidence), Classes(InClasses)
		{}
	};
	std::string ModelName;
	std::vector<std::string> ClassNames;
	cv::dnn::Net network;
	std::filesystem::path GetNetworkPath(std::string extension = "") const;
	void loadNames();
	void loadNet();
	void Preprocess(const cv::UMat& frame, cv::Size inpSize, float scale, const cv::Scalar& mean, bool swapRB);
	std::vector<Detection> Postprocess(const std::vector<cv::Mat> &outputBlobs, const std::vector<std::string> &layerNames, cv::Rect window);
public:
	YoloDetect(std::string inModelName = "cdfr", int inNumclasses = 4);
	virtual ~YoloDetect();
	
	const std::string& GetClassName(int index) const;

	int GetNumClasses() const;

	int Detect(CameraImageData InData, CameraFeatureData *OutData);

	static std::vector<ObjectData> Project(const CameraImageData &ImageData, const CameraFeatureData& FeatureData);
};





void YoloTest(VideoCaptureCameraSettings CamSett);