#include "DetectFeatures/YoloDetect.hpp"

#include <iostream>
#include <optional>
#include <fstream>
#include <chrono>
#include <array>

#include <opencv2/imgproc.hpp>
#include <opencv2/dnn.hpp>

#ifdef WITH_CORAL
#include <edgetpu.h>
#include <edgetpu_c.h>
#include <tensorflow/lite/interpreter.h>
#include <tensorflow/lite/interpreter_builder.h>
#include <tensorflow/lite/op_resolver.h>
#include <tensorflow/lite/core/kernels/register.h>
#endif

#include <Misc/GlobalConf.hpp>

using namespace std;
using namespace cv;

const int numclasses = 4;
const Size modelSize(640,480);

array<string, numclasses> ClassNames;

const string& GetYoloClassName(int index)
{
	return ClassNames[index];
}

int GetYoloNumClasses()
{
	return numclasses;
}

namespace OpenCVDNN
{
	optional<dnn::Net> YoloNet;

	void LoadNames()
	{
		ifstream NamesFile(GetAssetsPath()/"dnn"/"cdfr.names");
		for (size_t i = 0; i < numclasses; i++)
		{
			NamesFile >> ClassNames[i];
			cout << "Yolo class " << i << " is " << ClassNames[i] << endl;
		}
	}

	void LoadNet()
	{
		if (YoloNet.has_value())
		{
			return;
		}
		LoadNames();
		auto backends = dnn::getAvailableBackends();
		cout << "Listing available backends and targets" << endl;
		for (auto &&backend : backends)
		{
			cout << "Backend " << backend.first << " is available with target " << backend.second << endl;
		}
		//YoloNet = dnn::readNet(GetAssetsPath() / "dnn" / "run2_float32.onnx");
		auto dnnpath = GetAssetsPath() / "dnn";
		YoloNet = dnn::readNetFromDarknet(dnnpath/"cdfr.cfg", dnnpath/"cdfr.weights");
	}

	void Preprocess(const UMat& frame, dnn::Net& net, Size inpSize, float scale, const Scalar& mean, bool swapRB)
	{
		static Mat blob;
		// Create a 4D blob from a frame.
		if (inpSize.width <= 0) inpSize.width = frame.cols;
		if (inpSize.height <= 0) inpSize.height = frame.rows;
		dnn::blobFromImage(frame, blob, 1.0, inpSize, Scalar(), swapRB, false);
		//cout << "Input has size " << blob.size << endl;
		// Run a model.
		net.setInput(blob, "", scale, mean);
	}


	struct Detection
	{
		Rect BoundingBox;
		float Confidence;
		array<float, numclasses> Classes;

		Detection(const Rect &InBoundingBox, float InConfidence, const array<float, numclasses> &InClasses)
			:BoundingBox(InBoundingBox), Confidence(InConfidence), Classes(InClasses)
		{}
	};


	vector<Detection> Postprocess(vector<Mat> outputBlobs, vector<string> layerNames, Rect window)
	{
		vector<Rect> boxes;
		vector<float> scores;
		vector<array<float, numclasses>> classes;
		for (size_t blobidx = 0; blobidx < outputBlobs.size(); blobidx++)
		{
			auto& blob = outputBlobs[blobidx];
			auto& layername = layerNames[blobidx];
			(void) layername;
			int numdet = 1;
			for (int i = 0; i < blob.size.dims()-1; i++)
			{
				numdet*=blob.size[i];
			}
			int numelem = blob.size[blob.size.dims()-1];
			assert(numelem == numclasses +4 +1); //x, y, width, height, confidence, classes...
			//cout << "Layer " << layername << " has " << numdet << " detections and " << numelem << " elements/detection" << endl;
			size_t newnum = boxes.size() + numdet;
			boxes.reserve(newnum);
			scores.reserve(newnum);
			classes.reserve(newnum);
			int stride = blob.elemSize1() * numelem;
			assert(blob.elemSize1() == sizeof(float));
			//cout << "stride is " << stride << " bytes/detection" << endl;
			for (const uint8_t* ptr = blob.data; ptr < blob.dataend; ptr+=stride)
			{
				auto recast = reinterpret_cast<const float*>(ptr);
				float cx = recast[0]*window.width+window.x;
				float cy = recast[1]*window.height+window.y;
				float w = recast[2]*window.width;
				float h = recast[3]*window.height;
				boxes.emplace_back(cx-w/2, cy-h/2, w, h);
				scores.emplace_back(recast[4]);
				auto& classesloc = classes.emplace_back();
				copy(&recast[5], &recast[numelem], classesloc.data());
			}
		}
		vector<int> keptIndices;
		dnn::NMSBoxes(boxes, scores, 0.4, 0.5, keptIndices);
		vector<Detection> OutDetections;
		OutDetections.reserve(keptIndices.size());
		for (int kept : keptIndices)
		{
			OutDetections.emplace_back(boxes[kept], scores[kept], classes[kept]);
		}
		return OutDetections;
	}

	int DetectYolo(const CameraImageData &InData, CameraFeatureData& OutData)
	{
		LoadNet();


		Preprocess(InData.Image, YoloNet.value(), modelSize, 1.0/255.0, 0, true);
		//auto start = chrono::steady_clock::now();
		vector<Mat> outputBlobs;
		auto OutputNames = YoloNet->getUnconnectedOutLayersNames();
		YoloNet->forward(outputBlobs, OutputNames);
		//auto stop = chrono::steady_clock::now();
		//cout << "Inference took " << chrono::duration<double>(stop-start).count() << " s" << endl;
		auto detections = Postprocess(outputBlobs, OutputNames, Rect(0,0,InData.Image.cols, InData.Image.rows));
		Mat painted; InData.Image.copyTo(painted);
		int numdetections = detections.size();
		OutData.YoloDetections.clear();
		OutData.YoloDetections.reserve(numdetections);
		for (auto &det : detections)
		{
			int maxidx = 0;
			for (size_t i = 1; i < det.Classes.size(); i++)
			{
				if (det.Classes[i] > det.Classes[maxidx])
				{
					maxidx = i;
				}
			}
			YoloDetection final_detection;
			//cout << "Found " << maxidx << " at " << det.BoundingBox << " (Confidence " << det.Confidence << ")" << endl;
			final_detection.Class = maxidx;
			final_detection.Confidence = det.Confidence;
			final_detection.Corners = det.BoundingBox;
			OutData.YoloDetections.push_back(final_detection);
		}
		return numdetections;
	}
} // namespace OpenCVDNN

#ifdef WITH_CORAL
namespace EdgeDNN
{
	bool EdgeLoaded = false, EdgeOK = false;
	shared_ptr<edgetpu::EdgeTpuContext> context;
	unique_ptr<tflite::FlatBufferModel> model;
	unique_ptr<tflite::Interpreter> interpreter;

	std::string ToString(edgetpu_device_type type) {
		switch (type) {
			case EDGETPU_APEX_PCI:
			return "PCI";
			case EDGETPU_APEX_USB:
			return "USB";
		}
		return "Unknown";
	}

	bool BuildEdgeTpuInterpreter() {
		tflite::ops::builtin::BuiltinOpResolver resolver;
		resolver.AddCustom(edgetpu::kCustomOp, edgetpu::RegisterCustomOp());
		auto builder = tflite::InterpreterBuilder(*model, resolver);
		if (builder(&interpreter) != kTfLiteOk) {
			std::cerr << "Failed to build interpreter." << std::endl;
			return false;
		}
		// Bind given context with interpreter.
		interpreter->SetExternalContext(TfLiteExternalContextType::kTfLiteEdgeTpuContext, (TfLiteExternalContext*) context.get());
		interpreter->SetNumThreads(1);
		if (interpreter->AllocateTensors() != kTfLiteOk) {
			std::cerr << "Failed to allocate tensors." << std::endl;
			return false;
		}
		return true;
	}

	bool LoadNet()
	{
		if (EdgeLoaded)
		{
			return EdgeOK;
		}
		EdgeLoaded = true;

		if(1)
		{
			size_t num_edges = 0;
			std::unique_ptr<edgetpu_device, decltype(&edgetpu_free_devices)> devices(edgetpu_list_devices(&num_edges), &edgetpu_free_devices);
			for (size_t i = 0; i < num_edges; i++)
			{
				auto device = devices.get()[i];
				cout << "Found Coral @" << device.path << " over " << ToString(device.type) << endl;
			}
		}
		
		auto manager = edgetpu::EdgeTpuManager::GetSingleton();
		if (manager == nullptr)
		{
			cerr << "Failed to open EdgeTPU Manager" << endl;
			return false;
		}
		manager->SetVerbosity(1);

		context = manager->OpenDevice();
		if (context == nullptr)
		{
			cerr << "Failed to open EdgeTPU Device" << endl;
			return false;
		}
		
		const auto model_path = GetAssetsPath() / "dnn" / "edgetpu.tflite";
		model = tflite::FlatBufferModel::BuildFromFile(model_path.c_str());
		if (model == nullptr)
		{
			cerr << "Failed to load model for Coral" << endl;
			return false;
		}
		

		if(!BuildEdgeTpuInterpreter())
		{
			cerr << "Failed to build EdgeTPU interpreter" << endl;
			return false;
		}
		EdgeOK = true;
		return true;
	}

} // namespace EdgeDNN
#endif


int DetectYolo(const CameraImageData &InData, CameraFeatureData& OutData)
{
	#ifdef WITH_CORAL
	if(EdgeDNN::LoadNet())
	{

	}
	#else
	return OpenCVDNN::DetectYolo(InData, OutData);
	#endif
}