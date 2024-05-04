#include "DetectFeatures/YoloDetect.hpp"

#include <iostream>
#include <optional>
#include <fstream>
#include <chrono>
#include <array>

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#ifdef WITH_CORAL
#include <edgetpu.h>
#include <edgetpu_c.h>
#include <tensorflow/lite/interpreter.h>
#include <tensorflow/lite/interpreter_builder.h>
#include <tensorflow/lite/op_resolver.h>
#include <tensorflow/lite/core/kernels/register.h>
#endif

#include <Misc/GlobalConf.hpp>
#include <Misc/math3d.hpp>

using namespace std;
using namespace cv;

const Size modelSize(640,480);

YoloDetect::YoloDetect(string inModelName, int inNumclasses)
	:ModelName(inModelName)
{
	ClassNames.resize(inNumclasses);
	loadNames();
	loadNet();
}

YoloDetect::~YoloDetect()
{
}

filesystem::path YoloDetect::GetNetworkPath(string extension) const
{
	return GetAssetsPath()/"dnn"/(ModelName+extension);
}

void YoloDetect::loadNames()
{
	ifstream NamesFile(GetNetworkPath(".names"));
	for (size_t i = 0; i < ClassNames.size(); i++)
	{
		NamesFile >> ClassNames[i];
		cout << "Yolo class " << i << " is " << ClassNames[i] << endl;
	}
}

void YoloDetect::loadNet()
{
	auto backends = dnn::getAvailableBackends();
	cout << "Listing available backends and targets" << endl;
	for (auto &&backend : backends)
	{
		cout << "Backend " << backend.first << " is available with target " << backend.second << endl;
	}
	network = dnn::readNetFromDarknet(GetNetworkPath(".cfg"), GetNetworkPath(".weights"));
}

void YoloDetect::Preprocess(const UMat& frame, Size inpSize, float scale, const Scalar& mean, bool swapRB)
{
	Mat blob;
	// Create a 4D blob from a frame.
	if (inpSize.width <= 0) inpSize.width = frame.cols;
	if (inpSize.height <= 0) inpSize.height = frame.rows;
	dnn::blobFromImage(frame, blob, 1.0, inpSize, Scalar(), swapRB, false);
	//cout << "Input has size " << blob.size << endl;
	// Run a model.
	network.setInput(blob, "", scale, mean);
}


vector<YoloDetect::Detection> YoloDetect::Postprocess(const vector<Mat> &outputBlobs, const vector<string> &layerNames, Rect window)
{
	vector<Rect> boxes;
	vector<float> scores;
	vector<vector<float>> classes;
	int numclasses = ClassNames.size();
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
			classesloc.resize(numclasses);
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


const string& YoloDetect::GetClassName(int index) const
{
	return ClassNames[index];
}

int YoloDetect::GetNumClasses() const
{
	return ClassNames.size();
}

int YoloDetect::Detect(CameraImageData InData, CameraFeatureData *OutData)
{
	Preprocess(InData.Image, modelSize, 1.0/255.0, 0, true);
	auto start = chrono::steady_clock::now();
	vector<Mat> outputBlobs;
	auto OutputNames = network.getUnconnectedOutLayersNames();
	network.forward(outputBlobs, OutputNames);
	auto stop = chrono::steady_clock::now();
	auto detections = Postprocess(outputBlobs, OutputNames, Rect(0,0,InData.Image.cols, InData.Image.rows));
	int numdetections = detections.size();
	OutData->YoloDetections.clear();
	OutData->YoloDetections.reserve(numdetections);
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
		OutData->YoloDetections.push_back(final_detection);
	}
	(void) start; (void) stop;
	//cout << "Inference took " << chrono::duration<double>(stop-start).count() << "s and found " << numdetections << " objects" << endl;
	return numdetections;
}

static_assert(sizeof(Matx31d) == sizeof(Vec3d));
vector<ObjectData> YoloDetect::Project(const CameraImageData &ImageData, const CameraFeatureData& FeatureData)
{
	vector<ObjectData> objects;
	objects.reserve(FeatureData.YoloDetections.size());
	const Matx33d InvCameraMatrix = Mat(ImageData.CameraMatrix.inv());
	for (size_t i = 0; i < FeatureData.YoloDetections.size(); i++)
	{
		auto &Detection = FeatureData.YoloDetections[i];
		//auto ROI = ImageData.Image(Detection.Corners);
		auto center = (Detection.Corners.tl() + Detection.Corners.br())/2.0;
		Matx31d position = {center.x, center.y, 1};
		Matx31d vector(InvCameraMatrix * position); //TODO: This doesn't take into account distortion coeffs
		Matx31d WorldVector = FeatureData.CameraTransform.rotation() * vector;
		Vec3d WorldPosition = LinePlaneIntersection(FeatureData.CameraTransform.translation(), 
			*reinterpret_cast<Vec3d*>(&WorldVector), Vec3d(0,0,0.02), Vec3d(0,0,1));
		WorldPosition[2] = 0;
		ObjectType type = (ObjectType)((int)ObjectType::Fragile + Detection.Class);
		const auto &name = ObjectTypeNames.at(type);
		ObjectData object(type, name, 
			Affine3d(Vec3d::all(0), WorldPosition));
		objects.emplace_back(object);
		//imshow("Yolo ROI", ROI);
		//waitKey(10);
	}
	return objects;
}

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
