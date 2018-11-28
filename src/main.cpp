//============================================================================
// Name        : Processor.cpp
// Author      : Marcelo E. Kaihara
// Version     :
// Copyright   : 
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <iostream>
#include <thread>
#include <mutex>
#include <sstream>
#include <iostream>
#include <fstream>
#include <iterator>
#include <vector>
#include <opencv4/opencv2/core/types.hpp>
#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/features2d.hpp>
#include <opencv4/opencv2/imgcodecs.hpp>
#include <opencv4/opencv2/opencv.hpp>
#include <math.h>

#include "ctpl.hpp"
#include "Queue.hpp"
#include "Estimation.hpp"
#include "pipelineAlgo.hpp"

using namespace std;
using namespace cv;

// Thread-safe queue for communication between the generation of the images and the feature extraction
ScanVan::thread_safe_queue<Omni> imgProcQueue {};

// Thread-safe queue for communication between the feature extraction and pose estimation
ScanVan::thread_safe_queue<TripletsWithMatches> tripletsProcQueue {};

ctpl::thread_pool p(4 /* two threads in the pool */);

//=========================================================================================================

void generatePairImages () {

	const int staticImagesCount { 6 };

	cv::Mat staticImages[staticImagesCount] { };

	for (int i { 0 }; i < staticImagesCount; ++i) {
		std::stringstream ss { };
		ss << "./resources/0_" << (i + 1) << ".bmp";
		staticImages[i] = imread(ss.str(), IMREAD_UNCHANGED);
		cout << ss.str() << endl;
	}

	int i { 0 };

	for (;;) {

		std::shared_ptr<Omni> p1(new Omni { i });

		// This is to make the index increment and decrement in a saw-like fashion
		// staticImageId will take values 0, 1, 2, 3, 4, 5, 4, 3, 2, 1, 0, 1, 2, 3, etc
		int staticImageId { i % (staticImagesCount * 2 - 1 - 1) };
		if (staticImageId >= staticImagesCount)
			staticImageId = staticImagesCount * 2 - 1 - staticImageId - 1;

		p1->img = staticImages[staticImageId];

		// simulates the delay of image acquisition
		std::this_thread::sleep_for(std::chrono::milliseconds(1000));

		// push to the queue
		imgProcQueue.push(p1);

		std::stringstream ss { };
		ss << "=========================" << std::endl
		   << "Send pair images " << p1->idString() << std::endl
		   << "=========================" << std::endl;
		print(ss.str());

		++i;
	}
}

//=========================================================================================================

void procFeatures() {

	std::deque<std::shared_ptr<OmniWithFeatures>> v { };
	std::deque<std::shared_ptr<PairWithMatches>> lp { };

	// reads the mask to apply on the images
	auto mask = make_shared<Mat>(imread("resources/mask0.png", IMREAD_GRAYSCALE));

//	auto filterPath = vector<const char*>({"resources/0_0.bmp"});
//  vector<KeyPoint> filtersKpts; //Keypoints extracted from img.
//    Mat filtersDesc;
//	for(auto ip : filterPath){
//		auto filterImg = imread("resources/mire.bmp", IMREAD_UNCHANGED);
//	    Ptr<AKAZE> akaze = AKAZE::create(
//			AKAZE::DESCRIPTOR_MLDB,
//			0,  3,
//			0.0001f,  4,
//			4, KAZE::DIFF_PM_G2
//		);
//	    akaze->detectAndCompute(filterImg, Mat(), filtersKpts, filtersDesc);
//	}

	for (;;) {

		std::shared_ptr<Omni> receivedPairImages { };
		receivedPairImages = imgProcQueue.wait_pop();

		std::stringstream ss {};
		ss << "=========================" << std::endl
		   << "Received pair images " << receivedPairImages->idString() << std::endl
		   << "=========================" << std::endl;
		print (ss.str());

		auto featuredImages = extractFeatures(receivedPairImages, mask);

		// v is a sort of queue where the extracted features are stored
		v.push_front(featuredImages);
		// whenever two sets of features are extracted, the matches between these sets are pushed to lp
		// and the last set of features is discarded
		if (v.size() == 2) {
			lp.push_front(omniMatching(v[1], v[0]));
			v.pop_back();
		}

		// lp is a sort of queue where the matches are stored
		// whenever there are two sets of matches, the triplets are computed
		if (lp.size() == 2) {
			std::shared_ptr<TripletsWithMatches> p1 = commonPointsComputation(lp[1], lp[0]);
			lp.pop_back();
			// push the triplet to the thread-safe queue and send it for pose estimation
			tripletsProcQueue.push(p1);
		}
	}
}

//=========================================================================================================

void ProcPose() {

	std::vector<Model> vecm { };
	Model m { };
	size_t counter { 0 };
	RNG rng(12345);

	for (;;) {

		// gets the triplets from procFeatures
		std::shared_ptr<TripletsWithMatches> receivedTripletsImages { };
		receivedTripletsImages = tripletsProcQueue.wait_pop();

		std::vector<Vec_Points<double>> p3d_liste;

		// width and height of the images
		auto width = receivedTripletsImages->imgs[0]->omni->img.cols;
		auto height = receivedTripletsImages->imgs[0]->omni->img.rows;

		for (int idx { 0 }; idx < 3; ++idx) {
			Vec_Points<double> list_matches { };

			for(auto match : receivedTripletsImages->matches){
				// gets the positions of the features
				auto xy = receivedTripletsImages->imgs[idx]->kpts[match[idx]].pt;

				/* convert x and y to spherical */
				double theta { (xy.x / width) * 2 * M_PI };
				double phi { M_PI / 2 - (xy.y / (height - 1)) * M_PI };
				double x { -cos(phi) * cos(theta) };
				double y { cos(phi) * sin(theta) };
				double z { sin(phi) };
				Points<double> p { x, y, z };
				list_matches.push_back(p);
			}

			std::string plyFileNameOri { "./resources/models_ori_" + std::to_string(idx) + "_" + std::to_string(counter) + ".ply" };
			writePly(plyFileNameOri, list_matches);

			p3d_liste.push_back(list_matches);
		}

		Vec_Points<double> sv_scene { };
		std::vector<Points<double>> positions { };

		double error_max { 1e-8 };

		// call to pose estimation algorithm
		pose_estimation (p3d_liste, error_max, sv_scene, positions);

		Model m2 { };
		Matx13f modelCenter(0, 0, 0);

		for (size_t i { 0 }; i < positions.size(); ++i) {
			Points<double> f = positions[i];
			modelCenter = modelCenter + Matx13f(f[0], f[1], f[2]);
		}

		modelCenter = 1.0 / positions.size() * modelCenter;
		double averageDistance = 0;
		for (size_t i { 0 }; i < sv_scene.size(); ++i) {
			Points<double> f = sv_scene[i];
			averageDistance += norm(modelCenter - Matx13f(f[0], f[1], f[2]));
		}
		averageDistance /= sv_scene.size();
		RGB888 modelColor = RGB888(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));
		for (size_t i { 0 }; i < sv_scene.size(); ++i) {
			Points<double> f = sv_scene[i];
			if (norm(Matx13f(f[0], f[1], f[2]) - modelCenter) > 10 * averageDistance)
				continue;
			m2.features.push_back(ModelFeature(1000 * Matx13f(f[0], f[1], f[2]), modelColor));  //RGB888(255,255, counter * 0x40)
		}
		for (size_t i { 0 }; i < positions.size(); ++i) {
			Points<double> f = positions[i];
			m2.keypoints.push_back(ModelKeypoint(1000 * Matx13f(f[0], f[1], f[2])));
		}
		std::string plyFileName { "./resources/models_est_" + std::to_string(counter) + ".ply" };
		writePly(plyFileName, m2.features);


		std::stringstream ss {};
		ss << "=========================" << std::endl
		   << "Received triplets images " << receivedTripletsImages->idString() << std::endl
		   << "=========================" << std::endl;
		print (ss.str());

		fusionModel(&m, &m2);
		std::string fusionFileName { "./resources/models_fusion_" + std::to_string(counter) + ".ply" };
		writePly(fusionFileName, m.features);
		counter++;
	}
}


int main() {


	std::thread GenPairs (generatePairImages);
	std::thread ProcessFeatureExtraction (procFeatures);
	std::thread ProcessPoseEstimation (ProcPose);

	GenPairs.join();
	ProcessFeatureExtraction.join();
	ProcessPoseEstimation.join();

	return 0;

}






