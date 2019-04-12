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
#include <experimental/filesystem>

#include "Queue.hpp"
#include "Estimation.hpp"
#include "pipelineAlgo.hpp"
#include "Cartesian2Spherical.hpp"
#include "config.hpp"
#include "utils.hpp"
#include "log.hpp"

using namespace std;
using namespace cv;
namespace fs = std::experimental::filesystem;

// Thread-safe queue for communication between the generation of the images and the feature extraction
ScanVan::thread_safe_queue<Equirectangular> imgProcQueue {};

// Thread-safe queue for communication between the feature extraction and pose estimation
ScanVan::thread_safe_queue<TripletsWithMatches> tripletsProcQueue {};

//=========================================================================================================

void generatePairImages (Log *mt, Config *FC) {
// It reads the images from file and pushes to the queue for the feature extraction

	// list of file names of the input images
	std::vector<std::string> file_list{};

	// read the contents of the directory where the images are located
	fs::path pt = fs::u8path(FC->inputFolder);
	for (auto& p : fs::directory_iterator(pt)) {
		std::string str = p.path().u8string();
		if (str.substr(str.length()-3)=="bmp") {
			file_list.push_back(p.path().u8string());
		}
	}

	// sort the filenames alphabetically
	std::sort(file_list.begin(), file_list.end());

//	auto it1 = std::find(file_list.begin(),file_list.end(),"data_in/0_dataset/20181218-161153-843291.bmp");


	/*auto it = std::find(file_list.begin(),file_list.end(),inputFolder + "/" + inputDataSet + "/" + "20181218-161515-093294.bmp");
	file_list.erase(file_list.begin(), it);

	it = std::find(file_list.begin(),file_list.end(),inputFolder + "/" + inputDataSet + "/" + "20181218-162021-343307.bmp");
	file_list.erase(it, file_list.end());
*/

//	auto it2 = std::find(file_list.begin(),file_list.end(),"data_in/0_dataset/20181218-161314-343305.bmp");
//	file_list.erase(it, file_list.end());
	for (auto &n: file_list) {
		std::cout << n << std::endl;
	}
	std::cout << "Number of files considered: " << file_list.size() << std::endl;


	/*auto it1 = std::find(file_list.begin(), file_list.end(), "data_in/0_dataset/20181218-161314-343305.bmp");
	file_list.erase(file_list.begin(), it1);
	auto it2 = std::find(file_list.begin(), file_list.end(), "data_in/0_dataset/20181218-161530-093291.bmp");
	file_list.erase(it2, file_list.end());
	std::cout << "Number of files considered: " << file_list.size() << std::endl;
*/

	//file_list.erase(file_list.begin(),file_list.begin()+3);
	//file_list.erase(file_list.begin()+3, file_list.end());

	// counter for the image number
	int img_counter { 1 };

	for (auto &file: file_list) {

		// reads the image from the file
		cv::Mat input_image { };
		input_image = imread(file, cv::IMREAD_UNCHANGED);
		if (!input_image.data) {
			throw std::runtime_error("Could not load the input image");
		}

		// removes the folder name that precedes the path
		std::string fileName = file.substr(file.find_last_of("/", std::string::npos) + 1, std::string::npos);
		//std::cout << fileName << '\n';

		// creates a shared pointer from an anonymous object initialized with the image
		std::shared_ptr<Equirectangular> p1(new Equirectangular { input_image, img_counter, fileName });

		// simulates the delay of image acquisition
		//std::this_thread::sleep_for(std::chrono::milliseconds(1000));

		while (imgProcQueue.size()>10) {
			std::this_thread::sleep_for(1s);
		}

		// push to the queue
		imgProcQueue.push(p1);

		std::stringstream ss { };
		ss << "=========================" << std::endl
		   << "Send pair images " << p1->getImgNum() << std::endl
		   << "========================="
		   << std::endl;
		print(ss.str());

		img_counter++;
	}

	// signals the end of genPairs
	mt->terminateGenPairs = true;
}

//=========================================================================================================

void procFeatures (Log *mt, Config *FC) {

	std::deque<std::shared_ptr<EquirectangularWithFeatures>> v { };
	std::deque<std::shared_ptr<PairWithMatches>> lp { };

	// reads the mask to apply on the images
	//auto mask = make_shared<cv::Mat>(imread(inputFolder + "/" + inputMask + "/" + inputMaskFileName, IMREAD_GRAYSCALE));
	auto mask = make_shared<cv::Mat>(imread(FC->inputMask, IMREAD_GRAYSCALE));

	// If the mask was not loaded, throw an error
	if (! mask->data) {
		throw std::runtime_error("The image mask could not be read.");
	}

	// loop over while not terminate or the queue is not empty
	while ((!mt->terminateGenPairs)||(!imgProcQueue.empty())) {

		std::shared_ptr<Equirectangular> receivedPairImages { };
		receivedPairImages = imgProcQueue.wait_pop();

		std::stringstream ss {};
		ss << "=========================" << std::endl
		   << "Received equirectangular image " << receivedPairImages->getImgNum() << std::endl
		   << "=========================" << std::endl;
		print (ss.str());


		mt->start("1. Feature Extraction"); // measures the feature extraction
		////////////////////////////////////////////////////////////////////////////////////////////////////////
		std::shared_ptr<EquirectangularWithFeatures> featuredImages = extractFeatures(receivedPairImages, mask);
		////////////////////////////////////////////////////////////////////////////////////////////////////////
		mt->stop("1. Feature Extraction");  // measures the feature extraction

		//==========================================================================================
		// write into file the features
		FC->write_1_features(featuredImages);
		//==========================================================================================

		// v is a sort of queue where the extracted features are stored
		v.push_front(featuredImages);
		// whenever two sets of features are extracted, the matches between these sets are pushed to lp
		// and the last set of features is discarded
		if (v.size() == 2) {

			mt->start("2. Feature Matching Pairs"); // measures the feature matching of pairs
			////////////////////////////////////////////////////////////////////////////////////////////////////////
			lp.push_front(omniMatching(v[1], v[0]));
			////////////////////////////////////////////////////////////////////////////////////////////////////////
			mt->stop("2. Feature Matching Pairs"); // measures the feature matching of pairs

			//==========================================================================================
			// write the matched features for each pair of images
			FC->write_2_matches(lp.front());
			//==========================================================================================

			v.pop_back();
		}

		// lp is a sort of queue where the matches are stored
		// whenever there are two sets of matches, the triplets are computed
		if (lp.size() == 2) {

			// calculates the keypoints common on two pairs of images
			// in this case it creates a triplet

			mt->start("3. Common Points Computation - Triplets"); // measures the common points computation
			////////////////////////////////////////////////////////////////////////////////////////////////////////
			std::shared_ptr<TripletsWithMatches> p1 = commonPointsComputation(lp[1], lp[0]);
			////////////////////////////////////////////////////////////////////////////////////////////////////////
			mt->stop("3. Common Points Computation - Triplets"); // measures the common points computation

			// push the triplet to the thread-safe queue and send it for pose estimation
			while (tripletsProcQueue.size() > 5) {
				std::this_thread::sleep_for(1s);
			}

			tripletsProcQueue.push(p1);

			//==========================================================================================
			// write the matched features for two consecutive pair of images, i.e. triplets
			FC->write_3_triplets(p1);
			//==========================================================================================

			lp.pop_back();

		}
	}

	// signals the end of procFeatures
	mt->terminateProcFeatures = true;
}

//========================================================================================================

void ProcPose (Log *mt, Config *FC) {

	//std::vector<Model> vecm { };

	// the main model where all the models will be superposed
	Model m { };
	size_t counter { 0 };
	RNG rng(12345);

	while((!mt->terminateProcFeatures)||(!tripletsProcQueue.empty())) {

		// gets the triplets from procFeatures
		std::shared_ptr<TripletsWithMatches> receivedTripletsImages { };
		receivedTripletsImages = tripletsProcQueue.wait_pop();

		// print message
		std::stringstream ss { };
		ss << "=========================" << std::endl
		   << "Received triplets images " << "(" << receivedTripletsImages->getImageNumber1() << ", " << receivedTripletsImages->getImageNumber2() << ", " << receivedTripletsImages->getImageNumber3() << ")" << std::endl
   		   << "=========================" << std::endl;
		print(ss.str());

		// vector containing the spherical coordinates
		std::vector<Vec_Points<double>> p3d_liste { };



		// width and height of the images
		auto width = receivedTripletsImages->getImage()[0]->getOmni()->getImage().cols;
		auto height = receivedTripletsImages->getImage()[0]->getOmni()->getImage().rows;

		mt->start("4. Conversion to Spherical - Triplets"); // measures the conversion time to spherical
		// loop over the spheres
		for (int idx { 0 }; idx < 3; ++idx) {

			Vec_Points<double> list_matches { };

			// if there are matches then convert them to spherical
			if (receivedTripletsImages->getMatchVector().size() != 0) {
				for (const auto &match : receivedTripletsImages->getMatchVector()) {
					// match is a vector of indices of the keypoints
					// for triplets its size is 3

					// gets the xy coordinate of the keypoint corresponding to sphere idx
					auto keypoint = receivedTripletsImages->getImage()[idx]->getKeyPoints()[match[idx]].pt;

					// convert x and y coordinates to spherical
					Points<double> p = convertCartesian2Spherical(static_cast<double>(keypoint.x), static_cast<double>(keypoint.y), width, height);

					// push_back the spherical coordinate into list_matches
					list_matches.push_back(p);
				}
			}

			p3d_liste.push_back(list_matches);
		}
		mt->stop("4. Conversion to Spherical - Triplets"); // measures the conversion time to spherical

		// copy of the original vector containing the spherical coordinates
		std::vector<Vec_Points<double>> p3d_liste_orig {p3d_liste};

		//==========================================================================================
		// output in a file the spherical coordinates of the triplet
		FC->write_4_spherical(receivedTripletsImages, p3d_liste);
		//==========================================================================================

		Vec_Points<double> sv_scene { };
		std::vector<Points<double>> positions { Points<double>(), Points<double>(), Points<double>() };
		std::vector<Points<double>> sv_t_liste { Points<double>(), Points<double>()};
		std::vector<Mat_33<double>> sv_r_liste { Mat_33<double>(), Mat_33<double>()};

		double error_max { 1e-8 };

		//-----------------------------------------------------
		// call to pose estimation algorithm
		int initialNumberFeatures = p3d_liste[0].size();

		int numIter {};

		// Only compute if there are features in the vector
		if (initialNumberFeatures!=0) {
			mt->start("5. Pose Estimation"); // measures the time of pose estimation algorithm

			for (int i = 0; i < 1; ++i) {
				numIter = pose_estimation(p3d_liste, error_max, sv_scene, positions, sv_r_liste, sv_t_liste);

				//filter_keypoints(p3d_liste, sv_scene, positions, p3d_liste);

				std::cout << "Number of iterations pose estimation : " << numIter << std::endl;
			}

			mt->stop("5. Pose Estimation"); // measures the time of pose estimation algorithm
		}

		int finalNumberFeatures = p3d_liste[0].size();
		//-----------------------------------------------------

		//==========================================================================================
		// output in a file the rotation matrix and translation vector and statistics of the pose estimation algorithm
		FC->write_5_pose_3(receivedTripletsImages, sv_r_liste, sv_t_liste, numIter, initialNumberFeatures, finalNumberFeatures);
		//==========================================================================================

		//==========================================================================================
		// output in a file of the sparse point cloud of the triplet
		FC->write_6_sparse_3 (receivedTripletsImages, sv_scene);
		//==========================================================================================

		//==========================================================================================
		// write the matched features of the triplets that are filtered
		FC->write_3_triplets_filtered (receivedTripletsImages, p3d_liste_orig, p3d_liste);
		//==========================================================================================


		// The new model to add
		Model m2 { };

		// put the names of the images into the model
		m2.imgNames.push_back(receivedTripletsImages->getImageName1());
		m2.imgNames.push_back(receivedTripletsImages->getImageName2());
		m2.imgNames.push_back(receivedTripletsImages->getImageName3());

		// the position of the center of the triplets
		cv::Matx13f modelCenter(0, 0, 0);

		// calculates the mean position of the triplets
		for (size_t i { 0 }; i < positions.size(); ++i) {
			Points<double> f = positions[i];
			modelCenter = modelCenter + Matx13f(f[0], f[1], f[2]);
		}
		modelCenter = 1.0 / positions.size() * modelCenter;

		// calculates the average distance of the reconstructed points with respect to the center of the triplets
		double averageDistance = 0;
		for (size_t i { 0 }; i < sv_scene.size(); ++i) {
			Points<double> f = sv_scene[i];
			averageDistance += norm(modelCenter - Matx13f(f[0], f[1], f[2]));
		}
		averageDistance /= sv_scene.size();

		// assigns a random color to the model to add
		RGB888 modelColor = RGB888(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));

		// copies to features vector of the model the points whose average distance is relatively close
		for (size_t i { 0 }; i < sv_scene.size(); ++i) {
			Points<double> f = sv_scene[i];
			/*if (norm(Matx13f(f[0], f[1], f[2]) - modelCenter) > averageDistance)
				continue;*/
			//m2.features.push_back(ModelFeature(1000 * Matx13f(f[0], f[1], f[2]), modelColor));  //RGB888(255,255, counter * 0x40)
			m2.features.push_back(ModelFeature(Matx13f(f[0], f[1], f[2]), modelColor));  //RGB888(255,255, counter * 0x40)
		}

		// copies to camera positions vector the positions of the spheres
		for (size_t i { 0 }; i < positions.size(); ++i) {
			//Points<double> f = positions[i];
			// m2, i.e. the new model of the triplet contains the positions of the camera where the first position is (0,0,0)
			// and the rotation matrices, the first rotation matrix is eye(3,3), the second sv_r_liste[0], i.e. r12, and the third sv_r_liste[1], i.e. r23
			if (i==0) {
				m2.viewPoints.push_back(ModelViewPoint(cv::Matx13f::zeros(), cv::Matx33f::eye(), cv::Matx33f::eye()));
			} else if (i > 0) {
				Mat_33<double> & m1 = sv_r_liste[i - 1];
				Points<double> f = positions[i];
				// the relative rotation of the camera position with respect to the previous camera position
				cv::Matx33d mat1 { m1[0][0], m1[0][1], m1[0][2], m1[1][0], m1[1][1], m1[1][2], m1[2][0], m1[2][1], m1[2][2] };
				// the absolute rotation of the camera position with respect to the first camera position of the triplet
				cv::Matx33d mat2 = (m2.viewPoints[i-1].rotationAbsolute) * mat1.t();
				m2.viewPoints.push_back(ModelViewPoint(cv::Matx13d(f[0], f[1], f[2]), mat1, mat2));
			}
		}

		/*// writes the estimated triplet model
		std::string plyFileName { "./resources/models_est_" + std::to_string(counter) + ".ply" };
		writePly(plyFileName, m2.features);*/

		//-----------------------------------------------------
		fusionModel2(&m, &m2, 2);
		//-----------------------------------------------------

		//==========================================================================================
		// output in a file of the absolute rotation and translation matrices
		FC->write_7_odometry (m);
		//==========================================================================================

		//==========================================================================================
		// output in a file of the progressively merged models
		FC->write_8_progressiveModel(m);
		//==========================================================================================

		//std::string fusionFileName { "./resources/models_fusion_" + std::to_string(counter) + ".ply" };
		//writePly(fusionFileName, m.features);

		counter++;
	}
	//==========================================================================================
	// output in a file of the merged model
	FC->write_9_finalModel(m);
	//==========================================================================================


	// signals the end of pose estimation
	mt->terminateProcPose = true;
}


void RunAllPipeline (Config *FC) {

	Log mt{};

	// Process configuration file
	//ProcessConfigFile(cfg);
	// check if folders for writing the results exist
	FC->CheckFolders();

	std::thread GenPairs (generatePairImages, &mt, FC);
	std::thread ProcessFeatureExtraction (procFeatures, &mt, FC);
	std::thread ProcessPoseEstimation (ProcPose, &mt, FC);

	GenPairs.join();
	ProcessFeatureExtraction.join();
	ProcessPoseEstimation.join();

	mt.listRunningTimes();

}


int main(int argc, char* argv[]) {

	if (argc < 2) {
		std::cerr << "Usage: " << argv[0] << " config_file.txt" << std::endl;
		return 1;
	}

	std::string cfg = argv[1];

	Config FC (cfg);

	if (FC.execType == Config::RUN_ALL) {
		RunAllPipeline(&FC);
	}

	return 0;

}






