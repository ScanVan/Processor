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

#include "ctpl.hpp"
#include "Queue.hpp"
#include "Estimation.hpp"
#include "pipelineAlgo.hpp"
#include "Cartesian2Spherical.hpp"
#include "config.hpp"

using namespace std;
using namespace cv;
namespace fs = std::experimental::filesystem;

// Thread-safe queue for communication between the generation of the images and the feature extraction
ScanVan::thread_safe_queue<Equirectangular> imgProcQueue {};

// Thread-safe queue for communication between the feature extraction and pose estimation
ScanVan::thread_safe_queue<TripletsWithMatches> tripletsProcQueue {};

ctpl::thread_pool p(4 /* two threads in the pool */);

class MeasureTime {

public:

	bool terminateGenPairs = false;
	bool terminateProcFeatures = false;
	bool terminateProcPose = false;

	long int number_gen_pairs { 0 };
	std::chrono::duration<double> total_duration_gen_pairs { 0 };
	double get_avg_gen_pairs() {
		return total_duration_gen_pairs.count() / number_gen_pairs * 1000.0;
	}

	long int number_feature_extract { 0 };
	std::chrono::duration<double> total_duration_feature_extract { 0 };
	double get_avg_feature_extract() {
		return total_duration_feature_extract.count() / number_feature_extract * 1000.0;
	}

	long int number_pose_estimation { 0 };
	std::chrono::duration<double> total_duration_pose_estimation { 0 };
	double get_avg_pose_estimation() {
		return total_duration_pose_estimation.count() / number_pose_estimation * 1000.0;
	}

	long int number_fusion { 0 };
	std::chrono::duration<double> total_duration_fusion { 0 };
	double get_avg_fusion() {
		return total_duration_fusion.count() / number_fusion * 1000.0;
	}

	long int number_common_p { 0 };
	std::chrono::duration<double> total_duration_common_p { 0 };
	double get_avg_common_p() {
		return total_duration_common_p.count() / number_common_p * 1000.0;
	}

	long int number_omni_matching { 0 };
	std::chrono::duration<double> total_duration_omni_matching { 0 };
	double get_avg_omni_matching() {
		return total_duration_omni_matching.count() / number_omni_matching * 1000.0;
	}

};

//=========================================================================================================

void generatePairImages (MeasureTime *mt) {
// It reads the images from file and pushes to the queue for the feature extraction

	// list of file names of the input images
	std::vector<std::string> file_list{};

	// read the contents of the directory where the images are located
	fs::path pt = fs::u8path(inputFolder + "/" + inputDataSet);
	for (auto& p : fs::directory_iterator(pt))
		file_list.push_back(p.path().u8string());

	// sort the filenames alphabetically
	std::sort(file_list.begin(), file_list.end());

//	// for test purpose only
//	file_list.erase(file_list.begin()+3, file_list.end());

	// counter for the image number
	int img_counter { 1 };

	for (auto &file: file_list) {

//		if (img_counter > 68)  {

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

			// push to the queue
			imgProcQueue.push(p1);

			std::stringstream ss { };
			ss << "=========================" << std::endl << "Send pair images " << p1->getImgNum() << std::endl << "========================="
					<< std::endl;
			print(ss.str());

//		}

		img_counter++;
	}

	// signals the end of genPairs
	mt->terminateGenPairs = true;
}

//=========================================================================================================

void procFeatures (MeasureTime *mt) {

	std::deque<std::shared_ptr<EquirectangularWithFeatures>> v { };
	std::deque<std::shared_ptr<PairWithMatches>> lp { };

	// reads the mask to apply on the images
	auto mask = make_shared<cv::Mat>(imread(inputFolder + "/" + inputMask + "/" + inputMaskFileName, IMREAD_GRAYSCALE));

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

		////////////////////////////////////////////////////////////////////////////////////////////////////////
		std::shared_ptr<EquirectangularWithFeatures> featuredImages = extractFeatures(receivedPairImages, mask);
		////////////////////////////////////////////////////////////////////////////////////////////////////////

		//==========================================================================================
		// write into file the features
		// check if folder exists, if not create it
		if (!fs::exists(outputFolder + "/" + outputFeatures)) {
			fs::create_directory(outputFolder + "/" + outputFeatures);
		}

		std::string pathOutputFeature = outputFolder + "/" + outputFeatures + "/" + featuredImages->getImgName();

		// write features extracted for each image into files
		// open the file to write the features
		std::ofstream outputFile { pathOutputFeature };
		// go over all the features extracted and write them into the file
		for (const auto kp : featuredImages->getKeyPoints()) {
			outputFile << std::setprecision(15) << kp.pt.x << " " << kp.pt.y << std::endl;
		}
		outputFile.close();
		//==========================================================================================

		// v is a sort of queue where the extracted features are stored
		v.push_front(featuredImages);
		// whenever two sets of features are extracted, the matches between these sets are pushed to lp
		// and the last set of features is discarded
		if (v.size() == 2) {

			////////////////////////////////////////////////////////////////////////////////////////////////////////
			lp.push_front(omniMatching(v[1], v[0]));
			////////////////////////////////////////////////////////////////////////////////////////////////////////

			//==========================================================================================
			// write the matched features for each pair of images
			// check if folder exists, if not create it
			if (!fs::exists(outputFolder + "/" + outputMatches)) {
				fs::create_directory(outputFolder + "/" + outputMatches);
			}

			std::string pathOutputMatches = outputFolder + "/" + outputMatches + "/" + lp.front()->getPairImageName();
			// open the file to write the matches
			std::ofstream outputFileMatches { pathOutputMatches };

			// Keypoints of the first image
			std::vector<KeyPoint> kp1 = lp.front()->getKeyPoints1();
			// Keypoints of the second image
			std::vector<KeyPoint> kp2 = lp.front()->getKeyPoints2();

			// loop over the vector of matches
			for (const auto &m: lp.front()->getMatches()) {

				// m.queryIdx is the index of the Keypoints on the first image
				// m.trainIdx is the index of the Keypoints on the second image
				outputFileMatches << std::setprecision(15) << kp1[m.queryIdx].pt.x << " " << kp1[m.queryIdx].pt.y << " "  <<
						 kp2[m.trainIdx].pt.x << " " << kp2[m.trainIdx].pt.y << std::endl;
			}
			outputFileMatches.close();
			//==========================================================================================

			v.pop_back();
		}

		// lp is a sort of queue where the matches are stored
		// whenever there are two sets of matches, the triplets are computed
		if (lp.size() == 2) {

			// calculates the keypoints common on two pairs of images
			// in this case it creates a triplet
			////////////////////////////////////////////////////////////////////////////////////////////////////////
			std::shared_ptr<TripletsWithMatches> p1 = commonPointsComputation(lp[1], lp[0]);
			////////////////////////////////////////////////////////////////////////////////////////////////////////

			// push the triplet to the thread-safe queue and send it for pose estimation
			tripletsProcQueue.push(p1);

			//==========================================================================================
			// write the matched features for two consecutive pair of images, i.e. triplets
			// check if folder exists, if not create it
			if (!fs::exists(outputFolder + "/" + outputTriplets)) {
				fs::create_directory(outputFolder + "/" + outputTriplets);
			}

			std::string pathOutputTriplets = outputFolder + "/" + outputTriplets + "/" + p1->getTripletImageName();
			// open the file to write the matches
			std::ofstream outputFileTriplets { pathOutputTriplets };

			// Keypoints of the first image of the first pair
			std::vector<KeyPoint> kpt1 = p1->getImage()[0]->getKeyPoints();
			// Keypoints of the second image of the first pair
			std::vector<KeyPoint> kpt2 = p1->getImage()[1]->getKeyPoints();
			// Keypoints of the second image of the second pair
			std::vector<KeyPoint> kpt3 = p1->getImage()[2]->getKeyPoints();

			// loop over the vector of matches
			// v is vector with the indices of the keypoints
			for (const auto &v : p1->getMatchVector()) {
				// kpt1[v[0]] is the keypoint of the first image of the first pair
				// kpt2[v[1]] is the keypoint of the second image of the first pair
				// kpt3[v[2]] is the keypoint of the second image of the second pair
				outputFileTriplets << std::setprecision(15) << kpt1[v[0]].pt.x << " " << kpt1[v[0]].pt.y << " " << kpt2[v[1]].pt.x << " "
						<< kpt2[v[1]].pt.y << " " << kpt3[v[2]].pt.x << " " << kpt3[v[2]].pt.y << std::endl;
			}
			outputFileTriplets.close();
			//==========================================================================================

			lp.pop_back();

		}
	}

	// signals the end of procFeatures
	mt->terminateProcFeatures = true;
}

//========================================================================================================

void ProcPose (MeasureTime *mt) {

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
		ss << "=========================" << std::endl << "Received triplets images " << "(" << receivedTripletsImages->getImageNumber1() << ", "
				<< receivedTripletsImages->getImageNumber2() << ", " << receivedTripletsImages->getImageNumber3() << ")" << std::endl
				<< "=========================" << std::endl;
		print(ss.str());

		// vector containing the spherical coordinates
		std::vector<Vec_Points<double>> p3d_liste { };

		// width and height of the images
		auto width = receivedTripletsImages->getImage()[0]->getOmni()->getImage().cols;
		auto height = receivedTripletsImages->getImage()[0]->getOmni()->getImage().rows;

		// loop over the spheres
		for (int idx { 0 }; idx < 3; ++idx) {

			Vec_Points<double> list_matches { };

			// if there are matches then convert them to spherical
			if (receivedTripletsImages->getMatchVector().size() != 0) {
				for (auto match : receivedTripletsImages->getMatchVector()) {
					// match is a vector of indices of the keypoints
					// for triplets its size is 3

					// gets the xy coordinate of the keypoint corresponding to sphere idx
					auto xy = receivedTripletsImages->getImage()[idx]->getKeyPoints()[match[idx]].pt;

					// convert x and y coordinates to spherical
					Points<double> p = convertCartesian2Spherical(static_cast<double>(xy.x), static_cast<double>(xy.y), width, height);

					// push_back the spherical coordinate into list_matches
					list_matches.push_back(p);
				}
			}
//			std::string plyFileNameOri { "./resources/models_ori_" + std::to_string(idx) + "_" + std::to_string(counter) + ".ply" };
//			writePly(plyFileNameOri, list_matches);

			p3d_liste.push_back(list_matches);
		}

		//==========================================================================================
		// output in a file the spherical coordinates of the triplet

		// check if folder exists, if not create it
		if (!fs::exists(outputFolder + "/" + outputSpherical)) {
			fs::create_directory(outputFolder + "/" + outputSpherical);
		}

		std::string pathOutputSpherical = outputFolder + "/" + outputSpherical + "/" + receivedTripletsImages->getTripletImageName();
		// open the file to write the matches
		std::ofstream outputFileSpherical { pathOutputSpherical };

		// loop over the vector of spherical coordinates
		for (size_t i { 0 }; i < p3d_liste[0].size(); ++i) {
			// p3d_liste[0][i] is a point with x, y, z coordinates of sphere 1
			// p3d_liste[1][i] is a point with x, y, z coordinates of sphere 2
			// p3d_liste[2][i] is a point with x, y, z coordinates of sphere 3
			outputFileSpherical << std::setprecision(15) << p3d_liste[0][i] << " " << p3d_liste[1][i] << " " << p3d_liste[2][i] << std::endl;
		}
		outputFileSpherical.close();

		//==========================================================================================

		Vec_Points<double> sv_scene { };
		std::vector<Points<double>> positions { Points<double>(), Points<double>(), Points<double>() };
		std::vector<Points<double>> sv_t_liste { Points<double>(), Points<double>(), Points<double>() };
		std::vector<Mat_33<double>> sv_r_liste { Mat_33<double>(), Mat_33<double>(), Mat_33<double>() };

		double error_max { 1e-8 };

		//-----------------------------------------------------
		// call to pose estimation algorithm
		int initialNumberFeatures = p3d_liste[0].size();

		int numIter {};
		// Only compute if there are features in the vector
		if (initialNumberFeatures!=0)
			numIter = pose_estimation(p3d_liste, error_max, sv_scene, positions, sv_r_liste, sv_t_liste);

		int finalNumberFeatures = p3d_liste[0].size();
		//-----------------------------------------------------

		//==========================================================================================
		// output in a file the rotation matrix and translation vector and statistics of the pose estimation algorithm

		// check if folder exists, if not create it
		if (!fs::exists(outputFolder + "/" + outputPose3)) {
			fs::create_directory(outputFolder + "/" + outputPose3);
		}

		std::string pathOutputPose3 = outputFolder + "/" + outputPose3 + "/" + receivedTripletsImages->getTripletImageName();
		// open the file to write the matches
		std::ofstream outputFilePose3 { pathOutputPose3 };

		// Output the rotation and translation matrices
		// Format (R12)(t12')(R23)(t23')
		//         3x3  3x1   3x3  3x1
		outputFilePose3 << std::setprecision(15) << sv_r_liste[0][0][0] << " " << sv_r_liste[0][0][1] << " " << sv_r_liste[0][0][2] << " "
				<< sv_t_liste[0][0] << " " << sv_r_liste[1][0][0] << " " << sv_r_liste[1][0][1] << " " << sv_r_liste[1][0][2] << " "
				<< sv_t_liste[1][0] << std::endl;
		outputFilePose3 << std::setprecision(15) << sv_r_liste[0][1][0] << " " << sv_r_liste[0][1][1] << " " << sv_r_liste[0][1][2] << " "
				<< sv_t_liste[0][1] << " " << sv_r_liste[1][1][0] << " " << sv_r_liste[1][1][1] << " " << sv_r_liste[1][1][2] << " "
				<< sv_t_liste[1][1] << std::endl;
		outputFilePose3 << std::setprecision(15) << sv_r_liste[0][2][0] << " " << sv_r_liste[0][2][1] << " " << sv_r_liste[0][2][2] << " "
				<< sv_t_liste[0][2] << " " << sv_r_liste[1][2][0] << " " << sv_r_liste[1][2][1] << " " << sv_r_liste[1][2][2] << " "
				<< sv_t_liste[1][2] << std::endl;
		outputFilePose3 << std::endl;
		outputFilePose3 << "Number of iterations: " << numIter << std::endl;
		outputFilePose3 << "Initial Number of Features  : " << initialNumberFeatures << std::endl;
		outputFilePose3 << "Remaining Number of Features: " << finalNumberFeatures << std::endl;

		outputFilePose3.close();
		//==========================================================================================

		//==========================================================================================
		// output in a file of the sparse point cloud of the triplet

		// check if folder exists, if not create it
		if (!fs::exists(outputFolder + "/" + outputPointCloud3)) {
			fs::create_directory(outputFolder + "/" + outputPointCloud3);
		}

		std::string pathOutputPointCloud3 = outputFolder + "/" + outputPointCloud3 + "/" + receivedTripletsImages->getTripletImageName();
		// open the file to write the matches
		std::ofstream outputFilePointCloud3 { pathOutputPointCloud3 };

		// loop over the vector of point cloud
		for (size_t i { 0 }; i < sv_scene.size(); ++i) {
			// sv_scene[i] is a point with x, y, z coordinates of the reconstructed triplet
			outputFilePointCloud3 << std::setprecision(15) << sv_scene[i] << std::endl;
		}

		outputFilePointCloud3.close();

		// write the ply file
		std::string pathOutputPointCloud3ply = outputFolder + "/" + outputPointCloud3 + "/" + receivedTripletsImages->getTripletImageName() + ".ply";
		writePly(pathOutputPointCloud3ply, sv_scene);

		//==========================================================================================


/*

		Model m2 { };
		cv::Matx13f modelCenter(0, 0, 0);

		// calculates the mean position of the triplets
		for (size_t i { 0 }; i < positions.size(); ++i) {
			Points<double> f = positions[i];
			modelCenter = modelCenter + Matx13f(f[0], f[1], f[2]);
		}
		modelCenter = 1.0 / positions.size() * modelCenter;

		// calculates the average distance of the reconstracted points with respect to the mean of the positions
		double averageDistance = 0;
		for (size_t i { 0 }; i < sv_scene.size(); ++i) {
			Points<double> f = sv_scene[i];
			averageDistance += norm(modelCenter - Matx13f(f[0], f[1], f[2]));
		}
		averageDistance /= sv_scene.size();

		// assigns a random color to the model
		RGB888 modelColor = RGB888(rng.uniform(0, 255), rng.uniform(0, 255), rng.uniform(0, 255));

		// copies to features vector of the model the points whose average distance is relatively close
		for (size_t i { 0 }; i < sv_scene.size(); ++i) {
			Points<double> f = sv_scene[i];
			if (norm(Matx13f(f[0], f[1], f[2]) - modelCenter) > 10 * averageDistance)
				continue;
			//m2.features.push_back(ModelFeature(1000 * Matx13f(f[0], f[1], f[2]), modelColor));  //RGB888(255,255, counter * 0x40)
			m2.features.push_back(ModelFeature(Matx13f(f[0], f[1], f[2]), modelColor));  //RGB888(255,255, counter * 0x40)
		}

		// copies to keypoints vector the positions of the spheres * 1000
		for (size_t i { 0 }; i < positions.size(); ++i) {
			Points<double> f = positions[i];
			m2.cameraPositions.push_back(ModelViewPoint(Matx13f(f[0], f[1], f[2])));
		}

		// writes the estimated triplet model
		std::string plyFileName { "./resources/models_est_" + std::to_string(counter) + ".ply" };
		writePly(plyFileName, m2.features);

		//-----------------------------------------------------
		fusionModel(&m, &m2);
		//-----------------------------------------------------

		//std::string fusionFileName { "./resources/models_fusion_" + std::to_string(counter) + ".ply" };
		//writePly(fusionFileName, m.features);
*/

		counter++;
	}
	// signals the end of pose estimation
	mt->terminateProcPose = true;
}


int main() {

	MeasureTime mt{};

	std::thread GenPairs (generatePairImages, &mt);
	std::thread ProcessFeatureExtraction (procFeatures, &mt);
	std::thread ProcessPoseEstimation (ProcPose, &mt);

	GenPairs.join();
	ProcessFeatureExtraction.join();
	ProcessPoseEstimation.join();

//	std::cout << "Average duration lapse gen pairs: " << mt.get_avg_gen_pairs() << " ms" << std::endl;
//	std::cout << "Average duration lapse feature extraction: " << mt.get_avg_feature_extract() << " ms" << std::endl;
//	std::cout << "Average duration lapse omni matching: " << mt.get_avg_omni_matching() << " ms" << std::endl;
//	std::cout << "Average duration lapse common point computation: " << mt.get_avg_common_p() << " ms" << std::endl;
//	std::cout << "Average duration lapse pose estimation: " << mt.get_avg_pose_estimation() << " ms" << std::endl;
//	std::cout << "Average duration lapse fusion: " << mt.get_avg_fusion() << " ms" << std::endl;

	return 0;

}






