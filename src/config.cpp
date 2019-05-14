#include "config.hpp"

#include <experimental/filesystem>
#include <string>
#include <opencv4/opencv2/core/types.hpp>
#include "Vec_Points.hpp"
#include "DataModel.hpp"
#include "utils.hpp"

namespace fs = std::experimental::filesystem;

std::string Config::trim(const std::string& str, const std::string& whitespace = " \t") {
// trim white spaces and tabs from the beginning and end

	const auto strBegin = str.find_first_not_of(whitespace);
	if (strBegin == std::string::npos)
		return ""; // no content

	const auto strEnd = str.find_last_not_of(whitespace);
	const auto strRange = strEnd - strBegin + 1;

	return str.substr(strBegin, strRange);
}

std::string Config::ToUpper(const std::string &str) {
// Convert to upper case the string

	std::locale loc { };
	std::stringstream ss { };
	for (std::string::size_type i = 0; i < str.length(); ++i)
		ss << std::toupper(str[i], loc);
	return (ss.str());
}

void Config::ProcessConfigFile() {

	// open configuration file
	std::ifstream f(ConfigFilePath);

	if (!f.good()) {
		throw std::runtime_error("Error opening the configuration file \"" + ConfigFilePath + "\".");
	}

	// process configuration file
	while (f.good()) {

		std::string line { };
		std::getline(f, line);

		// remove white spaces from the line
		std::string trimmedLine = trim(line);
		if (trimmedLine == "")
			continue;

		// Ignore lines starting with hash symbol
		const auto strHash = trimmedLine.find_first_of("#");
		if (strHash == 0)
			continue;

		// Extract the command and the argument separated by the symbol "="
		const auto strEqual = trimmedLine.find_first_of("=");
		std::string command = ToUpper(trim(trimmedLine.substr(0, strEqual)));
		std::string argument = trim(trimmedLine.substr(strEqual + 1));

		// Check for commands
		if (command == "PATH_TO_INPUT") {
			// Path of the main input folder where the equirectangular images are located
			if (argument != "") {
				fs::path p = argument;
				if (!fs::exists(p)) {
					throw(std::runtime_error("Error: the path " + argument + " does not exist."));
				}
				inputFolder = argument;
			} else {
				throw(std::runtime_error("Error: PATH_TO_INPUT parameter is empty."));
			}
		} else if (command == "PATH_TO_MASK") {
			// Path to the input mask file
			if (argument != "") {
				fs::path p = argument;
				if (!fs::exists(p)) {
					throw(std::runtime_error("Error: the path " + argument + " does not exist."));
				}
				inputMask = argument;
			} else {
				throw(std::runtime_error("Error: PATH_TO_MASK parameter is empty."));
			}
		} else if (command == "PATH_TO_OUTPUT") {
			// Path to the output folder where a tree of sub-folders will be created
			if (argument != "") {
				fs::path p = argument;
				if (!fs::exists(p)) {
					throw(std::runtime_error("Error: the path " + argument + " does not exist."));
				}
				outputFolder = argument;
			} else {
				throw(std::runtime_error("Error: PATH_TO_OUTPUT parameter is empty."));
			}
		} else if (command == "EXEC_TYPE") {
			// The execution type for the pipeline
			if (argument != "") {
				if (argument == "RUN_ALL") {
					execType = RUN_ALL;
				} else if (argument == "FILTER_STILL") {
					execType = FILTER_STILL;
				} else {
					throw(std::runtime_error("Error: the parameter EXEC_TYPE is invalid"));
				}
			} else {
				throw(std::runtime_error("Error: EXEC_TYPE parameter is empty."));
			}

		}  else if (command == "STILL_THRS") {
			// The threshold for detecting that the car is not moving
			if (argument != "") {
				std::stringstream ss (argument);
				try {
					ss >> std::setprecision(15) >> stillThrs;
				} catch (...) {
					throw(std::runtime_error("Error: STILL_THRS has an invalid parameter."));
				}
			} else {
				throw(std::runtime_error("Error: STILL_THRS parameter is empty."));
			}

		}
	}
}

void Config::CheckFolders() {

	fs::path p1 = outputFolder;


	fs::path p2 = p1 / outputFeatures;
	PathOutputFeatures = p2.string();

	p2 = p1 / outputMatches;
	PathOutputMatches = p2.string();

	p2 = p1 / outputMatchesMoving;
	PathOutputMatchesMoving = p2.string();

	p2 = p1 / outputMatchesMovingIndex;
	PathOutputMatchesMovingIndex = p2.string();

	p2 = p1 / outputTriplets;
	PathOutputTriples = p2.string();

	p2 = p1 / outputTripletsFiltered;
	PathOutputTripletsFiltered = p2.string();

	p2 = p1 / outputSpherical;
	PathOutputSpherical = p2.string();

	p2 = p1 / outputPose3;
	PathOutputPose3 = p2.string();

	p2 = p1 / outputPointCloud3;
	PathOutputPointCloud3 = p2.string();

	p2 = p1 / outputOdometry;
	PathOutputOdometry = p2.string();

	p2 = p1 / outputProgressiveModel;
	PathOutputProgressiveModel = p2.string();

	p2 = p1 / outputMergedModelFileName;
	PathOutputMergeModelFileName = p2.string();

	// checks if the folder to write the outputs exist
	// if it exists, deletes all the content and create a new empty directory

	// folder that contains the feature points for each image
	if (!fs::exists(PathOutputFeatures)) {
		fs::create_directory(PathOutputFeatures);
	}

	// folder that contains the feature matches for each pair images
	if (!fs::exists(PathOutputMatches)) {
		fs::create_directory(PathOutputMatches);
	}

	// folder that contains the matches when the car is moving
	if (!fs::exists(PathOutputMatchesMoving)) {
		fs::create_directory(PathOutputMatchesMoving);
	}

	// folder that contains the matches when the car is moving
	if (!fs::exists(PathOutputMatchesMovingIndex)) {
		fs::create_directory(PathOutputMatchesMovingIndex);
	}

	// folder that contains the triplets
	if (!fs::exists(PathOutputTriples)) {
		fs::create_directory(PathOutputTriples);
	}

	// folder that contains the triplets filtered after pose estimation
	if (!fs::exists(PathOutputTripletsFiltered)) {
		fs::create_directory(PathOutputTripletsFiltered);
	}

	// folder that contains the spherical coordinates
	if (!fs::exists(PathOutputSpherical)) {
		fs::create_directory(PathOutputSpherical);
	}

	// folder that contains the rotation and translation matrices
	if (!fs::exists(PathOutputPose3)) {
		fs::create_directory(PathOutputPose3);
	}

	// folder that contains the sparse point cloud of the triplets
	if (!fs::exists(PathOutputPointCloud3)) {
		fs::create_directory(PathOutputPointCloud3);
	}

	// folder that contains the odometry results
	if (!fs::exists(PathOutputOdometry)) {
		fs::create_directory(PathOutputOdometry);
	}

	// folder that contains the progressive models
	if (!fs::exists(PathOutputProgressiveModel)) {
		fs::create_directory(PathOutputProgressiveModel);
	}

}

void Config::write_1_features(const std::shared_ptr<EquirectangularWithFeatures> &featuredImages) {
// writes the features in the output file

	fs::path p1 = PathOutputFeatures;
	fs::path p2 = p1 / featuredImages->getImgName();

	std::string pathOutputFeature = p2.string();

	// write features extracted for each image into files
	// open the file to write the features
	std::ofstream outputFile { pathOutputFeature, std::ios::trunc };
	// go over all the features extracted and write them into the file
	for (const auto kp : featuredImages->getKeyPoints()) {
		outputFile << std::setprecision(15) << kp.pt.x << " " << kp.pt.y << std::endl;
	}
	outputFile.close();
}


void Config::write_2_matches(const std::shared_ptr<PairWithMatches> &matches) {
// writes the matches in the output file

	fs::path p1 = PathOutputMatches;
	fs::path p2 = p1 / matches->getPairImageName();

	std::string pathOutputMatches = p2.string();
	// open the file to write the matches
	std::ofstream outputFileMatches { pathOutputMatches, std::ios::trunc };

	// Keypoints of the first image
	std::vector<cv::KeyPoint> kp1 = matches->getKeyPoints1();
	// Keypoints of the second image
	std::vector<cv::KeyPoint> kp2 = matches->getKeyPoints2();

	// loop over the vector of matches
	for (const auto &m : matches->getMatches()) {

		// m.queryIdx is the index of the Keypoints on the first image
		// m.trainIdx is the index of the Keypoints on the second image
		outputFileMatches << std::setprecision(15) << kp1[m.queryIdx].pt.x << " " << kp1[m.queryIdx].pt.y << " " << kp2[m.trainIdx].pt.x << " "
				<< kp2[m.trainIdx].pt.y << std::endl;
	}
	outputFileMatches.close();

}

void Config::write_2_matches_moving (const std::shared_ptr<PairWithMatches> &matches) {
// writes the matches in the output file

	fs::path p1 = PathOutputMatchesMoving;
	fs::path p2 = p1 / matches->getPairImageName();

	std::string pathOutputMatches = p2.string();
	// open the file to write the matches
	std::ofstream outputFileMatches { pathOutputMatches, std::ios::trunc };

	// Keypoints of the first image
	std::vector<cv::KeyPoint> kp1 = matches->getKeyPoints1();
	// Keypoints of the second image
	std::vector<cv::KeyPoint> kp2 = matches->getKeyPoints2();

	// loop over the vector of matches
	for (const auto &m : matches->getMatches()) {

		// m.queryIdx is the index of the Keypoints on the first image
		// m.trainIdx is the index of the Keypoints on the second image
		outputFileMatches << std::setprecision(15) << kp1[m.queryIdx].pt.x << " " << kp1[m.queryIdx].pt.y << " " << kp2[m.trainIdx].pt.x << " "
				<< kp2[m.trainIdx].pt.y << std::endl;
	}
	outputFileMatches.close();

}

void Config::write_2_matches_moving_index (const std::shared_ptr<PairWithMatches> &matches) {
// writes the matches in the output file

	fs::path p1 = PathOutputMatchesMovingIndex;
	fs::path p2 = p1 / matches->getPairImageName();

	std::string pathOutputMatches = p2.string();
	// open the file to write the matches
	std::ofstream outputFileMatches { pathOutputMatches, std::ios::trunc };

	// Keypoints of the first image
	std::vector<cv::KeyPoint> kp1 = matches->getKeyPoints1();
	// Keypoints of the second image
	std::vector<cv::KeyPoint> kp2 = matches->getKeyPoints2();

	// loop over the vector of matches
	for (const auto &m : matches->getMatches()) {

		// m.queryIdx is the index of the Keypoints on the first image
		// m.trainIdx is the index of the Keypoints on the second image
		outputFileMatches << std::setprecision(15) << m.queryIdx << " " << m.trainIdx << " " << std::endl;
	}
	outputFileMatches.close();

}

void Config::write_3_triplets(const std::shared_ptr<TripletsWithMatches> &p1) {
// writes the triplets in the output file

	fs::path path1 = PathOutputTriples;
	fs::path path2 = path1 / p1->getTripletImageName();
	std::string pathOutputTriplets = path2.string();

	// open the file to write the matches
	std::ofstream outputFileTriplets { pathOutputTriplets, std::ios::trunc };

	// Keypoints of the first image of the first pair
	std::vector<cv::KeyPoint> kpt1 = p1->getImage()[0]->getKeyPoints();
	// Keypoints of the second image of the first pair
	std::vector<cv::KeyPoint> kpt2 = p1->getImage()[1]->getKeyPoints();
	// Keypoints of the second image of the second pair
	std::vector<cv::KeyPoint> kpt3 = p1->getImage()[2]->getKeyPoints();

	// loop over the vector of matches
	// v is vector with the indices of the keypoints
	for (const auto &v : p1->getMatchVector()) {
		// kpt1[v[0]] is the keypoint of the first image of the first pair
		// kpt2[v[1]] is the keypoint of the second image of the first pair
		// kpt3[v[2]] is the keypoint of the second image of the second pair
		outputFileTriplets << std::setprecision(15)
				<< kpt1[v[0]].pt.x << " "
				<< kpt1[v[0]].pt.y << " "
				<< kpt2[v[1]].pt.x << " "
				<< kpt2[v[1]].pt.y << " "
				<< kpt3[v[2]].pt.x << " "
				<< kpt3[v[2]].pt.y << std::endl;
	}

//	// Outputs the frequency of usage of the keypoints to check that the same
//	// keypoint is not used many times
//	outputFileTriplets << std::endl;
//	for (const auto &x : p1->getFrequencyMatches1()) {
//		outputFileTriplets << x << " ";
//	}
//	outputFileTriplets << std::endl << std::endl;
//	for (const auto &x : p1->getFrequencyMatches2()) {
//		outputFileTriplets << x << " ";
//	}
//	outputFileTriplets << std::endl << std::endl;
//	for (const auto &x : p1->getFrequencyMatches3()) {
//		outputFileTriplets << x << " ";
//	}
//	outputFileTriplets << std::endl << std::endl;

	outputFileTriplets.close();
}

void Config::write_3_triplets_filtered(const std::shared_ptr<TripletsWithMatches> &p1, const std::vector<Vec_Points<double>> &p3d_liste_orig, const std::vector<Vec_Points<double>> &p3d_liste) {
// writes the filtered triplets in the output file

	fs::path path1 = PathOutputTripletsFiltered;
	fs::path path2 = path1 / p1->getTripletImageName();
	std::string pathOutputTriplets = path2.string();

	// open the file to write the matches
	std::ofstream outputFileTriplets { pathOutputTriplets, std::ios::trunc };

	// Keypoints of the first image of the first pair
	std::vector<cv::KeyPoint> kpt1 = p1->getImage()[0]->getKeyPoints();
	// Keypoints of the second image of the first pair
	std::vector<cv::KeyPoint> kpt2 = p1->getImage()[1]->getKeyPoints();
	// Keypoints of the second image of the second pair
	std::vector<cv::KeyPoint> kpt3 = p1->getImage()[2]->getKeyPoints();

	// Select the list of filtered points of the first sphere
	const Vec_Points<double> &final_list = p3d_liste[0];
	const Vec_Points<double> &original_list = p3d_liste_orig[0];

	if (p1->getMatchVector().size() != original_list.size()) {
		throw (std::runtime_error("Error in the length of the feature match vector and the vector of spherical coordinates."));
	}

	int index { 0 };
	// loop over the vector of matches
	// v is vector with the indices of the keypoints
	for (const auto &v : p1->getMatchVector()) {

		// a point in the original list from the sphere 0
		const Points<double> &point_orig = p3d_liste_orig[0][index];

		if (final_list.contains(point_orig)) {

			// kpt1[v[0]] is the keypoint of the first image of the first pair
			// kpt2[v[1]] is the keypoint of the second image of the first pair
			// kpt3[v[2]] is the keypoint of the second image of the second pair
			outputFileTriplets << std::setprecision(15) << kpt1[v[0]].pt.x << " " << kpt1[v[0]].pt.y << " " << kpt2[v[1]].pt.x << " "
					<< kpt2[v[1]].pt.y << " " << kpt3[v[2]].pt.x << " " << kpt3[v[2]].pt.y << std::endl;
		}

		index++;
	}

	outputFileTriplets.close();
}

void Config::write_4_spherical(const std::shared_ptr<TripletsWithMatches> &triplets, const std::vector<Vec_Points<double>> &p3d_liste) {
// writes the spherical coordinates of each sphere

	fs::path p1 = PathOutputSpherical;
	fs::path p2 = p1 / triplets->getTripletImageName();
	std::string pathOutputSpherical = p2.string();

	// open the file to write the matches
	std::ofstream outputFileSpherical { pathOutputSpherical, std::ios::trunc };

	// loop over the vector of spherical coordinates
	for (size_t i { 0 }; i < p3d_liste[0].size(); ++i) {
		// p3d_liste[0][i] is a point with x, y, z coordinates of sphere 1
		// p3d_liste[1][i] is a point with x, y, z coordinates of sphere 2
		// p3d_liste[2][i] is a point with x, y, z coordinates of sphere 3
		outputFileSpherical << std::setprecision(15) << p3d_liste[0][i] << " " << p3d_liste[1][i] << " " << p3d_liste[2][i] << std::endl;
	}
	outputFileSpherical.close();

}



void Config::write_5_pose_3(const std::shared_ptr<TripletsWithMatches> &triplets,
					const std::vector<Mat_33<double>> &sv_r_liste,
					const std::vector<Points<double>> &sv_t_liste,
					const int numIter,
					const int initialNumberFeatures,
					const int finalNumberFeatures)
{
// writes the rotation and translation matrices and statistics of the pose_estimation algorithm

	fs::path p1 = PathOutputPose3;
	fs::path p2 = p1 / triplets->getTripletImageName();
	std::string pathOutputPose3 = p2.string();

	// open the file to write the matches
	std::ofstream outputFilePose3 { pathOutputPose3, std::ios::trunc };

	// Output the rotation and translation matrices
	// Format (R12)(t12')(R23)(t23')
	//         3x3  3x1   3x3  3x1
	outputFilePose3 << std::setprecision(15)
				<< sv_r_liste[0][0][0] << " "
				<< sv_r_liste[0][0][1] << " "
				<< sv_r_liste[0][0][2] << " "
				<< sv_t_liste[0][0] << " "
				<< sv_r_liste[1][0][0] << " "
				<< sv_r_liste[1][0][1] << " "
				<< sv_r_liste[1][0][2] << " "
				<< sv_t_liste[1][0] << std::endl;
	outputFilePose3 << std::setprecision(15)
				<< sv_r_liste[0][1][0] << " "
				<< sv_r_liste[0][1][1] << " "
				<< sv_r_liste[0][1][2] << " "
				<< sv_t_liste[0][1] << " "
				<< sv_r_liste[1][1][0] << " "
				<< sv_r_liste[1][1][1] << " "
				<< sv_r_liste[1][1][2] << " "
				<< sv_t_liste[1][1] << std::endl;
	outputFilePose3 << std::setprecision(15)
				<< sv_r_liste[0][2][0] << " "
				<< sv_r_liste[0][2][1] << " "
				<< sv_r_liste[0][2][2] << " "
				<< sv_t_liste[0][2] << " "
				<< sv_r_liste[1][2][0] << " "
				<< sv_r_liste[1][2][1] << " "
				<< sv_r_liste[1][2][2] << " "
				<< sv_t_liste[1][2] << std::endl;
	outputFilePose3 << std::endl;
	outputFilePose3 << "Number of iterations: " << numIter << std::endl;
	outputFilePose3 << "Initial Number of Features  : " << initialNumberFeatures << std::endl;
	outputFilePose3 << "Remaining Number of Features: " << finalNumberFeatures << std::endl;

	outputFilePose3.close();
}


void Config::write_6_sparse_3 (const std::shared_ptr<TripletsWithMatches> &triplets, const Vec_Points<double> &sv_scene) {
// writes the sparse point cloud of the triplets

	fs::path p1 = PathOutputPointCloud3;
	fs::path p2 = p1 / triplets->getTripletImageName();
	std::string pathOutputPointCloud3 = p2.string();

	// open the file to write the matches
	std::ofstream outputFilePointCloud3 { pathOutputPointCloud3, std::ios::trunc };

	// loop over the vector of point cloud
	for (size_t i { 0 }; i < sv_scene.size(); ++i) {
		// sv_scene[i] is a point with x, y, z coordinates of the reconstructed triplet
		outputFilePointCloud3 << std::setprecision(15) << sv_scene[i] << std::endl;
	}

	outputFilePointCloud3.close();

	// write the ply file
	std::string pathOutputPointCloud3ply = outputFolder + "/" + outputPointCloud3 + "/" + triplets->getTripletImageName() + ".ply";
	writePly(pathOutputPointCloud3ply, sv_scene);

}


void Config::write_7_odometry (const Model &m) {
// writes the rotation and translation absolutes for each camera position

	// reverse iterator to the camera view points
	auto it2 = m.viewPoints.rbegin();

	for (auto it = m.imgNames.rbegin(); it != m.imgNames.rend(); ++it, ++it2) {
		fs::path p1 = PathOutputOdometry;
		fs::path p2 = p1 / *it;

		std::string pathOutputOdometry = p2.string();
		// Check if the file exists
		if (fs::exists(pathOutputOdometry)) {
			break;
		}
		// open the file to write the matches
		std::ofstream outputFileOdometry { pathOutputOdometry, std::ios::trunc };

		auto &rot = it2->rotationAbsolute;
		auto &pos = it2->position;

		// Output the rotation and translation matrices
		// Format (R)(t')
		//         3x3  3x1
		outputFileOdometry << std::setprecision(15)
					<< rot(0,0) << " "
					<< rot(0,1) << " "
					<< rot(0,2) << " "
					<< pos(0) << std::endl;
		outputFileOdometry << std::setprecision(15)
					<< rot(1,0) << " "
					<< rot(1,1) << " "
					<< rot(1,2) << " "
					<< pos(1) << std::endl;
		outputFileOdometry << std::setprecision(15)
					<< rot(2,0) << " "
					<< rot(2,1) << " "
					<< rot(2,2) << " "
					<< pos(2) << std::endl;

		outputFileOdometry.close();

	}

}

void Config::write_8_progressiveModel (const Model &m) {
// write the point cloud of the merged model as it is added
	fs::path p1 = PathOutputProgressiveModel;
	fs::path p2 = p1 / (std::to_string(m.viewPoints.size()) + ".ply");
	std::string fusionFileName = p2.string();
	writePly(fusionFileName, m.features);
}

void Config::write_9_finalModel (Model &m) {
// write the point cloud of the merged model

/*  // This adds the positions of the cameras as red points on the model
	// assigns a red color for the positions of the camera
	RGB888 modelColor = RGB888(255, 0, 0);

	for (size_t i { 0 }; i < m.viewPoints.size(); ++i) {
		cv::Matx13d position = m.viewPoints[i].position;
		m.features.push_back(ModelFeature(position, modelColor));
	}
*/

	std::string fusionFileName { PathOutputMergeModelFileName };
	writePly(fusionFileName, m.features);
}


void Config::writePly(const std::string &file, const Vec_Points<double> &features){

	std::ofstream s {};
	s.open (file);

	s << "ply" << std::endl;
	s << "format ascii 1.0 " << std::endl;
	s << "element vertex " << features.size() << std::endl;
	s << "comment " << file << std::endl;
	s << "property float32 x " << std::endl;
	s << "property float32 y " << std::endl;
	s << "property float32 z " << std::endl;
	s << "end_header " << std::endl;

	for(size_t i {0}; i < features.size(); ++i){
		Points<double> f = features[i];
		s << f[0] << " " << f[1] << " " << f[2] << std::endl;
	}

	s.close();
}

void Config::writePly(std::string file, const std::vector<ModelFeature> &features){
	std::ofstream s {};
	s.open (file, std::ios::trunc);

	s << "ply" << std::endl;
	s << "format ascii 1.0 " << std::endl;
	s << "element vertex " << features.size() << std::endl;
	s << "property float32 x " << std::endl;
	s << "property float32 y " << std::endl;
	s << "property float32 z " << std::endl;
	s << "property uchar red" << std::endl;
	s << "property uchar green " << std::endl;
	s << "property uchar blue " << std::endl;
	s << "element face 0 " << std::endl;
	s << "property list uint8 int32 vertex_index" << std::endl;
	s << "end_header " << std::endl;

	for(auto f : features){
		s << f.position(0) << " " << f.position(1) << " " << f.position(2) << " " <<  (uint16_t)f.color(0) << " " << (uint16_t)f.color(1) << " " << (uint16_t)f.color(2) << std::endl;
	}

	s.close();
}
