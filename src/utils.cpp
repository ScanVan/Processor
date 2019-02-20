#include "utils.hpp"
#include <experimental/filesystem>

namespace fs = std::experimental::filesystem;

void writePly(const std::string &file, const Vec_Points<double> &features);

static std::mutex mtx{};
void print (std::string st) {
	std::unique_lock<std::mutex> lck {mtx};
	std::cout << st;
}

void checkFolders() {

	// checks if the folder to write the outputs exist
	// if it exists, deletes all the content and create a new empty directory

	// folder that contains the feature points for each image
	if (fs::exists(outputFolder + "/" + outputFeatures)) {
		fs::remove_all(outputFolder + "/" + outputFeatures);
	}
	fs::create_directory(outputFolder + "/" + outputFeatures);

	// folder that contains the feature matches for each pair images
	if (fs::exists(outputFolder + "/" + outputMatches)) {
		fs::remove_all(outputFolder + "/" + outputMatches);
	}
	fs::create_directory(outputFolder + "/" + outputMatches);

	// folder that contains the triplets
	if (fs::exists(outputFolder + "/" + outputTriplets)) {
		fs::remove_all(outputFolder + "/" + outputTriplets);
	}
	fs::create_directory(outputFolder + "/" + outputTriplets);

	// folder that contains the spherical coordinates
	if (fs::exists(outputFolder + "/" + outputSpherical)) {
		fs::remove_all(outputFolder + "/" + outputSpherical);
	}
	fs::create_directory(outputFolder + "/" + outputSpherical);

	// folder that contains the rotation and translation matrices
	if (fs::exists(outputFolder + "/" + outputPose3)) {
		fs::remove_all(outputFolder + "/" + outputPose3);
	}
	fs::create_directory(outputFolder + "/" + outputPose3);

	// folder that conatains the sparse point cloud of the triplets
	if (fs::exists(outputFolder + "/" + outputPointCloud3)) {
		fs::remove_all(outputFolder + "/" + outputPointCloud3);
	}
	fs::create_directory(outputFolder + "/" + outputPointCloud3);

	// folder that contains the odometry results
	if (fs::exists(outputFolder + "/" + outputOdometry)) {
		fs::remove_all(outputFolder + "/" + outputOdometry);
	}
	fs::create_directory(outputFolder + "/" + outputOdometry);

	// folder that contains the progressive models
	if (fs::exists(outputFolder + "/" + outputProgressiveModel)) {
		fs::remove_all(outputFolder + "/" + outputProgressiveModel);
	}
	fs::create_directory(outputFolder + "/" + outputProgressiveModel);

	// if the sparse model exists, it deletes it
	if (fs::exists(outputFolder + "/" + outputMergedModelFileName)) {
		fs::remove(outputFolder + "/" + outputMergedModelFileName);
	}

}

void write_1_features(const std::shared_ptr<EquirectangularWithFeatures> &featuredImages) {
// writes the features in the output file

	std::string pathOutputFeature = outputFolder + "/" + outputFeatures + "/" + featuredImages->getImgName();

	// write features extracted for each image into files
	// open the file to write the features
	std::ofstream outputFile { pathOutputFeature, std::ios::trunc };
	// go over all the features extracted and write them into the file
	for (const auto kp : featuredImages->getKeyPoints()) {
		outputFile << std::setprecision(15) << kp.pt.x << " " << kp.pt.y << std::endl;
	}
	outputFile.close();
}

void write_2_matches(const std::shared_ptr<PairWithMatches> &matches) {
// writes the matches in the output file

	std::string pathOutputMatches = outputFolder + "/" + outputMatches + "/" + matches->getPairImageName();
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

void write_3_triplets(const std::shared_ptr<TripletsWithMatches> &p1) {
// writes the triplets in the output file

	std::string pathOutputTriplets = outputFolder + "/" + outputTriplets + "/" + p1->getTripletImageName();
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

void write_4_spherical(const std::shared_ptr<TripletsWithMatches> &triplets, const std::vector<Vec_Points<double>> &p3d_liste) {
// writes the spherical coordinates of each sphere

	std::string pathOutputSpherical = outputFolder + "/" + outputSpherical + "/" + triplets->getTripletImageName();
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

void write_5_pose_3(const std::shared_ptr<TripletsWithMatches> &triplets,
					const std::vector<Mat_33<double>> &sv_r_liste,
					const std::vector<Points<double>> &sv_t_liste,
					const int numIter,
					const int initialNumberFeatures,
					const int finalNumberFeatures)
{
// writes the rotation and translation matrices and statistics of the pose_estimation algorithm

	std::string pathOutputPose3 = outputFolder + "/" + outputPose3 + "/" + triplets->getTripletImageName();
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

void write_6_sparse_3 (const std::shared_ptr<TripletsWithMatches> &triplets, const Vec_Points<double> &sv_scene) {
// writes the sparse point cloud of the triplets

	std::string pathOutputPointCloud3 = outputFolder + "/" + outputPointCloud3 + "/" + triplets->getTripletImageName();
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

void write_7_odometry (const Model &m) {
// writes the rotation and translation absolutes for each camera position

	// reverse iterator to the camera view points
	auto it2 = m.viewPoints.rbegin();

	for (auto it = m.imgNames.rbegin(); it != m.imgNames.rend(); ++it, ++it2) {
		std::string pathOutputOdometry = outputFolder + "/" + outputOdometry + "/" + *it;
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

void write_8_progressiveModel (const Model &m) {
// write the point cloud of the merged model as it is added
	std::string fusionFileName { outputFolder + "/" + outputProgressiveModel + "/" + std::to_string(m.viewPoints.size()) + ".ply" };
	writePly(fusionFileName, m.features);
}

void write_9_finalModel (Model &m) {
// write the point cloud of the merged model

/*  // This adds the positions of the cameras as red points on the model
	// assigns a red color for the positions of the camera
	RGB888 modelColor = RGB888(255, 0, 0);

	for (size_t i { 0 }; i < m.viewPoints.size(); ++i) {
		cv::Matx13d position = m.viewPoints[i].position;
		m.features.push_back(ModelFeature(position, modelColor));
	}
*/

	std::string fusionFileName { outputFolder + "/" + outputMergedModelFileName };
	writePly(fusionFileName, m.features);
}


void writePly(std::string file, const std::vector<ModelFeature> &features){
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


void writePly(const std::string &file, const Vec_Points<double> &features){

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


std::vector<ModelFeature> keypointsToFeatures(std::vector<ModelViewPoint> *keypoints){
	std::vector<ModelFeature> ret;
	for(const auto &k : *keypoints) ret.push_back(ModelFeature(k.position, RGB888(0,255,0)));
	return ret;
}



cv::Matx13f cross(cv::Matx13f a, cv::Matx13f b){
	auto c = cv::Point3f(a(0), a(1), a(2)).cross(cv::Point3f(b(0), b(1), b(2)));
	return cv::Matx13f(c.x,c.y,c.z);
}
