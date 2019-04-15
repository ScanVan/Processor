#ifndef SRC_CONFIG_HPP_
#define SRC_CONFIG_HPP_

#include <experimental/filesystem>
#include <string>
#include <opencv4/opencv2/core/types.hpp>
#include "Vec_Points.hpp"
#include "DataModel.hpp"
#include "utils.hpp"

namespace fs = std::experimental::filesystem;

//#define USE_ORB_FEATURE
#define USE_AKAZE_FEATURE

//#define USE_KNN_MATCHER
#define USE_GMS_FILTER

class ModelViewPoint;
class ModelFeature;
class EquirectangularWithFeatures;
class PairWithMatches;
class TripletsWithMatches;
class Model;

//#define DISPLAY_TRIPLET_MATCHES
//#define DISPLAY_TRIPLET_MATCHES_INDIVIDUAL


class Config {
public:

	enum TypeOfExecution {
		RUN_ALL,
		RESUME,
		FILTER_STILL
	};

	// Path to the configuration file
	std::string ConfigFilePath {""};

	// Path to the main input folder, e.g. "/record/camera_40008603-40009302/20190319-115256_SionCar1"
	std::string inputFolder {""};

	// Path to the input mask file, e.g. "/model/camera_40008603-40009302/20190319-115256_SionCar1_20190327/input/1_mask/mask0.png"
	std::string inputMask {""};

	// Path to the output folder, e.g. "/model/camera_40008603-40009302/20190319-103441_SionCar1_20190327/output"
	std::string outputFolder {""};

	// Command of execution
	TypeOfExecution execType = RUN_ALL;

	// Threshold for determining the car is not moving
	double stillThrs = {};

	Config(std::string s) :
			ConfigFilePath(s) {
		ProcessConfigFile();
	}

	void SetInputFolder(std::string path) {
		inputFolder = path;
	}

	void SetInputMask(std::string path) {
		inputMask = path;
	}

	void SetOutputFolder(std::string path) {
		outputFolder = path;
	}

	// output folder where the features for each image is written
	const std::string outputFeatures { "1_features" };
	std::string PathOutputFeatures { };
	// output folder where the matches for each pair of image is written
	const std::string outputMatches { "2_matches" };
	std::string PathOutputMatches { };
	// output folder where the valid matches corresponding to vehicle in movement is written
	const std::string outputMatchesMoving { "2_matches_moving" };
	std::string PathOutputMatchesMoving { };
	// output folder where the triplets matches for two pairs of images is written
	const std::string outputTriplets { "3_triplets" };
	std::string PathOutputTriples { };
	// output folder where the filtered triplets matches (after pose estimation) for two pairs of images is written
	const std::string outputTripletsFiltered { "3_triplets_filtered" };
	std::string PathOutputTripletsFiltered { };
	// output folder where the spherical coordinates of the triplets are written
	const std::string outputSpherical { "4_spherical" };
	std::string PathOutputSpherical { };
	// output folder where the rotation and translation of triplets are written
	const std::string outputPose3 { "5_pose_3" };
	std::string PathOutputPose3 { };
	// output folder where the point cloud of the triplets are written
	const std::string outputPointCloud3 { "6_sparse_3" };
	std::string PathOutputPointCloud3 { };
	// output folder where the odometry results are written
	const std::string outputOdometry { "7_odometry" };
	std::string PathOutputOdometry { };
	// output folder where the progressive model is written
	const std::string outputProgressiveModel { "8_models" };
	std::string PathOutputProgressiveModel { };
	// output file name of the merged model
	const std::string outputMergedModelFileName { "sparse.ply" };
	std::string PathOutputMergeModelFileName { };


	void ProcessConfigFile();
	void CheckFolders();
	void write_1_features(const std::shared_ptr<EquirectangularWithFeatures> &featuredImages);
	void write_2_matches(const std::shared_ptr<PairWithMatches> &matches);
	void write_2_matches_moving (const std::shared_ptr<PairWithMatches> &matches);
	void write_3_triplets(const std::shared_ptr<TripletsWithMatches> &p1);
	void write_3_triplets_filtered(const std::shared_ptr<TripletsWithMatches> &p1, const std::vector<Vec_Points<double>> &p3d_liste_orig, const std::vector<Vec_Points<double>> &p3d_liste);
	void write_4_spherical(const std::shared_ptr<TripletsWithMatches> &triplets, const std::vector<Vec_Points<double>> &p3d_liste);
	void write_5_pose_3(const std::shared_ptr<TripletsWithMatches> &triplets,
						const std::vector<Mat_33<double>> &sv_r_liste,
						const std::vector<Points<double>> &sv_t_liste,
						const int numIter,
						const int initialNumberFeatures,
						const int finalNumberFeatures);
	void write_6_sparse_3 (const std::shared_ptr<TripletsWithMatches> &triplets, const Vec_Points<double> &sv_scene);
	void write_7_odometry (const Model &m);
	void write_8_progressiveModel (const Model &m);
	void write_9_finalModel (Model &m);
	void writePly(const std::string &file, const Vec_Points<double> &features);
	void writePly(std::string file, const std::vector<ModelFeature> &features);

	std::string trim(const std::string& str, const std::string& whitespace);
	std::string ToUpper(const std::string &str);
};




#endif /* SRC_CONFIG_HPP_ */
