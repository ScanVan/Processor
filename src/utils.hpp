#ifndef SRC_UTILS_HPP_
#define SRC_UTILS_HPP_


#include <string>
#include <opencv4/opencv2/core/types.hpp>
#include "Vec_Points.hpp"
#include "DataModel.hpp"
#include "config.hpp"

class ModelViewPoint;
class ModelFeature;
class EquirectangularWithFeatures;
class PairWithMatches;
class TripletsWithMatches;
class Model;

#define DEBUG_PTR(ptr) auto ptr##_ = ptr.get();

//std::string trim(const std::string& str, const std::string& whitespace);
//std::string ToUpper(const std::string &str);
//void ProcessConfigFile(std::string &cfg);
void checkFolders();
void write_1_features(const std::shared_ptr<EquirectangularWithFeatures> &featuredImages);
void write_2_matches(const std::shared_ptr<PairWithMatches> &matches) ;
void write_3_triplets(const std::shared_ptr<TripletsWithMatches> &triplets);
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

void print (std::string st);
void writePly(std::string file, const std::vector<ModelFeature> &features);
void writePly(const std::string &file, const Vec_Points<double> &features);
std::vector<ModelFeature> keypointsToFeatures(std::vector<ModelViewPoint> *keypoints);
cv::Matx13f cross(cv::Matx13f a, cv::Matx13f b);

#endif /* SRC_UTILS_HPP_ */
