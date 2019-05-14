#ifndef SRC_PIPELINEALGO_HPP_
#define SRC_PIPELINEALGO_HPP_

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

//#include "ctpl.hpp"
#include "DataModel.hpp"
#include "config.hpp"
#include "Queue.hpp"

std::shared_ptr<EquirectangularWithFeatures> extractFeatures(std::shared_ptr<Equirectangular> omni, std::shared_ptr<cv::Mat> mask);
std::shared_ptr<PairWithMatches> omniMatching (std::shared_ptr<EquirectangularWithFeatures> im1, std::shared_ptr<EquirectangularWithFeatures> im2);
bool movementCheck (std::shared_ptr<PairWithMatches> matches, double threshold);
std::shared_ptr<TripletsWithMatches> commonPointsComputation (std::shared_ptr<PairWithMatches> p1, std::shared_ptr<PairWithMatches> p2);
std::shared_ptr<Model> poseEstimation (std::shared_ptr<TripletsWithMatches> t1);
void fusionModel (Model *m1, Model *m2);
void fusionModel2 (Model *m1, Model *m2, uint32_t commonViewPointsCount);


#endif /* SRC_PIPELINEALGO_HPP_ */
