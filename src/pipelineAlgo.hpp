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

#include "ctpl.hpp"
#include "Queue.hpp"
#include "DataModel.hpp"
#include "config.hpp"


using namespace std;
using namespace cv;


shared_ptr<OmniWithFeatures> extractFeatures(shared_ptr<Omni> omni, shared_ptr<Mat> mask);
std::shared_ptr<PairWithMatches> omniMatching (std::shared_ptr<OmniWithFeatures> im1, std::shared_ptr<OmniWithFeatures> im2);
shared_ptr<TripletsWithMatches> commonPointsComputation (std::shared_ptr<PairWithMatches> p1, std::shared_ptr<PairWithMatches> p2);
shared_ptr<Model> poseEstimation (shared_ptr<TripletsWithMatches> t1);
void fusionModel (Model *m1, Model *m2);


void ProcPose();





#endif /* SRC_PIPELINEALGO_HPP_ */
