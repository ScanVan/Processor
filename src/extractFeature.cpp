#include "pipelineAlgo.hpp"

std::shared_ptr<EquirectangularWithFeatures> extractFeatures(std::shared_ptr<Equirectangular> omni, std::shared_ptr<cv::Mat> mask) {
	//DEBUG_PTR(omni);

	std::shared_ptr<EquirectangularWithFeatures> featured { new EquirectangularWithFeatures { omni } };

#ifdef USE_ORB_FEATURE
	cv::Ptr<cv::ORB> orb = cv::ORB::create(1000);
	orb->setFastThreshold(0);
	orb->detectAndCompute(omni->getImage(), *mask, featured->getKeyPoints(), featured->getDesc());
#endif

#ifdef USE_AKAZE_FEATURE
	cv::Ptr<cv::AKAZE> akaze = cv::AKAZE::create(cv::AKAZE::DESCRIPTOR_MLDB, 0, 3, 0.0005f, 4, 4, cv::KAZE::DIFF_PM_G2);
	akaze->detectAndCompute(omni->getImage(), *mask, featured->getKeyPoints(), featured->getDesc());
#endif

	return featured;
}
