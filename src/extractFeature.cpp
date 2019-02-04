#include "pipelineAlgo.hpp"

shared_ptr<EquirectangularWithFeatures> extractFeatures(shared_ptr<Equirectangular> omni, shared_ptr<cv::Mat> mask) {
	//DEBUG_PTR(omni);

	shared_ptr<EquirectangularWithFeatures> featured { new EquirectangularWithFeatures { omni } };

#ifdef USE_ORB_FEATURE
	Ptr<ORB> orb = ORB::create(1000);
	orb->setFastThreshold(0);
	orb->detectAndCompute(omni->img, *mask, featured->kpts, featured->desc);
#endif

#ifdef USE_AKAZE_FEATURE
	Ptr<AKAZE> akaze = AKAZE::create(AKAZE::DESCRIPTOR_MLDB, 0, 3, 0.0005f, 4, 4, KAZE::DIFF_PM_G2);
	akaze->detectAndCompute(omni->getImage(), *mask, featured->getKeyPoints(), featured->getDesc());
#endif

	return featured;
}
