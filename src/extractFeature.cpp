#include "pipelineAlgo.hpp"

shared_ptr<OmniWithFeatures> extractFeatures(shared_ptr<Omni> omni, shared_ptr<Mat> mask) {
	//DEBUG_PTR(omni);

	shared_ptr<OmniWithFeatures> featured { new OmniWithFeatures { omni } };

#ifdef USE_ORB_FEATURE
	Ptr<ORB> orb = ORB::create(1000);
	orb->setFastThreshold(0);
	orb->detectAndCompute(omni->img, *mask, featured->kpts, featured->desc);
#endif

#ifdef USE_AKAZE_FEATURE
	Ptr<AKAZE> akaze = AKAZE::create(AKAZE::DESCRIPTOR_MLDB, 0, 3, 0.0005f, 4, 4, KAZE::DIFF_PM_G2);
	akaze->detectAndCompute(omni->getImage(), *mask, featured->getKeyPoint(), featured->getDesc());
#endif

	return featured;
}
