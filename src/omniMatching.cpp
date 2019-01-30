#include "pipelineAlgo.hpp"

#include "gms_matcher.h"

std::shared_ptr<PairWithMatches> omniMatching (std::shared_ptr<OmniWithFeatures> im1, std::shared_ptr<OmniWithFeatures> im2) {
	std::stringstream ss {};
	ss << "=========================" << std::endl << "Feature Extraction (" << im1->idString() << " "  << im2->idString() << ")" << std::endl << "=========================" << std::endl;
	print (ss.str());

	std::shared_ptr<PairWithMatches> p1(new  PairWithMatches{im1, im2});
	//DEBUG_PTR(p1);

#ifdef USE_KNN_MATCHER
    BFMatcher matcher(NORM_HAMMING);
    vector<vector<DMatch>> nn_matches;
    matcher.knnMatch(im1->desc, im2->desc, nn_matches, 2);

	const float nn_match_ratio = 0.8f;   // Nearest neighbor matching ratio
    for(auto matches : nn_matches) {
        float dist1 = matches[0].distance;
        float dist2 = matches[1].distance;

        if(dist1 < nn_match_ratio * dist2) {
            p1->matches.push_back(matches[0]);
        }
    }
#endif

#ifdef USE_GMS_FILTER
    vector<DMatch> matches_all;
    BFMatcher matcher(NORM_HAMMING);
	matcher.match(im1->getDesc(), im2->getDesc(), matches_all);

	// GMS filter
	std::vector<bool> vbInliers;
	gms_matcher gms(im1->getKeyPoint(), (im1->getOmni()->getImage()).size(), im2->getKeyPoint(), (im2->getOmni()->getImage()).size(), matches_all);
	int num_inliers = gms.GetInlierMask(vbInliers, false, false);
	cout << "Get total " << num_inliers << " matches." << endl;

	// collect matches
	for (size_t i = 0; i < vbInliers.size(); ++i)
	{
		if (vbInliers[i] == true)
		{
			p1->matches.push_back(matches_all[i]);
		}
	}
#endif

	return p1;
}

