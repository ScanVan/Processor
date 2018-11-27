#include "pipelineAlgo.hpp"

shared_ptr<TripletsWithMatches> commonPointsComputation (std::shared_ptr<PairWithMatches> p1, std::shared_ptr<PairWithMatches> p2) {

	std::stringstream ss {};
	ss << "=========================" << std::endl << "Common Points Computation " << p1->idString() << " " << p2->idString() <<  std::endl << "=========================" << std::endl;
	print (ss.str());

	if (p1->imgs[1]->omni->imgNum != p2->imgs[0]->omni->imgNum) {
		throw std::runtime_error ("Error in indexes in CommonPointComputation");
	}

	shared_ptr<TripletsWithMatches> t1 (new TripletsWithMatches({p1->imgs[0], p1->imgs[1], p2->imgs[1]}));
	//DEBUG_PTR(t1);
	const int pairsCount = 2;
	PairWithMatches *p[pairsCount];
	p[0] = p1.get();
	p[1] = p2.get();


	for(size_t p0MatchId = 0;p0MatchId < p[0]->matches.size();p0MatchId++){
		bool ok = true;
		int matchIds[pairsCount+1];
		int nextQueryIdx = p[0]->matches[p0MatchId].trainIdx;

		matchIds[0] = p[0]->matches[p0MatchId].queryIdx;
		matchIds[1] = nextQueryIdx;
		for(size_t pxId = 1;pxId < pairsCount;pxId++){
			PairWithMatches *px = p[pxId];
			ok = false;
			for(size_t pxMatchId = 0;pxMatchId < px->matches.size();pxMatchId++){
				if(px->matches[pxMatchId].queryIdx == nextQueryIdx){
					nextQueryIdx = px->matches[pxMatchId].trainIdx;
					matchIds[pxId+1] = nextQueryIdx;
					ok = true;
					break;
				}
			}
			if(!ok) break;
		}
		if(ok){
			t1->matches.push_back(vector<int>(matchIds, matchIds + pairsCount + 1));
		}
	}



#ifdef DISPLAY_TRIPLET_MATCHES
	{
		RNG rng(12345);
		for(int repeat = 0;repeat < 1;repeat++){
			int w=t1->imgs[0]->omni->img.cols,h=t1->imgs[0]->omni->img.rows;
			Mat res(w, h, CV_8UC3, Scalar(0,0,0));
			t1->imgs[repeat]->omni->img.copyTo(res);
//			for(auto match : t1->matches){
//				circle(res, t1->imgs[0]->kpts[match[0]].pt,10,Scalar(0,255,0),2);
//			}
			for(auto match : t1->matches){
				for(int idx = 0;idx < 2;idx++){
					Scalar color = Scalar(rng.uniform(0,255), rng.uniform(0, 255), rng.uniform(0, 255));
					line(res, t1->imgs[idx]->kpts[match[idx]].pt, t1->imgs[idx + 1]->kpts[match[idx+1]].pt, color, 2);
				}
//				circle(res, t1->imgs[repeat]->kpts[match[repeat]].pt,10,Scalar(0,255,0),2);
			}
			namedWindow( "miaou", WINDOW_KEEPRATIO );
			imshow( "miaou", res);
			waitKey(0);
		}
	}

#endif

#ifdef DISPLAY_TRIPLET_MATCHES_INDIVIDUAL
	{
		RNG rng(12345);
		for(auto match : t1->matches){
			for(int repeat = 0;repeat < 3;repeat++){
				int w=t1->imgs[0]->omni->img.cols,h=t1->imgs[0]->omni->img.rows;
				Mat res(w, h, CV_8UC3, Scalar(0,0,0));
				t1->imgs[repeat]->omni->img.copyTo(res);
				for(int idx = 0;idx < 2;idx++){
					Scalar color = Scalar(rng.uniform(0,255), rng.uniform(0, 255), rng.uniform(0, 255));
					line(res, t1->imgs[idx]->kpts[match[idx]].pt, t1->imgs[idx + 1]->kpts[match[idx+1]].pt, color, 2);
				}

				circle(res, t1->imgs[repeat]->kpts[match[repeat]].pt,10,Scalar(0,255,0),2);

				namedWindow( "miaou", WINDOW_KEEPRATIO );
				imshow( "miaou", res);
				waitKey(0);
			}
		}
	}
#endif


	return t1;
}
