#include "pipelineAlgo.hpp"

void fusionModel (Model *m1, Model *m2) {
	std::stringstream ss { };
	ss << "=========================" << std::endl
	   << "Fusion Model (" << m2->keypoints.size() << " into " << m1->keypoints.size() << ")" << std::endl
	   << "=========================" << std::endl;
	print(ss.str());
	if(m1->keypoints.empty()){
		m1->features.insert(m1->features.end(), m2->features.begin(), m2->features.end());
		m1->keypoints.insert(m1->keypoints.end(), m2->keypoints.begin(), m2->keypoints.end());
		return;
	}
	assert(m1->keypoints.size() >= 3);
	assert(m2->keypoints.size() == 3);
	auto m1CapturesSize = m1->keypoints.size();
	auto m1C1 = m1->keypoints[m1CapturesSize-2].position;
	auto m1C2 = m1->keypoints[m1CapturesSize-1].position;
	auto m2C1 = m2->keypoints[0].position;
	auto m2C2 = m2->keypoints[1].position;
	auto m2C3 = m2->keypoints[2].position;
	auto scaleFactor = norm(m1C2-m1C1)/norm(m2C2-m2C1);
	auto t1 = (1/norm(m1C2-m1C1)) * (m1C2-m1C1);
	auto t2 = (1/norm(m2C2-m2C1)) * (m2C2-m2C1);
	auto v = cross(t1,t2);
	auto c = t1.dot(t2);
	auto w = Matx33f(0.0,-v(2),v(1),v(2),0.0,-v(0),-v(1),v(0),0.0);
	auto rotation = Matx33f().eye() + w + (1/(1+c))*(w*w);
	auto translation = m1C1-m2C1;

	for(auto source : m2->features){
		auto feature = ModelFeature();
		feature.color = source.color;
		feature.position = translation + scaleFactor * (source.position* rotation);
		m1->features.push_back(feature);
	}
	ModelKeypoint m1C3;
	m1C3.position = translation + scaleFactor * (m2C3*rotation);
	m1->keypoints.push_back(m1C3);
}

