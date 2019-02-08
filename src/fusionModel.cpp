#include "pipelineAlgo.hpp"

void fusionModel (Model *m1, Model *m2) {
	std::stringstream ss { };
	ss << "=========================" << std::endl
	   << "Fusion Model (" << m2->viewPoints.size() << " into " << m1->viewPoints.size() << ")" << std::endl
	   << "=========================" << std::endl;
	print(ss.str());
	if(m1->viewPoints.empty()){
		m1->features.insert(m1->features.end(), m2->features.begin(), m2->features.end());
		m1->viewPoints.insert(m1->viewPoints.end(), m2->viewPoints.begin(), m2->viewPoints.end());
		return;
	}
	assert(m1->viewPoints.size() >= 3);
	assert(m2->viewPoints.size() == 3);
	auto m1CapturesSize = m1->viewPoints.size();
	auto m1C1 = m1->viewPoints[m1CapturesSize-2].position;
	auto m1C2 = m1->viewPoints[m1CapturesSize-1].position;
	auto m2C1 = m2->viewPoints[0].position;
	auto m2C2 = m2->viewPoints[1].position;
	auto m2C3 = m2->viewPoints[2].position;
	auto scaleFactor = norm(m1C2-m1C1)/norm(m2C2-m2C1);
	auto t1 = (1/norm(m1C2-m1C1)) * (m1C2-m1C1);
	auto t2 = (1/norm(m2C2-m2C1)) * (m2C2-m2C1);
	auto v = cross(t1,t2);
	auto c = t1.dot(t2);
	auto w = cv::Matx33f(0.0,-v(2),v(1),v(2),0.0,-v(0),-v(1),v(0),0.0);
	auto rotation = cv::Matx33f().eye() + w + (1/(1+c))*(w*w);
	auto translation = m1C1-m2C1;

	for(auto source : m2->features){
		auto feature = ModelFeature();
		feature.color = source.color;
		feature.position = translation + scaleFactor * (source.position* rotation);
		m1->features.push_back(feature);
	}
	ModelViewPoint m1C3 {};
	m1C3.position = translation + scaleFactor * (m2C3*rotation);
	m1->viewPoints.push_back(m1C3);
}





void fusionModel2 (Model *m1, Model *m2, uint32_t commonViewPointsCount) {
	std::stringstream ss { };
	ss << "=========================" << std::endl
	   << "Fusion Model (" << m2->viewPoints.size() << " into " << m1->viewPoints.size() << ")" << std::endl
	   << "=========================" << std::endl;
	print(ss.str());
	if(m1->viewPoints.empty()){
		m1->features.insert(m1->features.end(), m2->features.begin(), m2->features.end());
		m1->viewPoints.insert(m1->viewPoints.end(), m2->viewPoints.begin(), m2->viewPoints.end());
		return;
	}

	uint32_t vpm1 = m1->viewPoints.size() - commonViewPointsCount;
	double scaleFactor { norm(m1->viewPoints[vpm1+1].position-m1->viewPoints[vpm1].position)
						/ norm(m2->viewPoints[1].position - m2->viewPoints[0].position) };

	for(auto &x : m2->viewPoints) x.position *= scaleFactor;
	for(auto &x : m2->features) x.position *= scaleFactor;

	auto translation = m1->viewPoints[vpm1].position - m2->viewPoints[0].position;

	auto rotation = cv::Matx33f::eye();
	for(uint32_t i = vpm1;i != 0;i--){
		rotation = m1->viewPoints[i].rotationRelative.t() * (rotation);
	}

	for(uint32_t i = commonViewPointsCount;i < m2->viewPoints.size();i++){
		auto vp = m2->viewPoints[i];
		vp.position = translation + (rotation * vp.position.t()).t();
		m1->viewPoints.push_back(vp);
	}

	for(auto f : m2->features){
		f.position = translation + (rotation * f.position.t()).t();
		m1->features.push_back(f);
	}


//	auto m1CapturesSize = m1->viewPoints.size();
//	auto m1C1 = m1->viewPoints[m1CapturesSize-2].position;
//	auto m1C2 = m1->viewPoints[m1CapturesSize-1].position;
//	auto m2C1 = m2->viewPoints[0].position;
//	auto m2C2 = m2->viewPoints[1].position;
//	auto m2C3 = m2->viewPoints[2].position;
//	auto scaleFactor = norm(m1C2-m1C1)/norm(m2C2-m2C1);
//	auto t1 = (1/norm(m1C2-m1C1)) * (m1C2-m1C1);
//	auto t2 = (1/norm(m2C2-m2C1)) * (m2C2-m2C1);
//	auto v = cross(t1,t2);
//	auto c = t1.dot(t2);
//	auto w = cv::Matx33f(0.0,-v(2),v(1),v(2),0.0,-v(0),-v(1),v(0),0.0);
//	auto rotation = cv::Matx33f().eye() + w + (1/(1+c))*(w*w);
//	auto translation = m1C1-m2C1;
//
//	for(auto source : m2->features){
//		auto feature = ModelFeature();
//		feature.color = source.color;
//		feature.position = translation + scaleFactor * (source.position* rotation);
//		m1->features.push_back(feature);
//	}
//	ModelViewPoint m1C3;
//	m1C3.position = translation + scaleFactor * (m2C3*rotation);
//	m1->viewPoints.push_back(m1C3);
}






