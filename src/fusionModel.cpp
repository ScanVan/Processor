#include "pipelineAlgo.hpp"

//void fusionModel (Model *m1, Model *m2) {
//	std::stringstream ss { };
//	ss << "=========================" << std::endl
//	   << "Fusion Model (" << m2->viewPoints.size() << " into " << m1->viewPoints.size() << ")" << std::endl
//	   << "=========================" << std::endl;
//	print(ss.str());
//	if(m1->viewPoints.empty()){
//		m1->features.insert(m1->features.end(), m2->features.begin(), m2->features.end());
//		m1->viewPoints.insert(m1->viewPoints.end(), m2->viewPoints.begin(), m2->viewPoints.end());
//		return;
//	}
//	assert(m1->viewPoints.size() >= 3);
//	assert(m2->viewPoints.size() == 3);
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
//	ModelViewPoint m1C3 {};
//	m1C3.position = translation + scaleFactor * (m2C3*rotation);
//	m1->viewPoints.push_back(m1C3);
//}





void fusionModel2 (Model *m1, Model *m2, uint32_t commonViewPointsCount) {
// Make a fusion of the models
// Input:
// m2						: Pointer to the model to add (triplet)
// commonViewPointsCount	: The number of triplets in common
// Output:
// m1						: The merged model

	std::stringstream ss { };
	ss << "=========================" << std::endl
	   << "Fusion Model (" << m2->viewPoints.size() << " into " << m1->viewPoints.size() << ")" << std::endl
	   << "=========================" << std::endl;
	print(ss.str());

	// if the the destination model is empty, copy the camera positions and the points and the image names
	if(m1->viewPoints.empty()){
		m1->features.insert(m1->features.end(), m2->features.begin(), m2->features.end());
		m1->viewPoints.insert(m1->viewPoints.end(), m2->viewPoints.begin(), m2->viewPoints.end());
		m1->imgNames.insert(m1->imgNames.end(), m2->imgNames.begin(), m2->imgNames.end());
		return;
	}

	// vpm1 is the number of camera positions of the destination model minus the overlapped number of camera positions
	uint32_t vpm1 = m1->viewPoints.size() - commonViewPointsCount;
	// Calculates the scale factor based on the distances of the camera positions on the overlapped portion
	// If m1->viewPoints.size() = 5, commonViewPointsCount = 2 (triplets)
	// vpm1 = 5 - 2 = 3
	// m1: p0	p1	p2	p3	p4
	// m2:              q0	q1	q2
	// scaleFactor = distance (p4, p3) / distance (q1, q0)
	double scaleFactor { norm(m1->viewPoints[vpm1+1].position-m1->viewPoints[vpm1].position)
						/ norm(m2->viewPoints[1].position - m2->viewPoints[0].position) };

	// scale the camera positions and the feature points of the model to add
	for (auto &x : m2->viewPoints)
		x.position *= scaleFactor;
	for (auto &x : m2->features)
		x.position *= scaleFactor;

	// calculates the translation between p3 and q0
	auto translation = m1->viewPoints[vpm1].position - m2->viewPoints[0].position;

	// calculates the absolute rotation of p3 with respect to p0 chaining the rotation matrices
	// p0 p1 p2
	//    q0 q1 q2
	//       r0 r1 r2
	//          s0 s1 s2
	auto rotation = m1->viewPoints[vpm1].rotationAbsolute;
//	auto rotation = cv::Matx33d::eye();
//	for (uint32_t i = vpm1; i != 0; i--) {
//		rotation = m1->viewPoints[i].rotationRelative.t() * (rotation);
//	}

	// calculates the camera positions of the non overlapped part of the model to add
	for (uint32_t i = commonViewPointsCount; i < m2->viewPoints.size(); i++) {
		// vp is the camera position q2
		auto vp = m2->viewPoints[i];
		// relative rotation is not altered
		// the absolute rotation of the camera point to add is the absolute rotation of m1
		vp.rotationAbsolute = (m1->viewPoints[vpm1 + i -1].rotationAbsolute) * (m2->viewPoints[i].rotationRelative).t();
		// the position of the new camera to add (non overlapped ones)
		vp.position = translation + (rotation * vp.position.t()).t();
		m1->viewPoints.push_back(vp);
		m1->imgNames.push_back(m2->imgNames[i]);
	}

	// calculates the new features to add with respect to the model to merge
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






