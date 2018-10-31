#ifndef SRC_TEST_PRIOR_MODEL_NUPLE_HPP_
#define SRC_TEST_PRIOR_MODEL_NUPLE_HPP_

#include "Vec_Points.hpp"
#include <opencv4/opencv2/core/types.hpp>
#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/features2d.hpp>
#include <opencv4/opencv2/imgcodecs.hpp>
#include <opencv4/opencv2/opencv.hpp>


void importation_data(const std::string &path, Vec_Points<double> &data) {
// path is input
// data is output

	std::ifstream ifs (path, std::ifstream::in);

	if (!ifs.is_open())
		throw std::runtime_error ("Data file cannot be opened.");

	std::string line { };
	while (getline(ifs, line)) {
		double d1 { }, d2 { }, d3 { };
		std::stringstream ss { };
		ss << line;
		ss >> d1 >> d2 >> d3;
		data.push_back(d1, d2, d3);
	}
}

void importation_centers (const std::string &path, const int &num, Vec_Points<double> &data) {
// path, num are inputs
// data is output

	std::ifstream ifs (path, std::ifstream::in);

	if (!ifs.is_open())
		throw std::runtime_error ("Data file with centers cannot be opened.");

	std::string line { };
	int counter { 0 };
	while (getline(ifs, line) && (counter < num)) {
		double d1 { }, d2 { }, d3 { };
		std::stringstream ss { };
		ss << line;
		ss >> d1 >> d2 >> d3;
		data.push_back(d1, d2, d3);
		counter++;
	}

}

void projection (const Vec_Points<double> &p3d, const Points<double> &center, Vec_Points<double> &res) {
// p3d, center are inputs
// res is output

	for (size_t i { 0 }; i < p3d.size(); ++i) {
		Points<double> vec { p3d[i] - center };
		vec = vec / (vec.norm());
		res.push_back(vec);
	}
}

void generate_unit_sphere (const Vec_Points<double> &p3d, const Vec_Points<double> &centers, std::vector<Vec_Points<double>> &spheres) {
// p3d, centers are inputs
// spheres is output

	for (size_t i {0}; i < centers.size(); ++i) {
		Vec_Points<double> p3d_proj {};
		projection (p3d, centers[i], p3d_proj);
		spheres.push_back(p3d_proj);
	}
}


void writePly(const std::string &file, const Vec_Points<double> &features){
//	‘__gnu_cxx::__normal_iterator<ModelKeypoint*, std::vector<ModelKeypoint> >
	std::ofstream s {};
	s.open (file);

	s << "ply" << std::endl;
	s << "format ascii 1.0 " << std::endl;
	s << "element vertex " << features.size() << std::endl;
	s << "comment " << file << std::endl;
	s << "property float32 x " << std::endl;
	s << "property float32 y " << std::endl;
	s << "property float32 z " << std::endl;
	s << "end_header " << std::endl;

	for(size_t i {0}; i < features.size(); ++i){
		Points<double> f = features[i];
		s << f[0] << " " << f[1] << " " << f[2] << std::endl;
	}

	s.close();
}

void misaechelle (const Vec_Points<double> &data1, const Vec_Points<double> &data2,
		  	   	  Vec_Points<double> &data1_out, Vec_Points<double> &data2_out) {

	size_t longueur { data1.size() };
	Points<double> zerop { };
	Points<double> trans1 { zerop - data1[0] };
	Points<double> trans2 { zerop - data2[0] };

	if (data1_out.size() < longueur) {
		Points<double> p { };
		for (size_t i { 0 }; i < longueur; ++i) {
			data1_out.push_back(p);
		}
	}
	if (data2_out.size() < longueur) {
		Points<double> p { };
		for (size_t i { 0 }; i < longueur; ++i) {
			data2_out.push_back(p);
		}
	}

	for (size_t i {0}; i < longueur; ++i) {
		data1_out[i] = data1[i] + trans1;
		data2_out[i] = data2[i] + trans2;
	}

	double scale {0};
	for (size_t i{1}; i < longueur; ++i) {
		scale += data1_out[i].norm() / data2_out[i].norm();
	}
	scale /= (longueur-1);
	for (size_t i{1}; i < longueur; ++i) {
		data2_out[i] = data2_out[i] * scale;
	}
}

void superposition (const Vec_Points<double> &data1, const Vec_Points<double> &data2,
					Vec_Points<double> &data1_sp, Vec_Points<double> &data2_sp) {
// data1 and data2 are inputs
// new_data and data2 are outputs

	size_t longueur { data1.size() };
	Vec_Points<double> data1_me { };
	Vec_Points<double> data2_me { };

	misaechelle (data1, data2, data1_me, data2_me);

	Mat_33<double> sv_corr_12 { };
	Points<double> sv_cent_1 { };
	Points<double> sv_cent_2 { };

	for (size_t i{0}; i < longueur; ++i) {
		sv_cent_1 = sv_cent_1 + data1_me[i];
		sv_cent_2 = sv_cent_2 + data2_me[i];
	}
	sv_cent_1 = sv_cent_1 / longueur;
	sv_cent_2 = sv_cent_2 / longueur;

	for (size_t i {0}; i < longueur; ++i) {
		Points<double> sv_diff_1 { data1_me[i] - sv_cent_1 };
		Points<double> sv_diff_2 { data2_me[i] - sv_cent_2 };
		sv_corr_12 = sv_corr_12 + sv_diff_1.outer(sv_diff_2);
	}

	Mat_33<double> svd_Ut_12 {}, svd_V_12 {};
	sv_corr_12.svd(svd_Ut_12, svd_V_12);

	Mat_33<double> rotation {};
	rotation.svd_rotation(svd_V_12, svd_Ut_12);

	Points<double> translation { sv_cent_2 - rotation * sv_cent_1};
	if (data1_sp.size() < longueur) {
		Points<double> p{};
		for (size_t i {data1_sp.size()}; i < longueur; ++i) {
			data1_sp.push_back(p);
		}
	}
	for (size_t i{0}; i < longueur; ++i) {
		Points<double> point { translation + rotation * data1_me[i] };
		data1_sp[i] = point;
	}
	data2_sp = data2_me;
}

void simulation (int num_model, int num_cam, double stop_crit) {

	std::string dirname {std::to_string(num_model)};
	std::string folder { "./prior_generated_test_models/" };
	Vec_Points<double> data {};
	importation_data (folder + dirname + "/model.dat", data);
	Vec_Points<double> centers_ori_1 {};
	importation_centers (folder + dirname + "/centers_R1.txt", num_cam, centers_ori_1);

	std::vector<Vec_Points<double>> spheres {};
	generate_unit_sphere(data, centers_ori_1, spheres);

	std::cout << std::setprecision(12);
	//std::cout << centers_ori_1;

	Vec_Points<double> sv_scene {};
	std::vector<Points<double>> positions {};
	pose_estimation (spheres, stop_crit, sv_scene, positions);

	/*std::cout << "positions: " << std::endl;

	for (const auto &x:positions) {
		std::cout << x << std::endl;
	}*/

	Vec_Points<double> x_0 {};
	for (const auto &x:positions) {
		x_0.push_back(x);
	}
	Vec_Points<double> x_1 { sv_scene };
	Vec_Points<double> centers_ori{}, centers_est{};
	superposition(centers_ori_1, x_0, centers_ori, centers_est);

	//std::cout << "centers_ori" << std::endl;
	//std::cout << centers_ori;


	Vec_Points<double> models_ori{}, models_est{};
	superposition(data, x_1, models_ori, models_est );

	std::string plyFileName1 { "./models_est_" + dirname + "_" + std::to_string(num_cam) + ".ply"};
	writePly(plyFileName1, models_est);
	std::string plyFileName2 { "./models_ori_" + dirname + ".ply" };
	writePly(plyFileName2, models_ori);

	double error {0};
	for (size_t i{0}; i < centers_ori.size(); ++i) {
		error += (centers_est[i]-centers_ori[i]).norm();
	}

	std::cout << num_model << " " << num_cam << " " << stop_crit << "done: error =>" << error << std::endl;
}



#endif /* SRC_TEST_PRIOR_MODEL_NUPLE_HPP_ */
