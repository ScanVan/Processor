//============================================================================
// Name        : Estimation.hpp
// Author      : Marcelo Kaihara
// Version     : 2.0
// Copyright   :
// Description : Pose Estimation algorithm implemented in C++
//============================================================================

#ifndef SRC_ESTIMATION_HPP_
#define SRC_ESTIMATION_HPP_

#include <stdio.h>
#include <iostream>
#include <vector>

// Include files to use OpenCV API
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <experimental/filesystem>
#include <limits>
#include <unistd.h>
#include "Points.hpp"
#include "Mat_33.hpp"
#include "Vec_Points.hpp"
#include "Print_Data.hpp"
#include <cmath>

template <typename T>
void ntuple_consistency (const std::vector<Vec_Points<T>> &p3d_liste,
						 const std::vector<std::vector<T>> &sv_u_liste,
						 T t_tol,
						 const std::vector<std::vector<T>> &sv_e_liste,
						 std::vector<Vec_Points<T>> &p3d_liste_dest,
						 std::vector<std::vector<T>> &sv_u_liste_dest)
// Inputs:
// p3d_liste		: the list of features
// sv_u_liste		: the list of radius of the features
// t_tol			: the tolerance
// Outputs:
// p3d_liste_dest	: the filtered list of features
// sv_u_liste_dest	: the filtered list of radius of the features
{
	// the number of spheres
	size_t nb_sph { p3d_liste.size() }; // this is m
	// the number of features
	size_t nb_pts { p3d_liste[0].size() }; // this is n

	// the mean values of the errors
	std::vector<T> t_m { };
	// calculation of the mean values
	for (size_t i { 0 }; i < nb_sph; ++i) {
		T m { std::accumulate(sv_e_liste[i].begin(), sv_e_liste[i].end(), 0.0) / sv_e_liste[i].size() };
		t_m.push_back(m);
	}

	// the standard deviation of the error
	std::vector<T> t_s { };
	// calculation of the standard deviation
	for (size_t i { 0 }; i < nb_sph; ++i) {
		T var { 0.0 };
		for (const auto & val : sv_e_liste[i]) {
			var += pow(val - t_m[i], 2);
		}
		// check error to avoid division by 0
		if (sv_e_liste[i].size() <= 1) {
			throw(std::runtime_error("Size of the vector of errors is less or equal to 1"));
		}
		var /= (sv_e_liste[i].size() - 1);
		t_s.push_back(sqrt(var) * t_tol);
	}

	// declare new vectors for the computation
	std::vector<Vec_Points<T>> p3d_liste_new { };
	std::vector<std::vector<T>> sv_u_liste_new { };

	// initialize the new vectors with empty elements
	Vec_Points<T> p3d { };
	std::vector<T> v { };
	for (size_t i { 0 }; i < nb_sph; ++i) {
		p3d_liste_new.push_back(p3d);
		sv_u_liste_new.push_back(v);
	}

	for (size_t i { 0 }; i < nb_pts; ++i) {

		bool flag { true };
		// check conditions for all the spheres
		for (size_t j { 0 }; (j < nb_sph) && (flag == true); ++j) {
			flag = sv_e_liste[j][i] <= t_s[j];
		}

		// if it passes all the filtering conditions copy the elements to the new vectors
		if (flag) {
			for (size_t j { 0 }; j < nb_sph; ++j) {
				p3d_liste_new[j].push_back(p3d_liste[j][i]);
				sv_u_liste_new[j].push_back(sv_u_liste[j][i]);
			}
		}
	}

	// copy the results to destination
	p3d_liste_dest = p3d_liste_new;
	sv_u_liste_dest = sv_u_liste_new;

}

template <typename T>
void ntuple_filter (const std::vector<Vec_Points<T>> &p3d_liste,
			     	const std::vector<std::vector<T>> &sv_u_liste,
					T t_tol,
					std::vector<Vec_Points<T>> &p3d_liste_dest,
					std::vector<std::vector<T>> &sv_u_liste_dest)
// Inputs:
// p3d_liste 		: the list of features
// sv_u_liste		: the list of radius of the features
// t_tol			: the tolerance
// Outputs:
// p3d_liste_dest	: the filtered list of features
// sv_u_liste_dest	: the filtered list of radius of the features
{

	// the number of spheres
	size_t nb_sph { p3d_liste.size() }; // this is m
	// the number of features
	size_t nb_pts { p3d_liste[0].size() }; // this is n

	// the mean values of the radius
	std::vector<T> t_m { };
	// calculation of the mean values
	for (size_t i { 0 }; i < nb_sph; ++i) {
		T m { std::accumulate(sv_u_liste[i].begin(), sv_u_liste[i].end(), 0.0) / sv_u_liste[i].size() };
		t_m.push_back(m);
	}

	// the standard deviation of the radius
	std::vector<T> t_s { };
	// calculation of the standard deviation
	for (size_t i { 0 }; i < nb_sph; ++i) {
		T var { 0.0 };
		for (const auto & val : sv_u_liste[i]) {
			var += pow(val - t_m[i], 2);
		}
		// check error to avoid division by 0
		if (sv_u_liste[i].size() <= 1) {
			throw(std::runtime_error("Size of the vector of radius is less or equal to 1"));
		}
		var /= (sv_u_liste[i].size() - 1);
		t_s.push_back(sqrt(var));
	}

	// declare new vectors for the computation
	std::vector<Vec_Points<T>> p3d_liste_new { };
	std::vector<std::vector<T>> sv_u_liste_new { };

	// initialize the new vectors with empty elements
	Vec_Points<T> p3d { };
	std::vector<T> v { };
	for (size_t i { 0 }; i < nb_sph; ++i) {
		p3d_liste_new.push_back(p3d);
		sv_u_liste_new.push_back(v);
	}

	for (size_t i { 0 }; i < nb_pts; ++i) {

		bool flag { true };
		// check conditions for all the spheres
		for (size_t j { 0 }; (j < nb_sph) && (flag == true); ++j) {
			flag = fabs(sv_u_liste[j][i] - t_m[j]) <= t_s[j] * t_tol;
		}

		// if it passes all the filtering conditions copy the elements to the new vectors
		if (flag) {
			for (size_t j { 0 }; j < nb_sph; ++j) {
				p3d_liste_new[j].push_back(p3d_liste[j][i]);
				sv_u_liste_new[j].push_back(sv_u_liste[j][i]);
			}
		}
	}

	// copy the results to destination
	p3d_liste_dest = p3d_liste_new;
	sv_u_liste_dest = sv_u_liste_new;
}

template <typename T>
void optimal_intersection (const std::vector<Points<T>> &t_p, const std::vector<Points<T>> &t_d, Points<T> &t_inter)
// Inputs:
// t_p is number_of_spheres x points
// t_d is number_of_spheres x direction
// Output:
// t_inter is a point
{
	// t_size contains the number of spheres
	size_t t_size { t_p.size() };

	// t_w is a 3x3 matrix initialized to all zeros
	Mat_33<T> t_w { };

	// t_v is a point filled with zeros
	Points<T> t_v { };

	// t_e is 3x3 identity matrix
	Mat_33<T> t_e { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

	// accumulation loop
	for (size_t t_i { 0 }; t_i < t_size; ++t_i) {

		Mat_33<T> t_t { };

		// t_t = t_e - t_d[t_i]' * t_d[t_i]
		// t_d[t_i] is a point
		t_t = t_e - t_d[t_i].outer(t_d[t_i]);

		// accumulation of the w successive components
		t_w = t_w + t_t;

		// accumulation of the v successive components
		// t_t * t_p[t_i] is a 3x3 matrix * point' and returns a point
		t_v = t_v + (t_t * t_p[t_i]);
	}

	// computation of the intersection point

	// t_w.inv() * t_v is a 3x3 matrix * point' and returns a point
	t_inter = t_w.inv() * t_v;
}


template <typename T>
inline void estimation_rot_trans (const std::vector<Vec_Points<T>> &p3d_liste, const std::vector<std::vector<T>> sv_u_liste,
						   std::vector<Mat_33<T>> &sv_r_liste, std::vector<Points<T>> &sv_t_liste)
// Takes as inputs p3d_liste and sv_u_liste,
// and generates outputs sv_r_liste, sv_t_liste
{

	size_t nb_sph { p3d_liste.size() };
	//size_t nb_pts { p3d_liste[0].size() };

	std::vector<Vec_Points<T>> p3d_liste_exp { };

	for (size_t i { 0 }; i < nb_sph; ++i) {
		p3d_liste_exp.push_back(p3d_liste[i] * sv_u_liste[i]);
	}

	std::vector<Points<T>> sv_cent_liste { };
	for (size_t i { 0 }; i < nb_sph; ++i) {
		sv_cent_liste.push_back(p3d_liste_exp[i].mean());
	}

	// Calculates the distances to the centers of the vector of points
	std::vector<Vec_Points<T>> sv_diff_liste { };
	for (size_t i { 0 }; i < nb_sph; ++i) {
		sv_diff_liste.push_back(p3d_liste_exp[i] - sv_cent_liste[i]);
	}

	// Multiply vector of points transposed by vector of points
	std::vector<Mat_33<T>> sv_corr_liste { };
	for (size_t i { 0 }; i < (nb_sph - 1); ++i) {
		sv_corr_liste.push_back(sv_diff_liste[i] * sv_diff_liste[i + 1]);
	}

	for (size_t i { 0 }; i < (nb_sph - 1); ++i) {
		// Matrices for SVD computation
		Mat_33<T> svd_Ut { };
		Mat_33<T> svd_V { };
		Mat_33<T> sv_r { };

		sv_corr_liste[i].svd(svd_Ut, svd_V);
		sv_r_liste[i].svd_rotation(svd_V, svd_Ut);
		sv_t_liste[i] = sv_cent_liste[i + 1] - (sv_r_liste[i] * sv_cent_liste[i]);

	}

}

template <typename T>
inline std::vector<T> intersection_bis (const std::vector<Points<T>> &liste_p, const std::vector<Points<T>> &liste_azim ){

	size_t nb_pts { liste_p.size() };
	Mat_33<T> sum_v { };
	Points<T> sum_vp { };

	for (size_t i { 0 }; i < nb_pts; ++i) {
		T v11 { 1.0 - liste_azim[i][0] * liste_azim[i][0] };
		T v22 { 1.0 - liste_azim[i][1] * liste_azim[i][1] };
		T v33 { 1.0 - liste_azim[i][2] * liste_azim[i][2] };
		T v12 { -liste_azim[i][0] * liste_azim[i][1] };
		T v13 { -liste_azim[i][0] * liste_azim[i][2] };
		T v23 { -liste_azim[i][1] * liste_azim[i][2] };
		sum_v[0][0] = sum_v[0][0] + v11;
		sum_v[0][1] += v12;
		sum_v[0][2] += v13;
		sum_v[1][0] += v12;
		sum_v[1][1] += v22;
		sum_v[1][2] += v23;
		sum_v[2][0] += v13;
		sum_v[2][1] += v23;
		sum_v[2][2] += v33;
		T p1 { liste_p[i][0] };
		T p2 { liste_p[i][1] };
		T p3 { liste_p[i][2] };
		sum_vp[0] += p1 * v11 + p2 * v12 + p3 * v13;
		sum_vp[1] += p1 * v12 + p2 * v22 + p3 * v23;
		sum_vp[2] += p1 * v13 + p2 * v23 + p3 * v33;
	}

	Points<T> inter { sum_v.inv() * sum_vp };

	std::vector<T> rayons { };

	for (size_t i { 0 }; i < nb_pts; ++i) {
		Points<T> centre { liste_p[i] };
		Points<T> azim { liste_azim[i] };
		Points<T> inter_proj { azim * ((inter - centre) * azim) / (azim * azim) };
		T direction { inter_proj * azim };
		if (direction < 0) {
			rayons.push_back(-inter_proj.norm());
		} else {
			rayons.push_back(inter_proj.norm());
		}
	}
	return rayons;
}


template <typename T>
inline void centers_determination (const std::vector<Mat_33<T>> &sv_r_liste, const std::vector<Points<T>> &sv_t_liste,
							std::vector<Points<T>> &center_liste)
// Takes sv_r_liste and sv_t_liste as inputs
// Generates center_liste as output
{
	size_t nb_sph { sv_r_liste.size() + 1 };

	// if center_liste is empty
	if (center_liste.size() < nb_sph) {
		Points<T> zp { };
		for (size_t i { center_liste.size() }; i < nb_sph; ++i) {
			center_liste.push_back(zp);
		}
	}

	for (size_t i { 0 }; i < nb_sph; ++i) {
		Points<T> center = { };
		for (int j { 0 }; j < static_cast<int>(i); ++j) {
			size_t k { i - 1 - j };
			center = (sv_r_liste[k].transpose()) * (center - sv_t_liste[k]);

		}
		center_liste[i] = center;
	}
}

template<typename T>
inline std::vector<Points<T>> azim_determination(std::vector<Points<T>> &azim_liste,
		        								  const std::vector<Mat_33<T>> &sv_r_liste,
				        						  const std::vector<Points<T>> &sv_t_liste) {

	size_t nb_sph { azim_liste.size() };
	for (size_t i { 0 }; i < nb_sph; ++i) {
		for (size_t j { 0 }; j < i; ++j) {
			size_t k { i - 1 - j };
			azim_liste[i] = sv_r_liste[k].transpose() * azim_liste[i];
		}
	}

	return azim_liste;

}


template<typename T>
inline void estimation_rayons_old(const std::vector<Vec_Points<T>> &p3d_liste, std::vector<std::vector<T>> &sv_u_liste, const std::vector<Mat_33<T>> &sv_r_liste,
		const std::vector<Points<T>> &sv_t_liste, std::vector<T> &sv_e_liste) {
// Takes as input p3d_liste, sv_u_liste, sv_r_liste and sv_t_liste
// and generates as output sv_u_liste and sv_e_liste

	size_t nb_sph { p3d_liste.size() };
	size_t nb_pts { p3d_liste[0].size() };

	std::vector<Points<T>> center_liste { };

	// Initialize center_liste
	if (center_liste.size() < nb_sph) {
		Points<T> p {};
		for (size_t i{ center_liste.size() }; i < nb_sph; ++i ) {
			center_liste.push_back(p);
		}
	}

	centers_determination(sv_r_liste, sv_t_liste, center_liste);

	// Initialize sv_e_liste
	if (sv_e_liste.size() < (nb_sph - 1)) {
		for (size_t i { sv_e_liste.size() }; i < (nb_sph - 1); ++i) {
			sv_e_liste.push_back(0);
		}
	} else {
		for (size_t i { 0 }; i < (nb_sph - 1); ++i) {
			sv_e_liste[i] = 0;
		}
	}

	for (size_t j { 0 }; j < nb_pts; ++j) {

		std::vector<Points<T>> azim_liste { };
		for (size_t i { 0 }; i < nb_sph; ++i) {
			azim_liste.push_back(p3d_liste[i][j]);
		}

		azim_determination(azim_liste, sv_r_liste, sv_t_liste);

		std::vector<T> rayons { };
		try {
			rayons = intersection_bis(center_liste, azim_liste);

			for (size_t k { 0 }; k < nb_sph; ++k) {
				sv_u_liste[k][j] = rayons[k];
			}
		} catch (...) {
			for (size_t k { 0 }; k < nb_sph; ++k) {
				rayons[k] = sv_u_liste[k][j];
			}
		}

		std::vector<Points<T>> inter_liste { };

		for (size_t i { 0 }; i < nb_sph; ++i) {
			inter_liste.push_back (center_liste[i] + azim_liste[i] * rayons[i]);
		}

		for (size_t i { 0 }; i < (nb_sph - 1); ++i) {
			Points<T> p { inter_liste[i] - inter_liste[i + 1] };
			T n { p.norm() };
			sv_e_liste[i] = sv_e_liste[i] > n ? sv_e_liste[i] : n;
		}

	}
}

template<typename T>
inline T iteration_error(const std::vector<std::vector<T>> &sv_e_liste,
						 const std::vector<Points<T>> &est_trans) {
// Input:
// features_error		: sv_e_liste, m x n single-value
// estimated translation: est_trans, (m - 1) x Points<T> (vector)
// m is the number of spheres
// n is the number of features
// Output:
// computed error		: a single floating-point value

	// assuming that sv_liste has the correct size for the number of spheres and number of features
	size_t nb_sph { sv_e_liste.size() }; // this is m

	std::vector<T> max_radius_per_sphere { };
	T max_radius_error { };
	for (size_t j{0}; j < nb_sph; ++j) {
		// search the max element of the vector for each sphere
		auto maxe = std::max_element(sv_e_liste[j].begin(), sv_e_liste[j].end());
		// populate the vector containing the max element per sphere
		max_radius_per_sphere.push_back(*maxe);
	}
	// search the max element of the max element per sphere
	auto max_t = std::max_element(max_radius_per_sphere.begin(), max_radius_per_sphere.end());
	max_radius_error = *max_t;

	std::vector<T> min_trans_vec { };
	T min_translation { };
	// Calculates the norm of the translation vectors and stores it in a std::vector
	for (size_t j{0}; j < nb_sph-1; ++j) {
		min_trans_vec.push_back(est_trans[j].norm());
	}
	// searches the min element of the vector of norms of translations
	auto min_t = std::min_element(min_trans_vec.begin(), min_trans_vec.end());
	min_translation = *min_t;

	return max_radius_error/min_translation;

}

template<typename T>
inline void estimation_rayons(const std::vector<Vec_Points<T>> &p3d_liste,
							  std::vector<std::vector<T>> &sv_u_liste,
							  const std::vector<Mat_33<T>> &sv_r_liste,
							  const std::vector<Points<T>> &sv_t_liste,
							  std::vector<std::vector<T>> &sv_e_liste) {
// Input:
// features directions  : p3d_liste, m x Vec_Points (vector of n Points)
// estimated rotation   : sv_r_liste, (m-1) x Mat_33
// estimated translation: sv_t_liste, (m-1) x Points
// m is the number of spheres
// n is the number of features
// Output:
// computed radius      : sv_u_liste, m x n single-value
// computed errors		: sv_e_liste, m x n single-value

	size_t nb_sph { p3d_liste.size() }; // this is m
	size_t nb_pts { p3d_liste[0].size() }; // this is n

	// center of all m spheres in the frame of the first one
	// it contains the sphere centers
	std::vector<Points<T>> center_liste { };

	// Initialize center_liste
	if (center_liste.size() < nb_sph) {
		Points<T> p {};
		for (size_t i{ center_liste.size() }; i < nb_sph; ++i ) {
			center_liste.push_back(p);
		}
	}

	centers_determination(sv_r_liste, sv_t_liste, center_liste);

	// Initialize sv_e_liste
	// creates a null vector of size equal to the number of features
	std::vector<T> nullvec (nb_pts, 0);
	if (sv_e_liste.size() < nb_sph) {
		for (size_t i { sv_e_liste.size() }; i < nb_sph; ++i) {
			sv_e_liste.push_back(nullvec);
		}
	} else {
		// if sv_e_liste.size() is > nb_sph, reset the size to nb_sph
		sv_e_liste = { };
		for (size_t i { 0 }; i < nb_sph; ++i) {
			sv_e_liste.push_back(nullvec);
		}
	}

	// Loop over the n features
	for (size_t j { 0 }; j < nb_pts; ++j) {

		// it stores the current feature direction on each of the m spheres
		std::vector<Points<T>> azim_liste { };

		for (size_t i { 0 }; i < nb_sph; ++i) {
			azim_liste.push_back(p3d_liste[i][j]);
		}

		azim_determination(azim_liste, sv_r_liste, sv_t_liste);

		Points<T> inter { };

		// Calculates the optimal intersection point inter passing the current feature centers and directions
		optimal_intersection(center_liste, azim_liste, inter);

		// The new radius of the feature
		for (size_t k{ 0 }; k < nb_sph; ++k) {
			// Using the optimal intersection (inter), the new radius is computed
			sv_u_liste[k][j] = azim_liste[k] * (inter - center_liste[k]);
		}


		for (size_t k{ 0 }; k < nb_sph; ++k) {
			// the error is computed
			sv_e_liste[k][j] = (center_liste[k] + azim_liste[k] * sv_u_liste[k][j] - inter).norm();
		}

	}

}

template <typename T>
void pose_scene (const std::vector<Vec_Points<T>> &p3d_liste,
			     const std::vector<std::vector<T>> &sv_u_liste,
			     const std::vector<Mat_33<T>> &sv_r_liste,
				 const std::vector<Points<T>> &sv_t_liste,
				 Vec_Points<T> &sv_scene,
				 std::vector<Points<T>> &center_liste) {

	size_t nb_sph { p3d_liste.size() };
	size_t nb_pts { p3d_liste[0].size() };

	centers_determination(sv_r_liste, sv_t_liste, center_liste);

	// If sv_scene empty, fill with zeros
	if (sv_scene.size() < nb_pts) {
		for (size_t i{0}; i < nb_pts; ++i) {
			sv_scene.push_back(0, 0, 0);
		}
	}

	for (size_t j { 0 }; j < nb_pts; ++j) {
		std::vector<Points<T>> azim_liste { };
		for (size_t i { 0 }; i < nb_sph; ++i) {
			azim_liste.push_back(p3d_liste[i][j]);
		}
		azim_liste = azim_determination(azim_liste, sv_r_liste, sv_t_liste);
		std::vector<T> rayons { };
		try {
			rayons = intersection_bis(center_liste, azim_liste);
		} catch (...) {
			for (size_t k { 0 }; k < nb_sph; ++k) {
				rayons[k] = sv_u_liste[k][j];
			}
		}
		std::vector<Points<T>> inter_liste { };

		// Initialize inter_liste
		if (inter_liste.size() < nb_sph) {
			for (size_t i { inter_liste.size() }; i < nb_sph; ++i) {
				Points<T> p { };
				inter_liste.push_back(p);
			}
		}

		for (size_t i { 0 }; i < nb_sph; ++i) {
			inter_liste[i] = center_liste[i] + azim_liste[i] * rayons[i];
		}

		Points<T> inter { };
		for (size_t i {0}; i < inter_liste.size(); ++i) {
			inter = inter + inter_liste[i];
		}
		sv_scene[j] = inter / inter_liste.size();

	}
}


template <typename T>
void pose_estimation (std::vector<Vec_Points<T>> &p3d_liste, const T error_max,
					  Vec_Points<T> &sv_scene,
					  std::vector<Points<T>> &positions) {

	size_t nb_sph = p3d_liste.size();
	size_t nb_pts = p3d_liste[0].size();

	std::vector<std::vector<T>> sv_u_liste { };
	std::vector<Points<T>> sv_t_liste { };
	std::vector<Mat_33<T>> sv_r_liste { };
	std::vector<std::vector<T>> sv_e_liste { };

	// Initialize sv_u_liste
	std::vector<T> ones(nb_pts, 1);
	for (size_t i { 0 }; i < nb_sph; ++i) {
		sv_u_liste.push_back(ones);
	}

	// Initialize sv_r_liste
	for (size_t i { 0 }; i < (nb_sph - 1); ++i) {
		Mat_33<T> m { };
		sv_r_liste.push_back(m);
	}

	// Initialize sv_t_liste;
	for (size_t i { 0 }; i < (nb_sph - 1); ++i) {
		Points<T> p { };
		sv_t_liste.push_back(p);
	}

	T sv_e_old { -1. };

	int counter {0};

	// condition to exit the loop
	bool loop_flag { true };

	while (loop_flag) {

		counter ++;

		estimation_rot_trans(p3d_liste, sv_u_liste, sv_r_liste, sv_t_liste);

		estimation_rayons(p3d_liste, sv_u_liste, sv_r_liste, sv_t_liste, sv_e_liste);

		T sv_e_cur = iteration_error(sv_e_liste, sv_t_liste);

		if (std::abs(sv_e_cur - sv_e_old) < error_max) {
			// stop condition raised, exit while loop
			loop_flag = false;
		}
		else {
			// push error current value
			sv_e_old = sv_e_cur;

			// check error consistency
			ntuple_consistency (p3d_liste, sv_u_liste, 5.0, sv_e_liste, p3d_liste, sv_u_liste);

			// filter out non convergent radius
			ntuple_filter (p3d_liste, sv_u_liste, 5.0, p3d_liste, sv_u_liste);

		}

		std::cout << "Iteration " << std::setfill('0') << std::setw(3) << counter << " : t_norm : ";
		std::cout << std::fixed << std::setprecision(6) << sv_t_liste[0].norm() << ", " << sv_t_liste[1].norm() << " : with " << p3d_liste[0].size()
				<< " features : ";
		std::cout << " mean radius : (" << std::accumulate(sv_u_liste[0].begin(), sv_u_liste[0].end(), 0.0) / sv_u_liste[0].size() << " "
				<< std::accumulate(sv_u_liste[1].begin(), sv_u_liste[1].end(), 0.0) / sv_u_liste[1].size() << " "
				<< std::accumulate(sv_u_liste[2].begin(), sv_u_liste[2].end(), 0.0) / sv_u_liste[2].size() << ")" << std::endl;

	}

	std::cout << sv_r_liste[0] << std::endl;
	std::cout << sv_r_liste[1] << std::endl;

	std::cout << sv_t_liste[0] << std::endl;
	std::cout << sv_t_liste[1] << std::endl;

	// Initialize positions
	if (positions.size() < nb_sph) {
		for (size_t i { 0 }; i < nb_sph; ++i) {
			Points<T> p { };
			positions.push_back(p);
		}
	}

	pose_scene(p3d_liste, sv_u_liste, sv_r_liste, sv_t_liste, sv_scene, positions);

}

#endif /* SRC_ESTIMATION_HPP_ */
