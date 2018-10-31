//============================================================================
// Name        : Print_Data.hpp
// Author      : Marcelo Kaihara
// Version     : 1.0
// Copyright   :
// Description : Prints contents of data structure for debug purposes
//============================================================================

#ifndef SRC_PRINT_DATA_HPP_
#define SRC_PRINT_DATA_HPP_

#include <fstream>
#include <iostream>
#include "Vec_Points.hpp"
#include "Mat_33.hpp"

const char * filename = "./data/data_out.txt";

template <typename T>
void print (const std::vector<Vec_Points<T>> &list_vec) {

	std::ofstream file {filename};

	if (!file.is_open()) throw std::runtime_error ("Output file cannot be opened.");

	file.precision(10);
	file << std::fixed;

	for (size_t i {0}; i < list_vec[0].size(); ++i) {
		for (size_t j {0}; j < list_vec.size(); ++j) {
			Points<T> p {list_vec[j][i]};
			file << p[0] << "\t" << p[1] << "\t" << p[2] << "\t\t";
		}
		file << std::endl;
	}

	file.close();
}

template <typename T>
void print (const std::vector<std::vector<T>> &list_s) {

	std::ofstream file {filename};

	if (!file.is_open()) throw std::runtime_error ("Output file cannot be opened.");

	file.precision(10);
	file << std::fixed;

	for (size_t i {0}; i < list_s[0].size(); ++i) {
		for (size_t j {0}; j < list_s.size(); ++j) {
			T d {list_s[j][i]};
			file << d << "\t\t";
		}
		file << std::endl;
	}

	file.close();
}


template <typename T>
void print (const std::vector<Mat_33<T>> &list_m) {

	std::ofstream file {filename};

	if (!file.is_open()) throw std::runtime_error ("Output file cannot be opened.");

	file.precision(10);
	file << std::fixed;

	for (size_t j {0}; j < list_m.size(); ++j) {
			file << list_m[j];
			file << std::endl;
	}

	file.close();
}

template <typename T>
void print (const std::vector<Points<T>> &list_p) {

	std::ofstream file {filename};

	if (!file.is_open()) throw std::runtime_error ("Output file cannot be opened.");

	file.precision(10);
	file << std::fixed;

	for (size_t j {0}; j < list_p.size(); ++j) {
			file << list_p[j];
			file << std::endl;
	}

	file.close();
}

template <typename T>
void print (const std::vector<T> &list_n) {

	std::ofstream file {filename};

	if (!file.is_open()) throw std::runtime_error ("Output file cannot be opened.");

	file.precision(10);
	file << std::fixed;

	for (size_t j {0}; j < list_n.size(); ++j) {
			file << list_n[j];
			file << std::endl;
	}

	file.close();
}

template <typename T>
void print (const Points<T> &p) {

	std::ofstream file {filename};

	if (!file.is_open()) throw std::runtime_error ("Output file cannot be opened.");

	file.precision(10);
	file << std::fixed;

	file << p;

	file.close();
}




#endif /* SRC_PRINT_DATA_HPP_ */
