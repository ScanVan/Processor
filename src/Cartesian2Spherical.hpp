#ifndef SRC_CARTESIAN2SPHERICAL_HPP_
#define SRC_CARTESIAN2SPHERICAL_HPP_

#include "Points.hpp"

Points<double> convertCartesian2Spherical (double x, double y, int width, int height) {
// Inputs:
// x 				: cartesian x-coordinate
// y				: cartesian y-coordinate
// width			: width of the equirectangular image
// height			: height of the equirectangular image
// Outputs:
// Points<double>	: converted spherical coordinates in type Points<double>


	/*
	// old implementation
	double theta { (x / width) * 2 * M_PI };
	double phi { M_PI / 2 - (y / (height - 1)) * M_PI };
	double px { -cos(phi) * cos(theta) };
	double py { cos(phi) * sin(theta) };
	double pz { sin(phi) };

	Points<double> p { px, py, pz };

	return p;*/

	/*// Implementation from Nils
	// coordinate re-normalization
	double tm1 { (x / width) * 2 * M_PI };
	double tm2 { (0.5 - (y / height)) * M_PI };

	// coordinate conversion
	double p1 { cos(tm2) * cos(tm1) };
	double p2 { cos(tm2) * sin(tm1) };
	double p3 { sin(tm2) };

	Points<double> p { p1, p2, p3 };
*/

	// Correction from Nils
	double tm1 { (x / width) * 2 * M_PI };
	double tm2 { ((y / (height-1)) - 0.5) * M_PI };

	// coordinate conversion
	double p1 { cos(tm2) * cos(tm1) };
	double p2 { cos(tm2) * sin(tm1) };
	double p3 { sin(tm2) };

	Points<double> p { p1, p2, p3 };



	return p;

}


#endif /* SRC_CARTESIAN2SPHERICAL_HPP_ */
