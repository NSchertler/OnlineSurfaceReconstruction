/*
	This file is part of the implementation for the technical paper

		Field-Aligned Online Surface Reconstruction
		Nico Schertler, Marco Tarini, Wenzel Jakob, Misha Kazhdan, Stefan Gumhold, Daniele Panozzo
		ACM TOG 36, 4, July 2017 (Proceedings of SIGGRAPH 2017)

	Use of this source code is granted via a BSD-style license, which can be found
	in License.txt in the repository root.

	@author Nico Schertler
*/

#pragma once

#include <Eigen/Dense>

namespace osr
{
	extern Eigen::Matrix<unsigned short, 3, 1> RGBToLab(const Eigen::Matrix<unsigned short, 3, 1>& rgb);
	extern Eigen::Matrix<unsigned short, 3, 1> LabToRGB(const Eigen::Matrix<unsigned short, 3, 1>& Lab);

	Eigen::Matrix<unsigned char, 3, 1> gammaCorrect(const Eigen::Vector3f& rgbColor);
}