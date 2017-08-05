/*
	This file is part of the implementation for the technical paper

		Field-Aligned Online Surface Reconstruction
		Nico Schertler, Marco Tarini, Wenzel Jakob, Misha Kazhdan, Stefan Gumhold, Daniele Panozzo
		ACM TOG 36, 4, July 2017 (Proceedings of SIGGRAPH 2017)

	Use of this source code is granted via a BSD-style license, which can be found
	in License.txt in the repository root.

	@author Nico Schertler
*/

#include "osr/Colors.h"

#include <algorithm>
#include <iostream>

using namespace osr;

const double delta = 6.0 / 29.0;
const double delta3 = delta * delta * delta;

//reference white
double Xn = 62289;
double Yn = 65535;
double Zn = 71356;

double f(double t)
{
	if (t > delta3)
		return pow(t, 1.0 / 3.0);
	else
		return t / (3 * delta * delta) + 4.0 / 29.0;
}

double fInv(double t)
{
	if (t > delta)
		return t * t * t;
	else
		return 3 * delta * delta * (t - 4.0 / 29.0);
}

Eigen::Matrix<unsigned short, 3, 1> osr::RGBToLab(const Eigen::Matrix<unsigned short, 3, 1>& rgb)
{
	Eigen::Matrix3d mRGBToXYZ;
	mRGBToXYZ <<
		0.4124564, 0.3575761, 0.1804375,
		0.2126729, 0.7151522, 0.0721750,
		0.0193339, 0.1191920, 0.9503041;

	auto rgbDbl = rgb.cast<double>();
	auto XYZ = mRGBToXYZ * rgbDbl;

	double fx = f(XYZ.x() / Xn);
	double fy = f(XYZ.y() / Yn);
	double fz = f(XYZ.z() / Zn);

	double L = 116 * fy - 16;
	double a = 500 * (fx - fy);
	double b = 200 * (fy - fz);

	L = std::max(0.0, std::min(65535.0, 655.35 * L));
	a = std::max(0.0, std::min(65535.0, 327.67 * a + 32768));
	b = std::max(0.0, std::min(65535.0, 327.67 * b + 32768));

	return Eigen::Matrix<unsigned short, 3, 1>((unsigned short)L, (unsigned short)a, (unsigned short)b);
}


Eigen::Matrix<unsigned short, 3, 1> osr::LabToRGB(const Eigen::Matrix<unsigned short, 3, 1>& Lab)
{
	double L = Lab.x() / 655.35;
	double a = (Lab.y() - 32768) / 327.67;
	double b = (Lab.z() - 32768) / 327.67;

	Eigen::Vector3d XYZ(Xn * fInv((L + 16) / 116.0 + a / 500.0), Yn * fInv((L + 16) / 116.0), Zn * fInv((L + 16) / 116.0 - b / 200.0));	

	Eigen::Matrix3d mXYZToRGB;
	mXYZToRGB <<
		3.2404542, -1.5371385, -0.4985314,
		-0.9692660, 1.8760108, 0.0415560,
		0.0556434, -0.2040259, 1.0572252;

	Eigen::Vector3d rgbDbl = mXYZToRGB * XYZ;

	for (int i = 0; i < 3; ++i)
		rgbDbl(i) = std::max(0.0, std::min(65535.0, rgbDbl(i)));

	return rgbDbl.cast<unsigned short>();
}

Eigen::Matrix<unsigned char, 3, 1> osr::gammaCorrect(const Eigen::Vector3f& rgbColor)
{
	float gamma = 1.0f / 2.2f;

	Eigen::Matrix<unsigned char, 3, 1> r;
	for (int i = 0; i < 3; ++i)
		r(i) = (unsigned char)(std::pow((float)rgbColor(i), gamma) * 255.0f);
	return r;
}