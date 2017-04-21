#pragma once

#include <Eigen/Dense>

extern Eigen::Matrix<unsigned short, 3, 1> RGBToLab(const Eigen::Matrix<unsigned short, 3, 1>& rgb);
extern Eigen::Matrix<unsigned short, 3, 1> LabToRGB(const Eigen::Matrix<unsigned short, 3, 1>& Lab);