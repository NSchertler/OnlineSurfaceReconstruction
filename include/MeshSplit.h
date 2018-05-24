#pragma once
#include <Eigen/Core>
#include <iostream>
#include <igl/triangle_triangle_adjacency.h>
#include <queue>
#include <set>
#include <igl/remove_unreferenced.h>

//void splitMesh(Eigen::MatrixXd V, Eigen::MatrixXi F, std::vector<Eigen::MatrixXd>& subVs, std::vector<Eigen::MatrixXi>& subFs, int bound = 64995);

void splitMesh(Eigen::MatrixXd V, Eigen::MatrixXi F, Eigen::MatrixXd C,
	std::vector<Eigen::MatrixXd>& subVs, std::vector<Eigen::MatrixXi>& subFs, std::vector<Eigen::MatrixXd>& subCs,
	int bound = 64995);

int findFirstZero(Eigen::VectorXi v);