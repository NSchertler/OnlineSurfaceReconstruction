#pragma once
#include <Eigen/Core>
#include <iostream>
#include <string>

typedef unsigned char byte;

class UnityMesh
{
public:
	UnityMesh();
	~UnityMesh();

	void SendName(const std::string& path);

	int vertexCnt;
	Eigen::Vector3f* vertices;
	Eigen::Vector3f* colors;
	int triCnt;
	int* triangles;
	int quadCnt;
	int* quads;

	void SetAmount(int vCnt, int tCnt, int qCnt);
	void AddPointColor(Eigen::Vector3f v, Eigen::Vector3f c);
	void AddTriangles(uint32_t* oneFace);
	void AddQuads(uint32_t* oneFace);
	void SendOut();

	int vcIndex;
	int triIndex;
	int quadIndex;
};
