#include "UnityMesh.h"
#include "zmq/zmqPub.h"

UnityMesh::UnityMesh()
{
	vcIndex = 0;
	triIndex = 0;
	quadIndex = 0;
	zmqPub::getInstance()->connect();
}


UnityMesh::~UnityMesh()
{
	delete[] vertices;
	delete[] colors;
	delete[] triangles;
	delete[] quads;
}

void UnityMesh::SendName(const std::string& path)
{
	std::cout << "UnityMesh::SendName\n";
	zmqPub::getInstance()->send("nm", path);
}

void UnityMesh::SetAmount(int vCnt, int tCnt, int qCnt)
{
	vertexCnt = vCnt;
	triCnt = tCnt;
	quadCnt = qCnt;

	vertices = new Eigen::Vector3f[vertexCnt];
	colors = new Eigen::Vector3f[vertexCnt];
	triangles = new int[triCnt*3];
	quads = new int[quadCnt*4];
}

void UnityMesh::AddPointColor(Eigen::Vector3f v, Eigen::Vector3f c)
{
	vertices[vcIndex] = v;
	colors[vcIndex] = c;
	++vcIndex;
}

void UnityMesh::AddTriangles(uint32_t* oneFace)
{
	triangles[triIndex * 3] = oneFace[0];
	triangles[triIndex * 3 + 1] = oneFace[1];
	triangles[triIndex * 3 + 2] = oneFace[2];
	++triIndex;
}

void UnityMesh::AddQuads(uint32_t* oneFace)
{
	quads[quadIndex * 4] = oneFace[0];
	quads[quadIndex * 4 + 1] = oneFace[1];
	quads[quadIndex * 4 + 2] = oneFace[2];
	quads[quadIndex * 4 + 3] = oneFace[3];
	++quadIndex;
}

void UnityMesh::SendOut()
{
	// serialize this into bytes
	std::cout << "sending mesh:" << vertexCnt << "\t" << triCnt << "\n";
	int len = 4 * 3 + vertexCnt * 3 * 2 * 4 + triCnt * 3 * 4 + quadCnt * 4 * 4;

	byte* msg = new byte[len];
	memcpy(&msg[0], &vertexCnt, 4);
	memcpy(&msg[4], &triCnt, 4);
	memcpy(&msg[8], &quadCnt, 4);
	for (int i = 0; i < vertexCnt; i++) {
		memcpy(&msg[4 * 3 + i * 4 * 6], &vertices[i], 4 * 3);
		memcpy(&msg[4 * 3 + i * 4 * 6 + 4 * 3], &colors[i], 4 * 3);
	}
	for (int i = 0; i < triCnt; i++) {
		memcpy(&msg[4 * 3 + vertexCnt * 4 * 6 + i * 4 * 3], &triangles[i], 4 * 3);
	}
	for (int i = 0; i < quadCnt; i++) {
		memcpy(&msg[4 * 3 + vertexCnt * 4 * 6 + triCnt * 4 * 3 + i * 4 * 4], &quads[i], 4 * 4);
	}
	zmqPub::getInstance()->send("mesh", msg, len);
}
