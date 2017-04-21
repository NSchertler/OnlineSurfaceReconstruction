#pragma once

#include "GLBuffer.h"
#include "GLVertexArray.h"
#include "common.h"
#include "AbstractViewer.h"

//Rendering helper that maintains a set of user-defined selection spheres.
class Selection
{
public:
	Selection();

	void init();

	void draw(const Matrix4f& mv, const Matrix4f& proj, float opacity, int begin = 0, int count = -1, const Vector3f& color = Vector3f(1, 0.8f, 0), bool gradient = true, bool additive = true);

	void addSphere(const Vector3f& position, float radius);
	void addSphere(const Vector4f& positionRadius);
	void resize(size_t);

	Matrix4Xf positions;

	void setFirstToScreenPos(const Eigen::Vector2i& p, float radius, AbstractViewer* viewer);

	void uploadData();

private:

	GLBuffer positionsBuffer;
	GLVertexArray vao;	
};