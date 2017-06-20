/*
	This file is part of the implementation for the technical paper

		Field-Aligned Online Surface Reconstruction
		Nico Schertler, Marco Tarini, Wenzel Jakob, Misha Kazhdan, Stefan Gumhold, Daniele Panozzo
		ACM TOG 36, 4, July 2017 (Proceedings of SIGGRAPH 2017)

	Use of this source code is granted via a BSD-style license, which can be found
	in License.txt in the repository root.

	@author Nico Schertler
*/

#include "Selection.h"

#include "ShaderPool.h"

using namespace osr;
using namespace osr::gui;
using namespace osr::gui::tools;

Selection::Selection()
	: positionsBuffer(VertexBuffer)
{ }

void Selection::init()
{
	vao.generate();
	vao.bind();
	positions.resize(4, 1);
	positions.col(0).setConstant(std::numeric_limits<float>::quiet_NaN());
	ShaderPool::Instance()->SphereShader.bind();
	positionsBuffer.uploadData(positions).bindToAttribute("positionRadius");
	vao.unbind();
}

void Selection::resize(size_t n)
{
	positions.resize(4, n);
}

void Selection::draw(const Matrix4f& mv, const Matrix4f& proj, float opacity, int begin, int count, const Vector3f& color, bool gradient, bool additive)
{
	if (count == -1)
		count = positions.cols() - begin;

	GLint saveBlendSrc, saveBlendDst;
	GLboolean saveDepthWriteMask;
	glGetIntegerv(GL_BLEND_SRC_RGB, &saveBlendSrc);
	glGetIntegerv(GL_BLEND_DST_RGB, &saveBlendDst);
	glGetBooleanv(GL_DEPTH_WRITEMASK, &saveDepthWriteMask);
	if (additive)
		glBlendFunc(GL_SRC_ALPHA, GL_ONE);
	else
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glDepthMask(false);

	auto& sphereShader = ShaderPool::Instance()->SphereShader;
	sphereShader.bind();
	vao.bind();
	sphereShader.setUniform("mv", mv);
	sphereShader.setUniform("p", proj);
	sphereShader.setUniform("color", Vector4f(color.x(), color.y(), color.z(), opacity));
	sphereShader.setUniform("gradient", gradient ? 1 : 0);
	glDrawArrays(GL_POINTS, begin, count);
	vao.unbind();

	glDepthMask(saveDepthWriteMask);
	glBlendFunc(saveBlendSrc, saveBlendDst);
}

void Selection::addSphere(const Vector3f & position, float radius)
{
	Vector4f posRadius;
	posRadius.block<3, 1>(0, 0) = position;
	posRadius(3) = radius;
	addSphere(posRadius);
}

void Selection::addSphere(const Vector4f & positionRadius)
{
	positions.conservativeResize(Eigen::NoChange, positions.cols() + 1);
	positions.col(positions.cols() - 1) = positionRadius;
	uploadData();
}

void Selection::setFirstToScreenPos(const Eigen::Vector2i & p, float radius, AbstractViewer* viewer)
{
	Vector4f pos;
	float depth = viewer->get3DPosition(p, pos);	

	if (depth < 1 && depth != 0)
	{		
		pos.w() = radius;
		positions.col(0) = pos;
	}
	else
	{
		positions.col(0).setConstant(std::numeric_limits<float>::quiet_NaN());
	}
	uploadData();
}

void Selection::uploadData()
{
	positionsBuffer.uploadData(positions);
}
