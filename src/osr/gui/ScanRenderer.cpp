#include "osr/gui/ScanRenderer.h"
#include "osr/gui/ShaderPool.h"
#include "osr/Colors.h"

using namespace osr;
using namespace osr::gui;

ScanRenderer::ScanRenderer()
	: positionBuffer(nse::gui::VertexBuffer), normalBuffer(nse::gui::VertexBuffer), colorBuffer(nse::gui::VertexBuffer), indexBuffer(nse::gui::IndexBuffer)
{
	showInput = true;
	showNormals = false;
}

void ScanRenderer::initialize(Scan& scan)
{	
	inputMesh.generate();

	Matrix3Xf dummy(3, 1);
	gui::ShaderPool::Instance()->ObjectShader.bind();
	inputMesh.bind();
	positionBuffer.uploadData(dummy).bindToAttribute("position");
	normalBuffer.uploadData(dummy).bindToAttribute("normal");
	colorBuffer.uploadData(dummy).bindToAttribute("color");
	indexBuffer.uploadData(scan.F());

	gui::ShaderPool::Instance()->NormalShader.bind();
	positionBuffer.bindToAttribute("position");
	normalBuffer.bindToAttribute("normal");
	inputMesh.unbind();

	updateData(scan);

	indexCount = scan.F().size();
}

void ScanRenderer::updateData(const Scan& scan)
{
	if (scan.V().size() > 0 && scan.N().cols() > 0 && scan.N().cols() < 1e17)
		positionBuffer.uploadData(scan.V());

	if (scan.N().size() > 0 && scan.N().cols() > 0 && scan.N().cols() < 1e17)
		normalBuffer.uploadData(scan.N());

	if (scan.C().size() > 0 && scan.N().cols() > 0 && scan.N().cols() < 1e17)
	{
		Matrix3Xf C_gpu(3, scan.C().cols());
#pragma omp parallel for
		for (int i = 0; i < scan.C().cols(); ++i)
		{
			C_gpu.col(i) = LabToRGB(scan.C().col(i)).cast<float>() / 65535.0f;
		}
		colorBuffer.uploadData(C_gpu);
	}
}

void ScanRenderer::draw(const Scan& scan, const Eigen::Matrix4f & v, const Eigen::Matrix4f & proj) const
{
	if (!showInput && !showNormals)
		return;

	Eigen::Matrix4f mv = v * scan.transform().matrix();
	Eigen::Matrix4f mvp = proj * mv;

	if (showInput)
	{
		glPointSize(5);
		//Draw input Mesh
		auto& shader = gui::ShaderPool::Instance()->ObjectShader;
		shader.bind();
		shader.setUniform("mv", mv);
		shader.setUniform("mvp", mvp);
		inputMesh.bind();
		if (scan.F().size() == 0)
			glDrawArrays(GL_POINTS, 0, scan.V().cols());
		else
			glDrawElements(GL_TRIANGLES, indexCount, GL_UNSIGNED_INT, 0);
		inputMesh.unbind();
	}

	if (showNormals)
	{
		auto& shader = gui::ShaderPool::Instance()->NormalShader;
		shader.bind();
		shader.setUniform("mvp", mvp);
		shader.setUniform("scale", 0.01f * scan.boundingBox().diagonal().norm());
		inputMesh.bind();
		glDrawArrays(GL_POINTS, 0, scan.V().cols());
		inputMesh.unbind();
	}
}