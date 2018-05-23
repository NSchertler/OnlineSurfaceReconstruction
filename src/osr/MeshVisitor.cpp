#include "osr/MeshVisitor.h"
#include "osr/Colors.h"
#include <iostream>

using namespace osr;

WritePLYMeshVisitor::WritePLYMeshVisitor(const std::string& path)
	: ply(path, std::ios::binary)
{ }

void WritePLYMeshVisitor::begin(unsigned int vertices, unsigned int faces)
{
	ply << "ply" << std::endl;
	ply << "format binary_little_endian 1.0" << std::endl;
	ply << "element vertex " << vertices << std::endl;
	ply << "property float32 x" << std::endl;
	ply << "property float32 y" << std::endl;
	ply << "property float32 z" << std::endl;
	ply << "property uchar red" << std::endl;
	ply << "property uchar green" << std::endl;
	ply << "property uchar blue" << std::endl;
	ply << "element face " << faces << std::endl;
	ply << "property list uint8 int32 vertex_index" << std::endl;
	ply << "end_header" << std::endl;
}

void WritePLYMeshVisitor::addVertex(const Eigen::Vector3f& pos, const Eigen::Vector3f& color)
{
	ply.write(reinterpret_cast<const char*>(pos.data()), 3 * sizeof(float));
	auto gammaCorrectedColor = osr::gammaCorrect(color);
	//Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
	//std::cout << "debug: color " << color << " gamma color:" << gammaCorrectedColor.format(CleanFmt) << "\n";
	//std::cout << "debug: color " << color << " gamma color:" << (int)gammaCorrectedColor[0] << " " << (int)gammaCorrectedColor[1] << " " << (int)gammaCorrectedColor[2] << "\n";
	ply.write(reinterpret_cast<const char*>(gammaCorrectedColor.data()), 3 * sizeof(unsigned char));
}

void WritePLYMeshVisitor::addFace(unsigned int count, const uint32_t* indices)
{
	ply.write(reinterpret_cast<const char*>(&count), 1);
	ply.write(reinterpret_cast<const char*>(indices), sizeof(uint32_t) * count);
}

void WritePLYMeshVisitor::end()
{
	ply.close();
}