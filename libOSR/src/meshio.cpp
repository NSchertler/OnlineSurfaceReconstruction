/*
	This file is part of the implementation for the technical paper

		Field-Aligned Online Surface Reconstruction
		Nico Schertler, Marco Tarini, Wenzel Jakob, Misha Kazhdan, Stefan Gumhold, Daniele Panozzo
		ACM TOG 36, 4, July 2017 (Proceedings of SIGGRAPH 2017)

	Use of this source code is granted via a BSD-style license, which can be found
	in License.txt in the repository root.

	@author Wenzel Jakob
	@author Nico Schertler
*/

#include "osr/meshio.h"
#include <nsessentials/data/FileHelper.h>
#include <unordered_map>
#include <fstream>
#include <map>
#if !defined(_WIN32)
#include <libgen.h>
#endif

#ifdef OSR_USE_MEMORY_MAPPED_FILE
#include <boost/iostreams/device/mapped_file.hpp>
#endif
#include <boost/spirit/include/qi.hpp>
#include <boost/lambda/lambda.hpp>

#define STB_IMAGE_IMPLEMENTATION
#include "3rd/stb_image.h"

#include <nsessentials/util/Timer.h>
#include "osr/Colors.h"

extern "C" {
    #include "rply.h"
}

using namespace osr;

inline uint32_t str_to_uint32_t(const std::string &str) {
	char *end_ptr = nullptr;
	uint32_t result = (uint32_t)strtoul(str.c_str(), &end_ptr, 10);
	if (*end_ptr != '\0')
		throw std::runtime_error("Could not parse unsigned integer \"" + str + "\"");
	return result;
}

inline uint32_t str_to_int32_t(const std::string &str) {
	char *end_ptr = nullptr;
	int32_t result = (int32_t)strtol(str.c_str(), &end_ptr, 10);
	if (*end_ptr != '\0')
		throw std::runtime_error("Could not parse signed integer \"" + str + "\"");
	return result;
}

inline Float str_to_float(const std::string &str) {
	char *end_ptr = nullptr;
	Float result = (Float)strtod(str.c_str(), &end_ptr);
	if (*end_ptr != '\0')
		throw std::runtime_error("Could not parse floating point value \"" + str + "\"");
	return result;
}

inline std::vector<std::string> &str_tokenize(const std::string &s, char delim, std::vector<std::string> &elems, bool include_empty = false) {
	std::stringstream ss(s);
	std::string item;
	while (std::getline(ss, item, delim))
		if (!item.empty() || include_empty)
			elems.push_back(item);
	return elems;
}

inline std::vector<std::string> str_tokenize(const std::string &s, char delim, bool include_empty) {
	std::vector<std::string> elems;
	str_tokenize(s, delim, elems, include_empty);
	return elems;
}

void osr::write_mesh(const std::string &filename, const MatrixXu &F,
                const Matrix3Xf &V, const Matrix3Xf &N, const Matrix3Xf &Nf,
                const Matrix3Xf &UV, const Matrix3Xf &C) 
{
    std::string extension;
    if (filename.size() > 4)
        extension = nse::data::str_tolower(filename.substr(filename.size()-4));

    if (extension == ".ply")
        write_ply(filename, F, V, N, Nf, UV, C);
    else if (extension == ".obj")
        write_obj(filename, F, V, N, Nf, UV, C);
    else
        throw std::runtime_error("write_mesh: Unknown file extension \"" + extension + "\" (.ply/.obj are supported)");
}

void osr::load_ply(const std::string &filename, MatrixXu &F, Matrix3Xf &V,
              Matrix3Xf &N, Matrix3Xus &C, bool pointcloud)
{
    auto message_cb = [](p_ply ply, const char *msg) { std::cerr << "rply: " << msg << std::endl; };

    nse::util::Timer<> timer;
    std::cout << "Loading \"" << filename << "\" .. ";
    std::cout.flush();

    p_ply ply = ply_open(filename.c_str(), message_cb, 0, nullptr);
    if (!ply)
        throw std::runtime_error("Unable to open PLY file \"" + filename + "\"!");

    if (!ply_read_header(ply)) {
        ply_close(ply);
        throw std::runtime_error("Unable to open PLY header of \"" + filename + "\"!");
    }

	const float gamma = 2.2f;

    p_ply_element element = nullptr;
    uint32_t vertexCount = 0, faceCount = 0;

	p_ply_property prop = nullptr;
	const char* pname;
	e_ply_type type, length, value;

	bool hasUv = false;
	bool hasNormals = false;
	bool hasColor = false;

    /* Inspect the structure of the PLY file, load number of faces if avaliable */
    while ((element = ply_get_next_element(ply, element)) != nullptr) {
        const char *name;
        long nInstances;

        ply_get_element_info(element, &name, &nInstances);
		if (!strcmp(name, "vertex"))
		{
			vertexCount = (uint32_t)nInstances;			
			while ((prop = ply_get_next_property(element, prop)) != nullptr)
			{
				ply_get_property_info(prop, &pname, &type, &length, &value);
				if (!strcmp(pname, "texture_u"))
					hasUv = true;
				else if (!strcmp(pname, "nx"))
					hasNormals = true;
				else if (!strcmp(pname, "red"))
					hasColor = true;
			}
		}
        else if (!strcmp(name, "face"))
            faceCount = (uint32_t) nInstances;
    }

	if (vertexCount == 0 && faceCount == 0)
		throw std::runtime_error("PLY file \"" + filename + "\" is invalid! No face/vertex/elements found!");
	else if (faceCount == 0)
		pointcloud = true;

    F.resize(3, faceCount);
    V.resize(3, vertexCount);
	N.resize(3, hasNormals ? vertexCount : 0);
	C.resize(3, hasColor ? vertexCount : 0);
	Eigen::Matrix<float, 2, Eigen::Dynamic> uv(2, hasUv ? vertexCount : 0);


    struct VertexCallbackData {
        Matrix3Xf &V;
        VertexCallbackData(Matrix3Xf &V)
            : V(V) { }
    };

    struct FaceCallbackData {
        MatrixXu &F;
        FaceCallbackData(MatrixXu &F)
            : F(F) { }
    };

    struct VertexNormalCallbackData {
        Matrix3Xf &N;
        VertexNormalCallbackData(Matrix3Xf &_N)
            : N(_N) { }
    };
	
	struct VertexUVCallbackData {
		Eigen::Matrix<float, 2, Eigen::Dynamic> &uv;
		VertexUVCallbackData(Eigen::Matrix<float, 2, Eigen::Dynamic> &_uv)
			: uv(_uv) { }
	};

	struct VertexColorCallbackData {
		Eigen::Matrix<unsigned short, 3, Eigen::Dynamic> &c;
		VertexColorCallbackData(Eigen::Matrix<unsigned short, 3, Eigen::Dynamic> &c)
			: c(c) { }
	};

    auto rply_vertex_cb = [](p_ply_argument argument) -> int {
        VertexCallbackData *data; long index, coord;
        ply_get_argument_user_data(argument, (void **) &data, &coord);
        ply_get_argument_element(argument, nullptr, &index);
        data->V(coord, index) = (Float) ply_get_argument_value(argument);
        return 1;
    };

    auto rply_vertex_normal_cb = [](p_ply_argument argument) -> int {
        VertexNormalCallbackData *data; long index, coord;
        ply_get_argument_user_data(argument, (void **) &data, &coord);
        ply_get_argument_element(argument, nullptr, &index);
        data->N(coord, index) = (Float) ply_get_argument_value(argument);
        return 1;
    };

	auto rply_vertex_uv_cb = [](p_ply_argument argument) -> int 
	{
		VertexUVCallbackData *data; long index, coord;
		ply_get_argument_user_data(argument, (void **)&data, &coord);
		ply_get_argument_element(argument, nullptr, &index);
		data->uv(coord, index) = (Float)ply_get_argument_value(argument);
		return 1;
	};

	auto rply_vertex_color_cb = [](p_ply_argument argument) -> int
	{
		VertexColorCallbackData *data; long index, coord;
		ply_get_argument_user_data(argument, (void **)&data, &coord);
		ply_get_argument_element(argument, nullptr, &index);
		auto colorInPly = ply_get_argument_value(argument);
		data->c(coord, index) = (unsigned short)(std::pow(colorInPly / 255.0, 2.2) * 65535);
		return 1;
	};

    auto rply_index_cb = [](p_ply_argument argument) -> int {
        FaceCallbackData *data;
        long length, value_index, index;
        ply_get_argument_property(argument, nullptr, &length, &value_index);

        if (length != 3)
            throw std::runtime_error("Only triangle faces are supported!");

        ply_get_argument_user_data(argument, (void **) &data, nullptr);
        ply_get_argument_element(argument, nullptr, &index);

        if (value_index >= 0)
            data->F(value_index, index) = (uint32_t) ply_get_argument_value(argument);

        return 1;
    };

    VertexCallbackData vcbData(V);
    FaceCallbackData fcbData(F);
    VertexNormalCallbackData vncbData(N);
	VertexUVCallbackData vuvcbData(uv);
	VertexColorCallbackData vcData(C);

	ply_set_read_cb(ply, "vertex", "x", rply_vertex_cb, &vcbData, 0);
	ply_set_read_cb(ply, "vertex", "y", rply_vertex_cb, &vcbData, 1);
	ply_set_read_cb(ply, "vertex", "z", rply_vertex_cb, &vcbData, 2);    

	ply_set_read_cb(ply, "vertex", "nx", rply_vertex_normal_cb, &vncbData, 0);
	ply_set_read_cb(ply, "vertex", "ny", rply_vertex_normal_cb, &vncbData, 1);
	ply_set_read_cb(ply, "vertex", "nz", rply_vertex_normal_cb, &vncbData, 2);

    if (faceCount > 0)
	{                   
        if (!ply_set_read_cb(ply, "face", "vertex_index", rply_index_cb, &fcbData, 0) && !ply_set_read_cb(ply, "face", "vertex_indices", rply_index_cb, &fcbData, 0))
		{
			ply_close(ply);
			throw std::runtime_error("PLY file \"" + filename + "\" does not contain vertex indices!");
        }
    }

	ply_set_read_cb(ply, "vertex", "texture_u", rply_vertex_uv_cb, &vuvcbData, 0);
	ply_set_read_cb(ply, "vertex", "texture_v", rply_vertex_uv_cb, &vuvcbData, 1);

	ply_set_read_cb(ply, "vertex", "red", rply_vertex_color_cb, &vcData, 0);
	ply_set_read_cb(ply, "vertex", "green", rply_vertex_color_cb, &vcData, 1);
	ply_set_read_cb(ply, "vertex", "blue", rply_vertex_color_cb, &vcData, 2);

    if (!ply_read(ply)) {
        ply_close(ply);
        throw std::runtime_error("Error while loading PLY data from \"" + filename + "\"!");
    }

    ply_close(ply);

	if (hasUv && !hasColor)
	{
		std::string texturePath = filename;
		//change extension from ply to png
		texturePath[texturePath.length() - 2] = 'n';
		texturePath[texturePath.length() - 1] = 'g';

		if (nse::data::file_exists(texturePath))
		{
			std::cout << "loading texture .. ";			
			C.resizeLike(V);
			using handleType = std::unique_ptr<uint8_t[], void(*)(void*)>;
			int w, h, n;
			handleType textureData(stbi_load(texturePath.c_str(), &w, &h, &n, 3), stbi_image_free);
			if (textureData)
			{	
#pragma omp parallel for
				for (int i = 0; i < V.cols(); ++i)
				{
					auto vuv = uv.col(i);
					int x = (int)std::floor(w * vuv.x());
					int y = (int)std::floor(h * (1 - vuv.y()));

					if (x >= w)
						x = w - 1;
					if (y >= h)
						y = h - 1;

					int offset = n * (x + w * y);

					//Gamma de-correction

					C.col(i) <<
						(uint16_t)(std::pow((float)textureData.get()[offset + 0] / 255.0f, gamma) * 65535.0f),
						(uint16_t)(std::pow((float)textureData.get()[offset + 1] / 255.0f, gamma) * 65535.0f), 
						(uint16_t)(std::pow((float)textureData.get()[offset + 2] / 255.0f, gamma) * 65535.0f);
				}
			}
		}
	}

#pragma omp parallel for
	for(int i = 0; i < C.cols(); ++i)
	{
		C.col(i) = RGBToLab(C.col(i));
	}

    std::cout << "done. (V=" << vertexCount;
    if (faceCount > 0)
		std::cout << ", F=" << faceCount;
	std::cout << ", took " << nse::util::timeString(timer.value()) << ")" << std::endl;
}

void osr::write_ply(const std::string &filename, const MatrixXu &F,
               const Matrix3Xf &V, const Matrix3Xf &N, const Matrix3Xf &Nf, const Matrix3Xf &UV,
               const Matrix3Xf &C)
{
    auto message_cb = [](p_ply ply, const char *msg) { std::cerr << "rply: " << msg << std::endl; };

    nse::util::Timer<> timer;
	std::cout << "Writing \"" << filename << "\" (V=" << V.cols()
         << ", F=" << F.cols() << ") .. ";
	std::cout.flush();

    if (N.size() > 0 && Nf.size() > 0)
        throw std::runtime_error("Please specify either face or vertex normals but not both!");

    p_ply ply = ply_create(filename.c_str(), PLY_DEFAULT, message_cb, 0, nullptr);
    if (!ply)
        throw std::runtime_error("Unable to write PLY file!");

    ply_add_comment(ply, "Generated by Instant Meshes");
    ply_add_element(ply, "vertex", V.cols());
    ply_add_scalar_property(ply, "x", PLY_FLOAT);
    ply_add_scalar_property(ply, "y", PLY_FLOAT);
    ply_add_scalar_property(ply, "z", PLY_FLOAT);

    if (N.size() > 0) {
        ply_add_scalar_property(ply, "nx", PLY_FLOAT);
        ply_add_scalar_property(ply, "ny", PLY_FLOAT);
        ply_add_scalar_property(ply, "nz", PLY_FLOAT);
        if (N.cols() != V.cols() || N.rows() != 3)
            throw std::runtime_error("Vertex normal matrix has incorrect size");
    }

    if (UV.size() > 0) {
        ply_add_scalar_property(ply, "u", PLY_FLOAT);
        ply_add_scalar_property(ply, "v", PLY_FLOAT);
        if (UV.cols() != V.cols() || UV.rows() != 2)
            throw std::runtime_error("Texture coordinate matrix has incorrect size");
    }

    if (C.size() > 0) {
        ply_add_scalar_property(ply, "red", PLY_FLOAT);
        ply_add_scalar_property(ply, "green", PLY_FLOAT);
        ply_add_scalar_property(ply, "blue", PLY_FLOAT);
        if (C.cols() != V.cols() || (C.rows() != 3 && C.rows() != 4))
            throw std::runtime_error("Color matrix has incorrect size");
    }

    /* Check for irregular faces */
    std::map<uint32_t, std::pair<uint32_t, std::map<uint32_t, uint32_t>>> irregular;
    size_t nIrregular = 0;
    if (F.rows() == 4) {
        for (uint32_t f=0; f<F.cols(); ++f) {
            if (F(2, f) == F(3, f)) {
                nIrregular++;
                auto &value = irregular[F(2, f)];
                value.first = f;
                value.second[F(0, f)] = F(1, f);
            }
        }
    }

    ply_add_element(ply, "face", F.cols() - nIrregular + irregular.size());
    ply_add_list_property(ply, "vertex_indices", PLY_UINT8, PLY_INT);
    if (Nf.size() > 0) {
        ply_add_scalar_property(ply, "nx", PLY_FLOAT);
        ply_add_scalar_property(ply, "ny", PLY_FLOAT);
        ply_add_scalar_property(ply, "nz", PLY_FLOAT);
        if (Nf.cols() != F.cols() || Nf.rows() != 3)
            throw std::runtime_error("Face normal matrix has incorrect size");
    }
    ply_write_header(ply);

    for (uint32_t j=0; j<V.cols(); ++j) {
        for (uint32_t i=0; i<V.rows(); ++i)
            ply_write(ply, V(i, j));
        if (N.size() > 0) {
            for (uint32_t i=0; i<N.rows(); ++i)
                ply_write(ply, N(i, j));
        }
        if (UV.size() > 0) {
            for (uint32_t i=0; i<UV.rows(); ++i)
                ply_write(ply, UV(i, j));
        }
        if (C.size() > 0) {
            for (uint32_t i=0; i<std::min(3u, (uint32_t) C.rows()); ++i)
                ply_write(ply, C(i, j));
        }
    }

    for (uint32_t f=0; f<F.cols(); ++f) {
        if (F.rows() == 4 && F(2, f) == F(3, f))
            continue;
        ply_write(ply, F.rows());
        for (uint32_t i=0; i<F.rows(); ++i)
            ply_write(ply, F(i, f));
        if (Nf.size() > 0) {
            for (uint32_t i=0; i<Nf.rows(); ++i)
                ply_write(ply, Nf(i, f));
        }
    }

    for (auto item : irregular) {
        auto face = item.second;
        uint32_t v = face.second.begin()->first, first = v;
        ply_write(ply, face.second.size());
        while (true) {
            ply_write(ply, v);
            v = face.second[v];
            if (v == first)
                break;
        }
        if (Nf.size() > 0) {
            for (uint32_t i=0; i<Nf.rows(); ++i)
                ply_write(ply, Nf(i, face.first));
        }
    }

    ply_close(ply);
	std::cout << "done. (";
    if (irregular.size() > 0)
		std::cout << irregular.size() << " irregular faces, ";
	std::cout << "took " << nse::util::timeString(timer.value()) << ")" << std::endl;
}

void osr::load_obj(const std::string &filename, MatrixXu &F, Matrix3Xf &V)
{
    /// Vertex indices used by the OBJ format
    struct obj_vertex {
        uint32_t p = (uint32_t) -1;
        uint32_t n = (uint32_t) -1;
        uint32_t uv = (uint32_t) -1;

        inline obj_vertex() { }

        inline obj_vertex(const std::string &string) {
            std::vector<std::string> tokens = str_tokenize(string, '/', true);

            if (tokens.size() < 1 || tokens.size() > 3)
                throw std::runtime_error("Invalid vertex data: \"" + string + "\"");

            p = str_to_uint32_t(tokens[0]);

            #if 0
                if (tokens.size() >= 2 && !tokens[1].empty())
                    uv = str_to_uint32_t(tokens[1]);

                if (tokens.size() >= 3 && !tokens[2].empty())
                    n = str_to_uint32_t(tokens[2]);
            #endif
        }

        inline bool operator==(const obj_vertex &v) const {
            return v.p == p && v.n == n && v.uv == uv;
        }
    };

    /// Hash function for obj_vertex
    struct obj_vertexHash : std::unary_function<obj_vertex, size_t> {
        std::size_t operator()(const obj_vertex &v) const {
            size_t hash = std::hash<uint32_t>()(v.p);
            hash = hash * 37 + std::hash<uint32_t>()(v.uv);
            hash = hash * 37 + std::hash<uint32_t>()(v.n);
            return hash;
        }
    };

    typedef std::unordered_map<obj_vertex, uint32_t, obj_vertexHash> VertexMap;

    std::ifstream is(filename);
    if (is.fail())
        throw std::runtime_error("Unable to open OBJ file \"" + filename + "\"!");
    std::cout << "Loading \"" << filename << "\" .. ";
	std::cout.flush();
    nse::util::Timer<> timer;

    std::vector<Vector3f>   positions;
    //std::vector<Vector2f>   texcoords;
    //std::vector<Vector3f>   normals;
    std::vector<uint32_t>   indices;
    std::vector<obj_vertex> vertices;
    VertexMap vertexMap;

    std::string line_str;
    while (std::getline(is, line_str)) {
        std::istringstream line(line_str);

        std::string prefix;
        line >> prefix;

        if (prefix == "v") {
            Vector3f p;
            line >> p.x() >> p.y() >> p.z();
            positions.push_back(p);
        } else if (prefix == "vt") {
            /*
            Vector2f tc;
            line >> tc.x() >> tc.y();
            texcoords.push_back(tc);
            */
        } else if (prefix == "vn") {
            /*
            Vector3f n;
            line >> n.x() >> n.y() >> n.z();
            normals.push_back(n);
            */
        } else if (prefix == "f") {
            std::string v1, v2, v3, v4;
            line >> v1 >> v2 >> v3 >> v4;
            obj_vertex tri[6];
            int nVertices = 3;

            tri[0] = obj_vertex(v1);
            tri[1] = obj_vertex(v2);
            tri[2] = obj_vertex(v3);

            if (!v4.empty()) {
                /* This is a quad, split into two triangles */
                tri[3] = obj_vertex(v4);
                tri[4] = tri[0];
                tri[5] = tri[2];
                nVertices = 6;
            }
            /* Convert to an indexed vertex list */
            for (int i=0; i<nVertices; ++i) {
                const obj_vertex &v = tri[i];
                VertexMap::const_iterator it = vertexMap.find(v);
                if (it == vertexMap.end()) {
                    vertexMap[v] = (uint32_t) vertices.size();
                    indices.push_back((uint32_t) vertices.size());
                    vertices.push_back(v);
                } else {
                    indices.push_back(it->second);
                }
            }
        }
    }

    F.resize(3, indices.size()/3);
    memcpy(F.data(), indices.data(), sizeof(uint32_t)*indices.size());

    V.resize(3, vertices.size());
    for (uint32_t i=0; i<vertices.size(); ++i)
        V.col(i) = positions.at(vertices[i].p-1);

	std::cout << "done. (V=" << V.cols() << ", F=" << F.cols() << ", took "
         << nse::util::timeString(timer.value()) << ")" << std::endl;
}


struct XYZGatherActionPositions
{
	std::vector<Eigen::Vector3f>& positions;

	XYZGatherActionPositions(std::vector<Eigen::Vector3f>& positions)
		: positions(positions)
	{}

	void operator()(const boost::fusion::vector<float, float, float>& v) const
	{
		positions.emplace_back(boost::fusion::at_c<0>(v), boost::fusion::at_c<1>(v), boost::fusion::at_c<2>(v));
	}
};

struct XYZGatherActionPositionsAndNormals
{
	std::vector<Eigen::Vector3f>& positions;
	std::vector<Eigen::Vector3f>& normals;

	XYZGatherActionPositionsAndNormals(std::vector<Eigen::Vector3f>& positions, std::vector<Eigen::Vector3f>& normals)
		: positions(positions), normals(normals)
	{}

	void operator()(const boost::fusion::vector<float, float, float, float, float, float>& v) const
	{
		positions.emplace_back(boost::fusion::at_c<0>(v), boost::fusion::at_c<1>(v), boost::fusion::at_c<2>(v));
		normals.emplace_back(boost::fusion::at_c<3>(v), boost::fusion::at_c<4>(v), boost::fusion::at_c<5>(v));
	}
};

void osr::load_xyz(const std::string &filename, Matrix3Xf &V, Matrix3Xf &N)
{
	const char* data_begin, *data_end;
#ifdef OSR_USE_MEMORY_MAPPED_FILE
	boost::iostreams::mapped_file mmap(filename, boost::iostreams::mapped_file::readonly);
	data_begin = mmap.const_data();
	data_end = data_begin + mmap.size();
#else
	std::ifstream file(filename, std::ios::binary | std::ios::ate);
	size_t size = file.tellg();
	file.seekg(0, std::ios::beg);
	std::vector<char> buffer(size);
	file.read(buffer.data(), size);
	data_begin = buffer.data();
	data_end = buffer.data() + size;
#endif

	namespace qi = boost::spirit::qi;	

	const char* f = data_begin;
	const char* l = data_end;

	bool hasNormals = qi::phrase_parse(f, l, qi::float_ >> qi::float_ >> qi::float_ >> qi::float_ >> qi::float_ >> qi::float_, qi::blank);
	f = data_begin; //reset pointer to beginning

	if (hasNormals)
	{
		std::vector<Eigen::Vector3f> positions, normals;
		XYZGatherActionPositionsAndNormals gatherAction(positions, normals);
		bool success = qi::phrase_parse(f, l, ((qi::float_ >> qi::float_ >> qi::float_ >> qi::float_ >> qi::float_ >> qi::float_)[gatherAction] >> *(qi::char_ - qi::eol)) % qi::eol, qi::blank);

		Eigen::Map<Eigen::Matrix3Xf> mapPositions(reinterpret_cast<float*>(positions.data()), 3, positions.size());
		V = mapPositions;
		Eigen::Map<Eigen::Matrix3Xf> mapNormals(reinterpret_cast<float*>(normals.data()), 3, normals.size());
		N = mapNormals;
	}
	else
	{
		std::vector<Eigen::Vector3f> positions;
		XYZGatherActionPositions gatherAction(positions);
		bool success = qi::phrase_parse(f, l, ((qi::float_ >> qi::float_ >> qi::float_)[gatherAction] >> *(qi::char_ - qi::eol)) % qi::eol, qi::blank);

		Eigen::Map<Eigen::Matrix3Xf> map(reinterpret_cast<float*>(positions.data()),3, positions.size());
		V = map;
	}		

}

void osr::write_obj(const std::string &filename, const MatrixXu &F,
                const Matrix3Xf &V, const Matrix3Xf &N, const Matrix3Xf &Nf,
                const Matrix3Xf &UV, const Matrix3Xf &C)
{
    nse::util::Timer<> timer;
	std::cout << "Writing \"" << filename << "\" (V=" << V.cols()
         << ", F=" << F.cols() << ") .. ";
	std::cout.flush();
    std::ofstream os(filename);
    if (os.fail())
        throw std::runtime_error("Unable to open OBJ file \"" + filename + "\"!");
    if (N.size() > 0 && Nf.size() > 0)
        throw std::runtime_error("Please specify either face or vertex normals but not both!");

    for (uint32_t i=0; i<V.cols(); ++i)
        os << "v " << V(0, i) << " " << V(1, i) << " " << V(2, i) << std::endl;

    for (uint32_t i=0; i<N.cols(); ++i)
        os << "vn " << N(0, i) << " " << N(1, i) << " " << N(2, i) << std::endl;

    for (uint32_t i=0; i<Nf.cols(); ++i)
        os << "vn " << Nf(0, i) << " " << Nf(1, i) << " " << Nf(2, i) << std::endl;

    for (uint32_t i=0; i<UV.cols(); ++i)
        os << "vt " << UV(0, i) << " " << UV(1, i) << std::endl;

    /* Check for irregular faces */
    std::map<uint32_t, std::pair<uint32_t, std::map<uint32_t, uint32_t>>> irregular;
    size_t nIrregular = 0;

    for (uint32_t f=0; f<F.cols(); ++f) {
        if (F.rows() == 4) {
            if (F(2, f) == F(3, f)) {
                nIrregular++;
                auto &value = irregular[F(2, f)];
                value.first = f;
                value.second[F(0, f)] = F(1, f);
                continue;
            }
        }
        os << "f ";
        for (uint32_t j=0; j<F.rows(); ++j) {
            uint32_t idx = F(j, f);
            idx += 1;
            os << idx;
            if (Nf.size() > 0)
                idx = f + 1;
            os << "//" << idx << " ";
        }
        os << std::endl;
    }

    for (auto item : irregular) {
        auto face = item.second;
        uint32_t v = face.second.begin()->first, first = v;
        os << "f ";
        while (true) {
            uint32_t idx = v + 1;
            os << idx;
            if (Nf.size() > 0)
                idx = face.first + 1;
            os << "//" << idx << " ";

            v = face.second[v];
            if (v == first)
                break;
        }
        os << std::endl;
    }

	std::cout << "done. (";
    if (irregular.size() > 0)
		std::cout << irregular.size() << " irregular faces, ";
	std::cout << "took " << nse::util::timeString(timer.value()) << ")" << std::endl;
}
