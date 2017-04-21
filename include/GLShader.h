#pragma once
//Adapted nanogui shader

#include <nanogui/glutil.h>

//Represents an OpenGL shader program
class GLShader 
{	
public:
	/// Create an unitialized OpenGL shader program
	GLShader()
		: mVertexShader(0), mFragmentShader(0), mGeometryShader(0),
		mProgramShader(0) { }

	~GLShader();

	/// Initialize the shader using the specified source strings
	bool init(const std::string &name, const std::string &vertex_str,
		const std::string &fragment_str,
		const std::string &geometry_str = "");

	bool initWithTessellation(const std::string &name, const std::string &vertex_str,
		const std::string &tessellation_control_str,
		const std::string &tessellation_eval_str,
		const std::string &fragment_str,
		const std::string &geometry_str = "");	

	/// Initialize the shader using the specified files on disk
	bool initFromFiles(const std::string &name,
		const std::string &vertex_fname,
		const std::string &fragment_fname,
		const std::string &geometry_fname = "");

	/// Return the name of the shader
	const std::string &name() const { return mName; }

	/// Set a preprocessor definition
	void define(const std::string &key, const std::string &value) { mDefinitions[key] = value; }

	/// Select this shader for subsequent draw calls
	void bind();

	/// Return the handle of a named shader attribute (-1 if it does not exist)
	GLint attrib(const std::string &name, bool warn = true) const;

	/// Return the handle of a uniform attribute (-1 if it does not exist)
	GLint uniform(const std::string &name, bool warn = true) const;	

	/// Initialize a uniform parameter with a 4x4 matrix (float)
	template <typename T>
	void setUniform(const std::string &name, const Eigen::Matrix<T, 4, 4> &mat, bool warn = true) {
		glUniformMatrix4fv(uniform(name, warn), 1, GL_FALSE, mat.template cast<float>().data());
	}

	/// Initialize a uniform parameter with an integer value
	template <typename T, typename std::enable_if<nanogui::detail::type_traits<T>::integral == 1, int>::type = 0>
	void setUniform(const std::string &name, T value, bool warn = true) {
		glUniform1i(uniform(name, warn), (int)value);
	}

	/// Initialize a uniform parameter with a floating point value
	template <typename T, typename std::enable_if<nanogui::detail::type_traits<T>::integral == 0, int>::type = 0>
	void setUniform(const std::string &name, T value, bool warn = true) {
		glUniform1f(uniform(name, warn), (float)value);
	}

	/// Initialize a uniform parameter with a 2D vector (int)
	template <typename T, typename std::enable_if<nanogui::detail::type_traits<T>::integral == 1, int>::type = 0>
	void setUniform(const std::string &name, const Eigen::Matrix<T, 2, 1>  &v, bool warn = true) {
		glUniform2i(uniform(name, warn), (int)v.x(), (int)v.y());
	}

	/// Initialize a uniform parameter with a 2D vector (float)
	template <typename T, typename std::enable_if<nanogui::detail::type_traits<T>::integral == 0, int>::type = 0>
	void setUniform(const std::string &name, const Eigen::Matrix<T, 2, 1>  &v, bool warn = true) {
		glUniform2f(uniform(name, warn), (float)v.x(), (float)v.y());
	}

	/// Initialize a uniform parameter with a 3D vector (int)
	template <typename T, typename std::enable_if<nanogui::detail::type_traits<T>::integral == 1, int>::type = 0>
	void setUniform(const std::string &name, const Eigen::Matrix<T, 3, 1>  &v, bool warn = true) {
		glUniform3i(uniform(name, warn), (int)v.x(), (int)v.y(), (int)v.z());
	}

	/// Initialize a uniform parameter with a 3D vector (float)
	template <typename T, typename std::enable_if<nanogui::detail::type_traits<T>::integral == 0, int>::type = 0>
	void setUniform(const std::string &name, const Eigen::Matrix<T, 3, 1>  &v, bool warn = true) {
		glUniform3f(uniform(name, warn), (float)v.x(), (float)v.y(), (float)v.z());
	}

	/// Initialize a uniform parameter with a 4D vector (int)
	template <typename T, typename std::enable_if<nanogui::detail::type_traits<T>::integral == 1, int>::type = 0>
	void setUniform(const std::string &name, const Eigen::Matrix<T, 4, 1>  &v, bool warn = true) {
		glUniform4i(uniform(name, warn), (int)v.x(), (int)v.y(), (int)v.z(), (int)v.w());
	}

	/// Initialize a uniform parameter with a 4D vector (float)
	template <typename T, typename std::enable_if<nanogui::detail::type_traits<T>::integral == 0, int>::type = 0>
	void setUniform(const std::string &name, const Eigen::Matrix<T, 4, 1>  &v, bool warn = true) {
		glUniform4f(uniform(name, warn), (float)v.x(), (float)v.y(), (float)v.z(), (float)v.w());
	}

	//Returns the shader program that has been bound last.
	static GLShader* currentProgram();	

protected:
	static GLShader* _currentProgram;

	std::string mName;
	GLuint mVertexShader;
	GLuint mTessellationControlShader;
	GLuint mTessellationEvalShader;
	GLuint mFragmentShader;
	GLuint mGeometryShader;
	GLuint mProgramShader;
	std::map<std::string, std::string> mDefinitions;	
};