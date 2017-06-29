/*
	This file is part of the implementation for the technical paper

		Field-Aligned Online Surface Reconstruction
		Nico Schertler, Marco Tarini, Wenzel Jakob, Misha Kazhdan, Stefan Gumhold, Daniele Panozzo
		ACM TOG 36, 4, July 2017 (Proceedings of SIGGRAPH 2017)

	Use of this source code is granted via a BSD-style license, which can be found
	in License.txt in the repository root.

	@author Nico Schertler
*/

#pragma once

#include <string>
#include <vector>
#include <algorithm>
#include <stdexcept>

#if _WIN32
#include <Windows.h>
#include <Shlwapi.h>
#pragma comment(lib, "Shlwapi.lib")
#else
#include <sys/stat.h>
#endif

namespace osr
{
	inline std::string str_tolower(std::string str)
	{
		std::transform(str.begin(), str.end(), str.begin(), ::tolower);
		return str;
	}

	inline bool file_exists(const std::string& path)
	{
		//implementation from https://stackoverflow.com/a/12774387/1210053
		if (FILE *file = fopen(path.c_str(), "r"))
		{
			fclose(file);
			return true;
		}
		else 
		{
			return false;
		}
	}


	//returns if a given path is a directory
	inline bool is_directory(const std::string& path)
	{
#if _WIN32
		return PathIsDirectory(path.c_str());
#else
		//code based on https://stackoverflow.com/a/146938/1210053
		struct stat s;
		if (stat(path.c_str(), &s) == 0)
		{
			if (s.st_mode & S_IFDIR)
				return true;
			else
				return false;
		}
		else
		{
			throw std::runtime_error("Cannot find properties of path \"" + path + "\".");
		}
#endif
	}

	inline size_t start_of_extension(const std::string& path)
	{
		for (size_t i = path.size() - 1; i > 0; --i)
		{
			bool isSeparator = path[i] == '\\' || path[i] == '/';
			if (isSeparator)
				return -1;
			bool isDot = path[i] == '.';
			if (isDot)
				return i;
		}
		return -1;
	}

	inline std::string extension(const std::string& path)
	{	
		auto soe = start_of_extension(path);
		if (soe == -1)
			return "";
		return str_tolower(path.substr(soe, path.size() - soe));
	}

	inline std::string replace_extension(const std::string& path, const std::string& new_extension)
	{
		auto soe = start_of_extension(path);
		bool dot_included = new_extension[0] == '.';
		if (soe == -1)
			return path;
		else
			return path.substr(0, soe + (dot_included ? 0 : 1)) + new_extension;
	}

	inline std::string filename_without_extension_and_directory(const std::string& path)
	{
		size_t filenameUpTo = path.size();
		for (size_t i = path.size() - 1; i > 0; --i)
		{
			bool isSeparator = path[i] == '\\' || path[i] == '/';
			if (isSeparator)
				return path.substr(i + 1, filenameUpTo - i - 1);
			bool isDot = path[i] == '.';
			if (isDot && filenameUpTo == path.size())
				filenameUpTo = i;
		}
		return path.substr(0, filenameUpTo);
	}

	//Returns the parent of a given path. The parent is found by cutting of everything
	//after the last directory separator
	inline std::string parent_path(const std::string& path)
	{
		bool observedPathName = false; //the path might end with path separators; this flag determines if we are already past them
		for (size_t i = path.size() - 1; i > 0; --i)
		{
			bool isSeparator = path[i] == '\\' || path[i] == '/';
			if (isSeparator && observedPathName)
				return path.substr(0, i);
			if (!isSeparator)
				observedPathName = true;
		}
		throw std::runtime_error("The path \"" + path + "\" has an invalid format.");
	}

	extern void files_in_dir(const std::string &path, std::vector<std::string>& result);
}