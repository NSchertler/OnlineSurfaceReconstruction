#include "osr/filehelper.h"

#if !_WIN32
#include <dirent.h>
#endif

void osr::files_in_dir(const std::string &path, std::vector<std::string>& result)
{
	//based on nanogui::loadImageDirectory
	result.clear();
#if !defined(_WIN32)
	DIR *dp = opendir(path.c_str());
	if (!dp)
		throw std::runtime_error("Could not open directory!");
	struct dirent *ep;
	while ((ep = readdir(dp)))
	{
		const char *fname = ep->d_name;
#else
	WIN32_FIND_DATA ffd;
	std::string searchPath = path + "/*.*";
	HANDLE handle = FindFirstFileA(searchPath.c_str(), &ffd);
	if (handle == INVALID_HANDLE_VALUE)
		throw std::runtime_error("Could not open image directory!");
	do
	{
		const char *fname = ffd.cFileName;
#endif
		result.push_back(path + "/" + std::string(fname));
#if !defined(_WIN32)
	}
	closedir(dp);
#else
	} while (FindNextFileA(handle, &ffd) != 0);
	FindClose(handle);
#endif
}