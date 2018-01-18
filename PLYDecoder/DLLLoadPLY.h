#pragma once
#define HAVE_NANOGUI 1
#define HAVE_EIGEN 1
#define _CRT_SECURE_NO_WARNINGS 1

#define DLL_LOAD_PLY __declspec(dllexport) 

extern "C" {
	DLL_LOAD_PLY void TestSort(int a[], int length);

	DLL_LOAD_PLY void LoadPLY(unsigned char * charF, unsigned char * charV, unsigned char * charN, int meshSize[]);
	DLL_LOAD_PLY void LoadPLY2(unsigned char * charF, unsigned char * charV, unsigned char * charN, int meshSize[]);
	DLL_LOAD_PLY float* LoadPLY3(const char* path, unsigned char ** pF, float ** pV, int meshSize[]);
	DLL_LOAD_PLY void LoadPLYDirect(int* charF, float* charV, float* charN, int meshSize[]);
}