# Online Surface Reconstruction
[![Build Status](https://travis-ci.org/NSchertler/OnlineSurfaceReconstruction.svg?branch=master)](https://travis-ci.org/NSchertler/OnlineSurfaceReconstruction)
[![Build status](https://ci.appveyor.com/api/projects/status/qwh18dalevme8nf8?svg=true)](https://ci.appveyor.com/project/NSchertler/onlinesurfacereconstruction)

This repository contains the source code for the research paper:
> **Field-Aligned Online Surface Reconstruction** <br/>
> Nico Schertler, Marco Tarini, Wenzel Jakob, Misha Kazhdan, Stefan Gumhold, Daniele Panozzo <br/>
> ACM TOG 36, 4, July 2017 <br/>
> DOI: http://dx.doi.org/10.1145/3072959.3073635

## Pre-requisites:
* Boost headers (no compiled components are necessary)

On Linux, the following packages are necessary (available via `apt-get`):
* xorg-dev
* libglu1-mesa-dev
* libboost-dev (only the header files are needed)

## Compilation
To compile the project, simply check out the git repository, initialize all submodules, use CMake to generate project files for your favorite build environment, and run the build (use a 64 bit generator).
On Unix-based systems, this may look as follows:

    git clone https://github.com/NSchertler/OnlineSurfaceReconstruction.git
    cd OnlineSurfaceReconstruction
    git submodule update --init --recursive
    mkdir build
    cd build
    cmake .. -DNANOGUI_USE_GLAD=ON
    make
	
There are a couple of CMake options (prefix `OSR_`) that let you customize the build and add more features.

Some parts of the code make heavy use of OpenMP.
If your build environment does not support OpenMP (e.g. Clang on MacOS), the program will still compile but performance may be inferior due to missing parallelization.

## Binaries
The following binaries are compiled from the latest commit with default options:
* [Windows Binaries](https://github.com/NSchertler/OnlineSurfaceReconstruction/raw/deploy-windows/osr-windows.zip) (Visual Studio 2017)
* [Linux Binaries](https://github.com/NSchertler/OnlineSurfaceReconstruction/raw/deploy-linux/osr-linux.zip) (GCC 6)
* [macOS Binaries](https://github.com/NSchertler/OnlineSurfaceReconstruction/raw/deploy-osx/osr-macos.zip) (Clang, without OpenMP support)

## Data sets
A selection of data sets that we presented in our paper can be found here: https://wwwpub.zih.tu-dresden.de/~s0244354/OSRData/. This repository contains the original scan data, a project file for the OSR application, and the final reconstruction.

## Documentation
Source code documentation can be found in the [wiki](https://github.com/NSchertler/OnlineSurfaceReconstruction/wiki)