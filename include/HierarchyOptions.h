#pragma once

//Specify the hierarchy to use. Only HMortonMultiPass is kept updated. The other 
//hierarchies are former attempts and do not conform to the required interface anymore.

#define HUnstructured 0
#define HRegularSubdiv 1
#define HOctree 2
#define HMortonMultiPass 3

//#define UsedHierarchy HUnstructured
//#define UsedHierarchy HRegularSubdiv
//#define UsedHierarchy HOctree
#define UsedHierarchy HMortonMultiPass