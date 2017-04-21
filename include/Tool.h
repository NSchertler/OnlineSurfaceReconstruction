#pragma once

#include "GUIObject.h"

//Abstract class for a modification tool that alters point data.
class Tool : public GUIObject
{
public:
	//Activate the tool
	virtual void enterTool() { };

	//Deactivate the tool
	virtual void exitTool() { };	
};