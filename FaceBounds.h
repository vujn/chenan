#pragma once

#include "stdafx.h"
#include "Curve.h"


class FaceBounds
{
public:
	FaceBounds();

	~FaceBounds();

public:

	/*! face_outer_bound orientation */
	Standard_Byte boundsOri_; 

	/*! save the edge_curve infors;*/
	vector<Curve*> edgeLoop_; 
};

