#pragma once

#include "stdafx.h"
#include "Curve.h"


class FaceBounds
{
public:
	FaceBounds();

	virtual ~FaceBounds();


public:

	/*  */
	BOOLEAN orientation_; 

	/*  */
	vector<Curve*> edgeLoop_;
	//edgeLoop vector
};

