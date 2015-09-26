
#pragma once
#include "stdafx.h"
#include "GeomCalc.h"


class  CTriChip 
{

public:
	//attributes
	CPoint3D vex_[3];
	CVector3D normal_;

public:
	CTriChip();
	CTriChip(const CPoint3D& v0, const CPoint3D& v1, const CPoint3D& v2, const CVector3D& norm);
	virtual ~CTriChip();


	//operator
	const CTriChip& operator=(const CTriChip&);
};
