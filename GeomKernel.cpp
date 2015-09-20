#include "stdafx.h"
#include "GeomKernel.h"
#include "math.h"


	
CTriChip::CTriChip()
{
}

CTriChip::CTriChip(const CPoint3D& v0, const CPoint3D& v1, const CPoint3D& v2, const CVector3D& norm)
{
	vex_[0] = v0;
	vex_[1] = v1;
	vex_[2] = v2;
	normal_ = norm;
}

CTriChip::~CTriChip()
{

}

const CTriChip& CTriChip::operator =(const CTriChip& tri)
{
	normal_ = tri.normal_;
	for (int i=0; i<3; i++)
	{
		vex_[i] = tri.vex_[i];
	}
	return *this;
}



