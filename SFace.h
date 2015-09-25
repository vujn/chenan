#pragma once

#include "stdafx.h"
#include "Structure.h"
#include "FaceBounds.h"


///////////////////////////////////////////////////////////////////////////
class SFace
{
public:
	SFace();
	virtual ~SFace();
public:
	virtual void GenerateCoefficient();
	virtual Geom_Surface * ToOCCT();
public:
	PointPosition PointIsIn(CPoint3D TestPoint);

public:
	Standard_CString name_;
	Standard_Boolean entityID_;
	Standard_Byte adFaceSameSense_;//  advanced_face of same_sense    2015/09/08
	vector<FaceBounds*> faceBounds_;
	GeometryData* position_;
	Standard_Real coefficient_[10];//系数数组，二次曲面的一般方程为Ax^2+2Bxy+2Cxz+2Dx+Ey^2+2Fyz+2Gy+Hz^2+2Iz+J = 0，共10个系数
	EdgeCurveVertex* vertex_;
};

///////////////////////////////////////////////////////////////////////////
class SPlane : public SFace
{
public:
	SPlane();
	virtual ~SPlane();
public:
	void GenerateCoefficient();
	Geom_Surface * ToOCCT();

};

///////////////////////////////////////////////////////////////////////////
class SCylindrical : public SFace
{
public:
	SCylindrical();
	virtual ~SCylindrical();
public:
	void GenerateCoefficient();
	Geom_Surface * ToOCCT();

public:
	Standard_Real radius_;
};

///////////////////////////////////////////////////////////////////////////
class SSpherical : public SFace
{
public:
	SSpherical();
	virtual ~SSpherical();
public:
	void GenerateCoefficient();
	Geom_Surface * ToOCCT();

public:
	Standard_Real radius_;
};

///////////////////////////////////////////////////////////////////////////
class SConical : public SFace
{
public:
	SConical();
	virtual ~SConical();
public:
	void GenerateCoefficient();
	Geom_Surface * ToOCCT();

public:
	Standard_Real radius_;
	Standard_Real semi_angle_;
	Standard_Real vertex_[3];
};

///////////////////////////////////////////////////////////////////////////
class SToroidal : public SFace
{
public:
	SToroidal();
	virtual ~SToroidal();
public:
	void GenerateCoefficient();
	Geom_Surface * ToOCCT();

public:
	Standard_Real major_radius_;
	Standard_Real minor_radius_;
};
