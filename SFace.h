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
	char* name_;
	size_t entityID_;

	BOOLEAN adFaceSameSense_;//  advanced_face of same_sense    2015/09/08
	vector<FaceBounds*> faceBounds_;

	GeometryData* position_;
	double coefficient_[10];//ϵ�����飬���������һ�㷽��ΪAx^2+2Bxy+2Cxz+2Dx+Ey^2+2Fyz+2Gy+Hz^2+2Iz+J = 0����10��ϵ��
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
	double radius_;
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
	double radius_;
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
	double radius_;
	double semi_angle_;
	double vertex_[3];
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
	double major_radius_;
	double minor_radius_;
};
