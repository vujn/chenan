// URVE.h: interface for the CURVE class.
//
//////////////////////////////////////////////////////////////////////

#pragma once
#include "stdafx.h"
#include <string>
#include "Structure.h"

////////////////////////////////////////////////////////////////////////
class Curve
{
public:
	Curve();
	virtual ~Curve();
	virtual Curve * GenerateCoefficient();
	virtual Geom2d_Curve * ToOCCT();

public:
	char* curveName_;
	size_t edgeCurveId_;
	BOOLEAN orientedEdgeOri_;		//oriented_edge
	BOOLEAN edgeCurvesameSense_;	//edge_curve
	CPoint3D edgeStart_;
	CPoint3D edgeEnd_;
	double coefficient_[6];//�������ߵ�һ�㷽��ΪAx^2+2Bxy+2Cx+Dy^2+2Ey+F = 0����6��ϵ��
};

////////////////////////////////////////////////////////////////////////
class CIRCLE :public Curve
{
public:
	CIRCLE();
	virtual ~CIRCLE();
	Curve * GenerateCoefficient(CMatrix3D RTMatrix);
	Geom2d_Curve * ToOCCT();

public:
	GeometryData position_;
	double radius_;
};


////////////////////////////////////////////////////////////////////////
class ELLIPSE :public Curve
{
public:
	ELLIPSE();
	virtual ~ELLIPSE();
	Curve * GenerateCoefficient(CMatrix3D RTMatrix);
	Geom2d_Curve * ToOCCT();
public:
	GeometryData position_;
	double semi_axis_1_;
	double semi_axis_2_;
};


////////////////////////////////////////////////////////////////////////
class LINE : public Curve
{
public:
	LINE();
	virtual ~LINE();
	Curve * GenerateCoefficient(CMatrix3D RTMatrix);
	Geom2d_Curve * ToOCCT();
public:
	CPoint3D pnt_; //��ʾ�䶨λ��
	CVector3D dir_;//��ʾ�䷽��
	double magnitude_;//��ʾ���С
};

////////////////////////////////////////////////////////////////////////
class B_Spline_Curve_With_Knots : public Curve
{
public:
	B_Spline_Curve_With_Knots();
	virtual ~B_Spline_Curve_With_Knots();
	Curve * GenerateCoefficient();
	Geom2d_Curve * ToOCCT();
public:
	int degree_;
	ListOfstp_cartesian_point* controlPointsList_;
	stp_b_spline_curve_form curveForm_;
	RoseLogical closedCurve_;
	RoseLogical selfIntersect_;
	ListOfInteger* knotMultiplicities_;
	ListOfDouble* knots_;
	stp_knot_type kontSpec_;
};
