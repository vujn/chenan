// URVE.h: interface for the CURVE class.
//
//////////////////////////////////////////////////////////////////////

#pragma once
#include "stdafx.h"
#include "GeomCalc.h"
#include <string>
#include "Geom2d_Curve.hxx"
#include "Geom2d_Line.hxx"
#include "Geom2d_Circle.hxx"
#include "gp_Ax2d.hxx"
#include "Structure.h"

////////////////////////////////////////////////////////////////////////
class Curve
{
public:
	Curve();
	virtual ~Curve();
	virtual Curve * GenerateCoefficient();
	virtual Geom2d_Curve * ToOCCT();

	BOOLEAN orientedEdgeOri_;		//oriented_edge
	BOOLEAN edgeCurvesameSense_;	//edge_curve

	EdgeCurveVertex* edgeStart_;
	EdgeCurveVertex* edgeEnd_;

	double coefficient_[6];//二次曲线的一般方程为Ax^2+2Bxy+2Cx+Dy^2+2Ey+F = 0，共6个系数
};


////////////////////////////////////////////////////////////////////////
class Circle :public Curve
{
public:
	Circle();
	virtual ~Circle();
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
class Line : public Curve
{
public:
	Line();
	virtual ~Line();
	Curve * GenerateCoefficient(CMatrix3D RTMatrix);
	Geom2d_Curve * ToOCCT();
public:
	CPoint3D pnt_;
	CVector3D dir_;
	double magnitude_;
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
