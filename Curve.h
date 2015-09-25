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
	Standard_CString curveName_;
	Standard_Boolean edgeCurveId_;
	Standard_Byte orientedEdgeOri_;		//oriented_edge
	Standard_Byte edgeCurvesameSense_;	//edge_curve
	CPoint3D edgeStart_;
	CPoint3D edgeEnd_;
	Standard_Real coefficient_[6];//二次曲线的一般方程为Ax^2+2Bxy+2Cx+Dy^2+2Ey+F = 0，共6个系数
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
	CPoint3D pnt_; //表示其定位点
	CVector3D dir_;//表示其方向
	Standard_Real magnitude_;//表示其大小
};


////////////////////////////////////////////////////////////////////////圆
class CIRCLE :public Curve
{
public:
	CIRCLE();
	virtual ~CIRCLE();
	Curve * GenerateCoefficient(CMatrix3D RTMatrix);
	Geom2d_Curve * ToOCCT();

public:
	GeometryData position_;
	Standard_Real radius_;
};


////////////////////////////////////////////////////////////////////////椭圆
class ELLIPSE :public Curve
{
public:
	ELLIPSE();
	virtual ~ELLIPSE();
	Curve * GenerateCoefficient(CMatrix3D RTMatrix);
	Geom2d_Curve * ToOCCT();
public:
	GeometryData position_;
	Standard_Real semi_axis_1_;
	Standard_Real semi_axis_2_;
};


////////////////////////////////////////////////////////////////////////双曲线
class HYPERBOLA : Curve
{
public:
	HYPERBOLA();
	virtual ~HYPERBOLA();
	Curve * GenerateCoefficient(CMatrix3D RTMatrix);
	Geom2d_Curve * ToOCCT();

public:
	GeometryData position_;
	Standard_Real semi_axis_;
	Standard_Real semi_imag_axis_;

};


////////////////////////////////////////////////////////////////////////抛物线
class PARABOLA : Curve
{
public:
	PARABOLA();
	virtual ~PARABOLA();

	Curve * GenerateCoefficient(CMatrix3D RTMatrix);
	Geom2d_Curve * ToOCCT();

public:
	GeometryData position_;
	Standard_Real focal_dist_;

};


////////////////////////////////////////////////////////////////////////b样条曲线
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
