#include "stdafx.h"
#include "Curve.h"
#include <Geom2d_Ellipse.hxx>
#include <TColgp_Array1OfPnt2d.hxx>
#include <TColStd_Array1OfInteger.hxx>
#include <Geom2d_BSplineCurve.hxx>
#include <TColStd_Array1OfReal.hxx>
#include <TColStd_Array1OfInteger.hxx>

//////////////////////////////////////////////////////////////////////
// CURVE
//////////////////////////////////////////////////////////////////////

Curve::Curve()
{

}

Curve::~Curve()
{

}

Curve * Curve::GenerateCoefficient()
{
	return NULL;
}

Geom2d_Curve * Curve::ToOCCT()
{
	return NULL;
}


//////////////////////////////////////////////////////////////////////
// CIRCLE
//////////////////////////////////////////////////////////////////////
	
Circle::Circle()
{
}

Circle::~Circle()
{

}

Curve * Circle::GenerateCoefficient(CMatrix3D RTMatrix)
{
	//二次曲线的一般方程为Ax^2+2Bxy+2Cx+Dy^2+2Ey+F = 0，共6个系数

	CPoint3D center = position_.point * RTMatrix;
	Circle * pCircle =  new Circle;
	pCircle->position_.point = center;
	pCircle->radius_ = radius_;
	return pCircle;
}

Geom2d_Curve * Circle::ToOCCT()
{
	gp_Pnt2d Pnt1(position_.point.x,position_.point.y);
	gp_Vec2d Vec1(1,0);
	gp_Dir2d Dir1(Vec1);
	gp_Ax2d Ax2d1(Pnt1,Dir1);
	Geom2d_Circle * pcircle = new Geom2d_Circle(Ax2d1,radius_,Standard_True);
	return pcircle;
}


//////////////////////////////////////////////////////////////////////
// ELLIPSE
//////////////////////////////////////////////////////////////////////
ELLIPSE::ELLIPSE()
{

}

ELLIPSE::~ELLIPSE()
{

}

Curve * ELLIPSE::GenerateCoefficient(CMatrix3D RTMatrix)
{
//二次曲线的一般方程为Ax^2+2Bxy+2Cx+Dy^2+2Ey+F = 0，共6个系数
	CPoint3D center = position_.point * RTMatrix;
	coefficient_[0] = 1.0/(semi_axis_1_*semi_axis_1_);  //A
	coefficient_[1]=0.0;  //B
	coefficient_[2] = -center.x / (semi_axis_1_*semi_axis_1_);  //C
	coefficient_[3] = 1.0 / (semi_axis_2_*semi_axis_2_);  //D
	coefficient_[4] = -center.y / (semi_axis_2_*semi_axis_2_);  //E
	coefficient_[5] = center.x*center.x / (semi_axis_1_*semi_axis_1_) 
		+ center.y*center.y / (semi_axis_2_*semi_axis_2_) - 1;  //F
	return NULL;
}

Geom2d_Curve * ELLIPSE::ToOCCT()
{
	gp_Pnt2d Pnt1(position_.point.x, position_.point.y);
	gp_Vec2d Vec1(1, 0);
	gp_Dir2d Dir1(Vec1);
	gp_Ax2d Ax2d1(Pnt1, Dir1);
	Geom2d_Ellipse* ell = new Geom2d_Ellipse(Ax2d1, semi_axis_1_, semi_axis_2_, Standard_True);
	return ell;
	return NULL;
}


//////////////////////////////////////////////////////////////////////
// LINE
//////////////////////////////////////////////////////////////////////
Line::Line()
{
}

Line::~Line()
{

}

Curve * Line::GenerateCoefficient(CMatrix3D RTMatrix)
{
	//二次曲线的一般方程为Ax^2+2Bxy+2Cx+Dy^2+2Ey+F = 0，共6个系数
	CPoint3D direction = CPoint3D(dir_.dx,dir_.dy,dir_.dz)*RTMatrix;
	CPoint3D point = pnt_*RTMatrix;
	Line * pLine = new Line;
	pLine->pnt_ = point;
	pLine->dir_.dx = direction.x;
	pLine->dir_.dy = direction.y;
	pLine->dir_.dz = direction.z;
	return pLine;
}

Geom2d_Curve* Line::ToOCCT()
{
	gp_Vec2d Vec1(dir_.dx,dir_.dy);
	gp_Dir2d Dir1(Vec1);
	gp_Pnt2d Pnt1(pnt_.x,pnt_.y);
	Geom2d_Curve * pLine = new Geom2d_Line(Pnt1,Dir1);
	return pLine;
}

//////////////////////////////////////////////////////////////////////
//B_SPLINE_CURVE_WITH_KNOTS
//////////////////////////////////////////////////////////////////////

B_Spline_Curve_With_Knots::B_Spline_Curve_With_Knots()
{
}

B_Spline_Curve_With_Knots::~B_Spline_Curve_With_Knots()
{

}

Curve * B_Spline_Curve_With_Knots::GenerateCoefficient()
{

	return NULL;
}
Geom2d_Curve * B_Spline_Curve_With_Knots::ToOCCT()
{
	stp_cartesian_point* point = controlPointsList_->get(0);
	gp_Pnt2d pnt1(point->coordinates()->get(0), point->coordinates()->get(1));
	TColgp_Array1OfPnt2d poles(pnt1, 1, 1);/////????????????
	TColStd_Array1OfReal knots(knots_->get(0), knots_->get(1), knots_->get(2));
	TColStd_Array1OfInteger multiplicities(knotMultiplicities_->get(0), knotMultiplicities_->get(1), knotMultiplicities_->get(2));
	Standard_Integer degree = degree_;
	Standard_Boolean periodic = Standard_False;
	Geom2d_BSplineCurve * bSpline = new Geom2d_BSplineCurve(poles,
		knots,
		multiplicities,
		degree,
		periodic);
	return bSpline;
}

















