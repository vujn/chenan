#pragma once
#include "stdafx.h"
#include <string>
#include <Standard_ErrorHandler.hxx> 
#include "Geom2d_Curve.hxx"
#include "Geom2dAPI_InterCurveCurve.hxx"
#include "GeomAPI_IntCS.hxx"
#include "GeomAPI_IntSS.hxx"
#include "Geom2d_Line.hxx"
#include "Geom2d_Circle.hxx"
#include "Geom_Surface.hxx"
#include "Structure.h"
#include "Geom_SphericalSurface.hxx"
#include "Geom_ConicalSurface.hxx"
#include "Geom2d_Ellipse.hxx"
#include "Geom_BSplineCurve.hxx"
#include "Geom2d_BoundedCurve.hxx"
#include "Geom2d_BSplineCurve.hxx"
#include "TColgp_Array1OfPnt2d.hxx"
#include "TColStd_Array1OfReal.hxx"
#include "TColStd_Array1OfInteger.hxx"
#include "SFace.h"
#include "Curve.h"


using namespace std;
class StepEntity;


class InterProcess
{
public:
	InterProcess(void);
	~InterProcess(void);
	int SetInterPointList(SFace* F1, SFace* F2, SFace* F3);//三维空间求交，生成交点，写进交点列表
	int SetInterPointList(Curve* C1,SFace* F1);//三维空间求交，生成交点，写进交点列表
	int SetInterPointList(Curve* C1, Curve* C2);//二维空间求交，生成交点，写进交点列表
public:
	vector<CPoint3D> interPointList_;  //交点链表
	Standard_Real tolerance_;
};
