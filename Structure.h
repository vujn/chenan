#pragma once

#include "stdafx.h"


enum HalfSpaceOrientation
{
	POSITIVE = 1,
	NEGATIVE = -1,
	INVALIDATE = -2
};

enum TermPosition
{
	InSolid = 1,
	OutSolid = -1,
	Undetermined = 0
};

enum PointPosition
{
	IN_FACE = 1,
	OUT_FACE = -1,
	ON_FACE = 0
};

enum ConvexConcave
{
	convex = 1,
	concave = -1,
	complex = 0
};

enum LOGICAL
{
	T = 1,
	F = 0,
	NOT_KNOWN = -1
};

typedef struct tagTermIndex
{
	TermPosition tPosition;
	int tListIndex;
	bool isNotNull;
} TermIndex;

struct ORIENTATION
{
	BOOLEAN orientedEdgeOri;
	BOOLEAN edgeCurveOri;
};

struct GeometryData 
{
	CPoint3D point;
	CPoint3D pointReserve;
	CVector3D verAxis;
	CVector3D verRefDirection;
};

struct EdgeCurveVertex
{
	CPoint3D cartesianStart;
	CPoint3D cartesianEnd;
	CVector3D axisDirection;
};


