#pragma once

#include  "stdafx.h"
#include "SFace.h"
#include "BRepAlgoAPI_Section.hxx"


#define ZOOMTIME 10.0


typedef	struct Orientation
{
	BOOLEAN advancedFaceOri;
	BOOLEAN boundsOri;
	BOOLEAN orientedEdgeOri;
	BOOLEAN edgeCurveOri;
}orientationFaceA, orientationFaceB;


class Partition
{
public:
	Partition(RoseDesign* design);
	~Partition();

	void PartitionFace(stp_closed_shell* closeShell);

public:

	void StepConversionAndOutput();
	void NatlHalfVector(stp_advanced_face* adFace);
	void GetAxisData(stp_axis2_placement_3d* axis, GeometryData& data);
	stp_cartesian_point* EdgeCurveStartOrEnd(stp_vertex* ver);

public:

	
	bool IsPartitionFace();
	vector<SFace*> OcctSplit();		//occt切割面
	bool JudgeIntersection(SFace* Fa, SFace* Fb, orientationFaceA oriA, CPoint3D start, CPoint3D end);// 两个面相交或者多个面相交
	void FindPartitionFace(SFace* Fa, SFace* Fb);

public:
	vector<SFace*> NatlHalfSpaceList_;			//自然半空间队列

private:

	vector<SFace*> intersectionFaceList_;
	vector<SFace*> faceInfors_;
	RoseDesign* design_;
};

