#pragma once

#include  "stdafx.h"
#include "SFace.h"


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

	
	void IsPartitionFace();
	vector<SFace*> OcctSplit();		//occt�и���
	bool JudgeIntersection(SFace* Fa, SFace* Fb, char* curveName, orientationFaceA oriA,
		EdgeCurveVertex curveA, EdgeCurveVertex curveB, CPoint3D pointA);// �������ཻ���߶�����ཻ
	void FindPartitionFace(SFace* Fa, SFace* Fb);

public:
	vector<SFace*> NatlHalfSpaceList_;			//��Ȼ��ռ����

private:
	multimap<size_t, SFace*> partitionFaceList_;
	vector<SFace*> intersectionFaceList_;
	vector<SFace*> faceInfors_;
	RoseDesign* design_;
};

