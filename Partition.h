#pragma once

#include  "stdafx.h"
#include "SFace.h"
#include "StepEntity.h"


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


public:

	void GetSFaceInfo(SetOfstp_face* stpFace);
	void StepConversionAndOutput();
	void NatlHalfVector(stp_advanced_face* adFace);
	void GetAxisData(stp_axis2_placement_3d* axis, GeometryData& data);
	stp_cartesian_point* EdgeCurveStartOrEnd(stp_vertex* ver);

public:

	
	void PartitionFace();
	vector<SFace*> OcctSplit();		//occt�и���
	bool JudgeIntersection(SFace* Fa, SFace* Fb, char* curveName, orientationFaceA oriA,
		EdgeCurveVertex curveA, EdgeCurveVertex curveB, CPoint3D pointA);// �������ཻ���߶�����ཻ
	void FindPartitionFace(SFace* Fa, SFace* Fb);
	SFace* ChoosePartitionFace();
	void CurrentStructToOCCT(TopoDS_Shape& aShape, SFace* face);
	void OcctToCurrentStruct(TopoDS_Shape& aShape);
	TopoDS_Shape AddNewSplit(TopoDS_Shape Stock, Handle(Geom_Surface)& plane1);

public:
	std::vector<std::string> vecOut_;//�������Ϣ�б�

public:
	vector<SFace*> NatlHalfSpaceList_;			//��Ȼ��ռ����

private:
	multimap<size_t, SFace*> partitionFaceList_;
	vector<SFace*> intersectionFaceList_;
	vector<SFace*> faceInfors_;
	RoseDesign* design_;
};

