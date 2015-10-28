#pragma once

#include  "stdafx.h"
#include "SFace.h"
#include "StepEntity.h"
#include "ShapeCutter.h"


#define ZOOMTIME 10.0


typedef	struct Orientation
{
	Standard_Byte advancedFaceOri;
	Standard_Byte boundsOri;
	Standard_Byte orientedEdgeOri;
	Standard_Byte edgeCurveOri;
}orientationFaceA, orientationFaceB;


class Partition
{
public:
	Partition(RoseDesign* design);
	Partition(TopoDS_Shape& shapes);
	~Partition();


public:

	void LoadStepFromOCCT();
	std::string Partition::DumpOrientation(const TopAbs_Orientation& orient);

	void GetSFaceInfo(SetOfstp_face* stpFace);
	void StepConversionAndOutput(stp_representation_item* item, string shapeName, int& m, int& n);
	void NatlHalfVector(stp_advanced_face* adFace);
	void GetAxisData(stp_axis2_placement_3d* axis, GeometryData& data);
	stp_cartesian_point* EdgeCurveStartOrEnd(stp_vertex* ver);
	
public:

	
	bool IsPartitionFace(vector<SFace*> faceList);
	void OcctSplit(vector<SFace*> faceList, SFace* splitFace);		//occt�и���
	bool JudgeIntersection(TopoDS_Face Fa, TopoDS_Face Fb);
	bool JudgeIntersection(SFace* Fa, SFace* Fb, Standard_CString curveName, orientationFaceA oriA, orientationFaceB oriB,
		EdgeCurveVertex curveA, EdgeCurveVertex curveB, CPoint3D pointA, CPoint3D pointB);// �������ཻ���߶�����ཻ
	void FindPartitionFace(SFace* Fa, SFace* Fb);
	SFace* ChoosePartitionFace();
	void CurrentStructToOCCT( SFace* face, TopoDS_Face& aFace );
	vector<SFace*> OcctToCurrentStruct(TopoDS_Shape aShape);
//	void AddNewSplit(TopoDS_Shape Stock, Handle(Geom_Surface)& plane1);
	SFace* CloneFace(SFace* face);
	GeometryData* CloneEntity(GeometryData* geo);
	void GetFaceList(vector<SFace*> faceList, SFace* splitFace);

	//��ȡ��ķ�����������
	gp_Vec GetFaceAxis(TopoDS_Face face);
public:
	std::vector<std::string> vecOut_;//�������Ϣ�б�
	int mp_;

public:
	vector<SFace*> NatlHalfSpaceList_;			//��Ȼ��ռ����

private:
	TopoDS_Shape shapes_;
	multimap<size_t, SFace*> partitionFaceList_;
	vector<vector<SFace*>> canSplitFaceList;
	vector<vector<SFace*>> intersectionFaceList_;
	vector<SFace*> test_;
	vector<SFace*> faceInfors_;
	RoseDesign* design_;
	stp_representation_item* item_;
	bool isHasPartitionFace_;
	
};

