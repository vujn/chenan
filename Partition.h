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
	~Partition();


public:

	void GetSFaceInfo(SetOfstp_face* stpFace);
	void StepConversionAndOutput(stp_representation_item* item);
	void NatlHalfVector(stp_advanced_face* adFace);
	void GetAxisData(stp_axis2_placement_3d* axis, GeometryData& data);
	stp_cartesian_point* EdgeCurveStartOrEnd(stp_vertex* ver);
	void replace_all(std::string & s, std::string const & t, std::string const & w);
	void SplitString(const char* str, const char* c, vector<string>& vecSplit);
public:

	
	bool IsPartitionFace(vector<SFace*> faceList);
	void OcctSplit(vector<SFace*> faceList, SFace* splitFace);		//occt�и���
	bool JudgeIntersection(SFace* Fa, SFace* Fb, Standard_CString curveName, orientationFaceA oriA,
		EdgeCurveVertex curveA, EdgeCurveVertex curveB, CPoint3D pointA, CPoint3D pointB);// �������ཻ���߶�����ཻ
	void FindPartitionFace(SFace* Fa, SFace* Fb);
	SFace* ChoosePartitionFace();
	void CurrentStructToOCCT( SFace* face, TopoDS_Shape& aShape );
	vector<SFace*> OcctToCurrentStruct(TopoDS_Shape aShape);
//	void AddNewSplit(TopoDS_Shape Stock, Handle(Geom_Surface)& plane1);

public:
	std::vector<std::string> vecOut_;//�������Ϣ�б�

public:
	vector<SFace*> NatlHalfSpaceList_;			//��Ȼ��ռ����

private:
	multimap<size_t, SFace*> partitionFaceList_;
	vector<vector<SFace*>> canSplitFaceList;
	vector<vector<SFace*>> intersectionFaceList_;
	vector<SFace*> faceInfors_;
	RoseDesign* design_;
	stp_representation_item* item_;
	bool isHasPartitionFace_;

};

