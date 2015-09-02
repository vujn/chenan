#pragma once

#include  "stdafx.h"
#include "SFace.h"
#include "BRepAlgoAPI_Section.hxx"

class Partition
{
public:
	Partition();
	~Partition();

	void PartitionFace();

public:

	bool IsPartitionFace();
	vector<SFace*> OcctSplit();		//occt�и���
	void JudgeCommon(SFace*  Fa, SFace* Fb);// �������ཻ���߶�����ཻ
	void TraverseEdgeCurve(vector<stp_edge_curve*>& edgeList);
	vector<SFace*> intersectionFaceList_;
};

