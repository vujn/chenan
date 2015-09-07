#pragma once

#include  "stdafx.h"
#include "SFace.h"
#include "BRepAlgoAPI_Section.hxx"

class Partition
{
public:
	Partition();
	~Partition();

	void PartitionFace(stp_closed_shell* closeShell);

public:

	bool IsPartitionFace();
	vector<SFace*> OcctSplit();		//occt�и���
	void JudgeIntersection(SFace*  Fa, SFace* Fb);// �������ཻ���߶�����ཻ
	void TraverseEdgeCurve(vector<stp_edge_curve*>& edgeList);
	vector<SFace*> intersectionFaceList_;
};

