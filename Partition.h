#pragma once

#include  "stdafx.h"
#include "SFace.h"

class Partition
{
public:
	Partition();
	~Partition();

	void PartitionFace();

public:

	bool IsPartitionFace();
	vector<SFace*> OcctSplit();		//occt切割面
	void JudgeCommon(SFace*  Fa, SFace* Fb);// 两个面相交或者多个面相交
	void TraverseEdgeCurve(vector<stp_edge_curve*>& edgeList);
	vector<SFace*> intersectionFaceList_;
};

