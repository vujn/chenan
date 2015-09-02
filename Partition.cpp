#include "stdafx.h"
#include "Partition.h"


Partition::Partition()
{

}


Partition::~Partition()
{
}

void Partition::PartitionFace()
{
	if (IsPartitionFace)
	{
		OcctSplit();
	}
	else
	{

	}
}

bool Partition::IsPartitionFace()
{
	vector<stp_edge_curve*> edgeCurveList;
	TraverseEdgeCurve(edgeCurveList);
	
	JudgeCommon();
}

vector<SFace*> Partition::OcctSplit()
{
//	BRepAlgoAPI_Section

}

void Partition::JudgeCommon(SFace* Fa, SFace* Fb)
{
	if (strcmp(Fa->name_, "plane") && strcmp(Fb->name_, "plane"))
	{

	}
	else if (strcmp(Fa->name_, "plane") && strcmp(Fb->name_, ""))//Fa为平面,Fb为曲面 
	{

	}
	else if (strcmp(Fa->name_, "") && strcmp(Fb->name_, "plane"))//Fa为曲面,Fb为平面
	{

	}
	else if (strcmp(Fa->name_, "") && strcmp(Fb->name_, ""))//Fa为曲面,Fb为曲面,相交为圆锥曲线的情况
	{

	}
}

void Partition::TraverseEdgeCurve(vector<stp_edge_curve*>& edgeList)
{

}
