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
	else if (strcmp(Fa->name_, "plane") && strcmp(Fb->name_, ""))//FaΪƽ��,FbΪ���� 
	{

	}
	else if (strcmp(Fa->name_, "") && strcmp(Fb->name_, "plane"))//FaΪ����,FbΪƽ��
	{

	}
	else if (strcmp(Fa->name_, "") && strcmp(Fb->name_, ""))//FaΪ����,FbΪ����,�ཻΪԲ׶���ߵ����
	{

	}
}

void Partition::TraverseEdgeCurve(vector<stp_edge_curve*>& edgeList)
{

}
