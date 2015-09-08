#include "stdafx.h"
#include "Partition.h"


Partition::Partition()
{

}


Partition::~Partition()
{
}

void Partition::PartitionFace(stp_closed_shell* closeShell)
{
	SetOfstp_face* stpFace = closeShell->cfs_faces();
	for(size_t i = 0; i < stpFace->size(); i++)
	{
		stp_face* face = stpFace->get(i);
		stp_advanced_face* adFace = ROSE_CAST(stp_advanced_face, face);
		
		//NatlHalfVector(adFace);// 保存每个面结构的信息 faceInfors_;
	}
	//比较两个面是否有相交的边 如果有->判断是否是分割面; 否则继续下两个面的判断
	SFace* Fa = new SFace;
	SFace* Fb = new SFace;
	JudgeIntersection(Fa, Fb);
	
	if (IsPartitionFace())
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

	return false;
}

vector<SFace*> Partition::OcctSplit()
{
	vector<SFace*> faceTemp;
//	BRepAlgoAPI_Section

	return faceTemp;
}

void Partition::JudgeIntersection(SFace* Fa, SFace* Fb)
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
