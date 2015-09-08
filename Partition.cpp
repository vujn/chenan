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
		
		//NatlHalfVector(adFace);// ����ÿ����ṹ����Ϣ faceInfors_;
	}
	//�Ƚ��������Ƿ����ཻ�ı� �����->�ж��Ƿ��Ƿָ���; �����������������ж�
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
