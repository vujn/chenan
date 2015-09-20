#pragma once
#include <memory>
#include "GeomKernel.h"
#include "InterProcess.h"
#include "Structure.h"
#include "SFace.h"
#include "Curve.h"

using namespace std;

#define ZOOMTIME 10.0

class StepEntity
{
public:
	StepEntity(vector<SFace*> natlHalfSpaceList);
	~StepEntity(void);
	
public:

	void GenerateSepHalfspace(SFace* face);				//���ɷָ��ռ�
	void GenerateHalfSpaceList();								//���ɰ�ռ����
	void UniqueHalfSpaceList();							//ɾ����ռ�����е��ظ���
	void GenerateCIT();									//���ɹ淶�ཻ��
	void GenerateHalfCharacteristicPoint();			//�淶�ཻ����������㼯��,����CITCPList��CITPList
	void GenerateOffsetHalfSpaceList();				//����ƫ�ư�ռ����,���������а�ռ��ϵ������
	void PMCtest();				//PMC���ԣ�ȷ����Щ�淶�ཻ����ʵ����

	//����MCNP���������ļ�
	void Output(int * p_m, int * p_n, string p_shpnm, vector<string>& vecOut,
		bool p_withvoid, bool p_spdir, bool is_outer, bool is_last);
	
	ConvexConcave MakeBoundFace(vector<Curve*> edgeLoop_);
	
	/*
	��������ADVANCED_FACE���̵�һ���㣬�жϸõ��Ƿ���ADVANCED_FACE֮��
	������ADVANCED_FACE���̲���BOUND֮��,����1���ڱ߽���;����-1���ڱ߽���;����0���ڱ߽��ϣ���������
	*/
	int IsInBound(SFace* ent, CPoint3D TestPoint, InterProcess &mP);

	//����matlab����������Ľ���,��������ӵ�MP�Ľ�����У������ֱ�Ϊ������
	void SetSurfaceIntersectPoints(SFace* Surf1, SFace* Surf2, SFace* Surf3);

	void TriangleSeparate(vector<CPoint3D>& boundFacePtarray, vector<CTriChip*>& vTriList);
	
	//Ψһ���淶�ཻ����������㼯�ϣ�ͬʱҲ�����ɹ淶�ཻ�������㼯�ϵĹ���
	void UniqueHalfCharacteristicPoint();

	ConvexConcave UniqueBoundFaceList();

	//�Ƚ�������ռ��Ƿ���ͬ
	bool CompareSTEPEntity(SFace* face1, SFace* face2);

	stp_cartesian_point* EdgeCurveStartOrEnd(stp_vertex* ver);

	void SaveAxisVertex( GeometryData axis2, ORIENTATION ori, CVector3D& cVector);

	void UniqueAxisVertex();

	GeometryData* CloneEntity(GeometryData* geo);

private:

// 	void ListOfPcurveOrSurfaceInfo(ListOfstp_pcurve_or_surface* pList );
// 	void StpDefRepresentationInfo(stp_definitional_representation* def );
// 	void LineInfo(stp_curve* cur ); 
//	void BSplineCurveWithKnotsInfo(stp_curve*, stp_cartesian_point* eStart, stp_cartesian_point* eEnd);
	void SurfaceCurveInfo(stp_curve* cur, stp_cartesian_point* eStart, stp_cartesian_point* eEnd );
	void CircleInfo(CIRCLE* cir, ORIENTATION ori,CPoint3D eStart, CPoint3D eEnd );
	void EllipseInfo(ELLIPSE* cur, ORIENTATION ori, CPoint3D eStart, CPoint3D eEnd);
//	void GetAxisData(stp_axis2_placement_3d* axis, GeometryData& data);
	CPoint3D GetPoint(stp_cartesian_point* pt);

private:

	RoseDesign* design_;
	vector<EdgeCurveVertex*> axisVertex_;
	vector<SFace*> SepHalfspacePlacementList_;	//�ָ��ռ����
	vector<SFace*> NatlHalfSpaceList_;			//��Ȼ��ռ����
	vector<SFace*> CSGHalfSpaceList_;			//CSG���ʽ��ռ���У�������Ȼ��ռ䣬�ָ��ռ䣩
	vector<SFace*> HalfSpaceList_;				//������ƫ�ư�ռ��ܶ���
	vector<SFace*> ForwardHalfSpaceList_;		//����ƫ�ư�ռ����
	vector<SFace*> BackwardHalfSpaceList_;		//����ƫ�ư�ռ����
	vector<CPoint3D> CITPList_;					//canonical intersection term characteristic points list �淶�ཻ�������㼯��
	vector<CPoint3D> CITCPList_;				//canonical intersection term components characteristic points list �淶�ཻ����������㼯��
	HalfSpaceOrientation ** CIT_;	//�淶�ཻ��
	TermIndex* CITIndex_;			//��ʾ�淶�ཻ���Ƿ���ʵ���ڲ����Ƿ���Ч������
	InterProcess MP_;				//������ά��
	InterProcess MP2D_;				//��ά��
	bool muticp_;					//�Ƿ���ʵ��
	LOGICAL logical_;

};
