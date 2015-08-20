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
	StepEntity( RoseDesign* design );
	~StepEntity(void);

public:
	std::vector<std::string> vecOut_;//输出的信息列表

	void StepConversionAndOutput();	 //转换
	
private:

	void GenerateSepHalfspace(SFace* face);				//生成分割半空间
	void GenerateHalfSpaceList(SetOfstp_face* stpFace);//生成半空间队列
	void UniqueHalfSpaceList();							//删除半空间队列中的重复项
	void GenerateCIT();									//生成规范相交项
	void GenerateHalfCharacteristicPoint();			//规范相交项组件特征点集合,包括CITCPList和CITPList
	void GenerateOffsetHalfSpaceList();				//生成偏移半空间队列,并生成所有半空间的系数数组
	void PMCtest(SetOfstp_face* face);				//PMC测试，确定哪些规范相交项在实体内

	//生成MCNP几何输入文件
	void Output(int * p_m, int * p_n, string p_shpnm, vector<string>& vecOut,
		bool p_withvoid, bool p_spdir, bool is_outer, bool is_last);
	
	ConvexConcave MakeBoundFace(ListOfstp_oriented_edge* oriList);
	
	/*
	给定满足ADVANCED_FACE方程的一个点，判断该点是否在ADVANCED_FACE之内
	（满足ADVANCED_FACE方程并在BOUND之内,返回1，在边界内;返回-1，在边界外;返回0，在边界上，舍弃重算
	*/
	int IsInBound(SFace* ent, CPoint3D TestPoint, InterProcess &mP);

	void NatlHalfVector(stp_advanced_face* adFace);

	//调用matlab求三个曲面的交点,并将其添加到MP的交点队列，参数分别为三个面
	void SetSurfaceIntersectPoints(SFace* Surf1, SFace* Surf2, SFace* Surf3);

	void TriangleSeparate(vector<CPoint3D>& boundFacePtarray, vector<CTriChip*>& vTriList);
	
	//唯一化规范相交项组件特征点集合，同时也是生成规范相交项特征点集合的过程
	void UniqueHalfCharacteristicPoint();

	ConvexConcave UniqueBoundFaceList();

	//比较两个半空间是否相同
	bool CompareSTEPEntity(SFace* face1, SFace* face2);

	stp_cartesian_point* EdgeCurveStartOrEnd(stp_vertex* ver);

	void SaveAxisVertex(stp_axis2_placement* axis2, ORIENTATION ori, CVector3D& cVector);

	void UniqueAxisVertex();

	GeometryData* CloneEntity(GeometryData* geo);

private:

	void ListOfPcurveOrSurfaceInfo(ListOfstp_pcurve_or_surface* pList );
	void StpDefRepresentationInfo(stp_definitional_representation* def );
	void LineInfo(stp_curve* cur ); 
	//void BSplineCurveWithKnotsInfo(stp_curve*, stp_cartesian_point* eStart, stp_cartesian_point* eEnd);
	void SurfaceCurveInfo(stp_curve* cur, stp_cartesian_point* eStart, stp_cartesian_point* eEnd );
	void CircleInfo(stp_curve* cur, ORIENTATION ori,stp_cartesian_point* eStart, stp_cartesian_point* eEnd );
	void EllipseInfo(stp_curve* cur, ORIENTATION ori, stp_cartesian_point* eStart, stp_cartesian_point* eEnd);
	void GetAxisData(stp_axis2_placement_3d* axis, GeometryData& data);
	CPoint3D GetPoint(stp_cartesian_point* pt);

private:

	RoseDesign* design_;
	vector<EdgeCurveVertex*> axisVertex_;
	vector<SFace*> SepHalfspacePlacementList_;	//分割半空间队列
	vector<SFace*> NatlHalfSpaceList_;			//自然半空间队列
	vector<SFace*> CSGHalfSpaceList_;			//CSG表达式半空间队列（包括自然半空间，分割半空间）
	vector<SFace*> HalfSpaceList_;				//正负向偏移半空间总队列
	vector<SFace*> ForwardHalfSpaceList_;		//正向偏移半空间队列
	vector<SFace*> BackwardHalfSpaceList_;		//负向偏移半空间队列
	vector<CPoint3D> CITPList_;					//canonical intersection term characteristic points list 规范相交项特征点集合
	vector<CPoint3D> CITCPList_;				//canonical intersection term components characteristic points list 规范相交项组件特征点集合
	HalfSpaceOrientation ** CIT_;	//规范相交项
	TermIndex* CITIndex_;			//表示规范相交项是否在实体内部和是否有效的数组
	InterProcess MP_;				//用于三维求交
	InterProcess MP2D_;				//二维求交
	bool muticp_;					//是否复杂实体
	LOGICAL logical_;
	vector<stp_face_bound*> faceBound_;
};
