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
	Partition(TopoDS_Shape& shapes);
	~Partition();


public:

	void LoadStepFromOCCT();// 通过occ读取step文件
	std::string Partition::DumpOrientation(const TopAbs_Orientation& orient);

	void GetSFaceInfo(SetOfstp_face* stpFace);
	void StepConversionAndOutput(stp_representation_item* item, string shapeName, int& m, int& n);
	void NatlHalfVector(stp_advanced_face* adFace);
	void GetAxisData(stp_axis2_placement_3d* axis, GeometryData& data);
	stp_cartesian_point* EdgeCurveStartOrEnd(stp_vertex* ver);
	
public:

	bool IsPartitionFace(vector<SFace*> faceList);
	void OcctSplit(vector<SFace*> faceList, SFace* splitFace);		//occt切割面
	
	/*! \brief 判断面Fa和Fb是否是切割面

	*/
	bool JudgeIntersection(SFace* Fa, SFace* Fb, Standard_CString curveName, orientationFaceA oriA, orientationFaceB oriB,
		EdgeCurveVertex curveA, EdgeCurveVertex curveB, CPoint3D pointA, CPoint3D pointB);
	void FindPartitionFace(SFace* Fa, SFace* Fb);
	SFace* ChoosePartitionFace();
	
	/*! \brief 结构转换SFace->OCCT
	*/
	void CurrentStructToOCCT( SFace* face, TopoDS_Face& aFace );
	
	/*! \brief 结构转换 OCCT->SFace
	*/
	vector<SFace*> OcctToCurrentStruct(TopoDS_Shape aShape);

	void AddNewSplit(TopoDS_Shape Stock, Handle(Geom_Surface)& plane1);
	
	SFace* CloneFace(SFace* face);
	GeometryData* CloneEntity(GeometryData* geo);
	void GetFaceList(vector<SFace*> faceList, SFace* splitFace);
	

	/*! \brief 获取面的信息
	\param vec 面的法线
	\param start edge的起始点
	\param end edge的终点
	\param orient edge的方向
	\param axis 轴
	*/
	void GetFaceAxis(TopoDS_Face face, TopoDS_Edge edge, gp_Vec& vec, gp_Pnt& start, gp_Pnt& end, TopAbs_Orientation& orient, gp_Pnt& axis);
	
	
	void MatrixInformation(TopoDS_Solid solid, gp_Mat& mat, gp_XYZ& xyz);
	bool FindTheIntersectionFace(TopoDS_Shell shell);
	TopoDS_Face ChooseFace();
	bool JudgeIntersection(TopoDS_Face Fa, TopoDS_Face Fb, TopoDS_Edge aEdge);
	void OcctSplit(TopoDS_Shape shape, TopoDS_Face face);
	double CompareNum(double matrix);
	void DumpVertex(const TopoDS_Vertex& v);
	void SplitSolid(TopoDS_Shell& shell);
	void StepConversionAndOutput(TopoDS_Shell shell, string shapeName, int& m, int& n);
public:
	std::vector<std::string> vecOut_;//输出的信息列表
	int mp_;

public:
	vector<SFace*> NatlHalfSpaceList_;			//自然半空间队列
	template < class T>
	string ConvertToString(T value)
	{
		stringstream ss;
		ss << value;
		return ss.str();
	}
private:
	TopoDS_Shape shapes_;
	vector<TopoDS_Face> topoFaceList_;
	multimap<size_t,TopoDS_Face> partList_;
	vector<TopoDS_Face> faceList_;
	TopTools_HSequenceOfShape solids_;
	multimap<size_t, SFace*> partitionFaceList_;
	vector<vector<SFace*>> canSplitFaceList;
	vector<vector<SFace*>> intersectionFaceList_;
	vector<SFace*> test_;
	vector<SFace*> faceInfors_;
	RoseDesign* design_;
	stp_representation_item* item_;
	bool isHasPartitionFace_;

	TopTools_ShapeSet setShapes_;
	map<int, int> repeNum_;
	Repetition repe_;
	int intex_;
	vector<string> TR_;
};

