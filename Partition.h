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

	void LoadStepFromOCCT();// load info form occ
	std::string Partition::DumpOrientation(const TopAbs_Orientation& orient);

	/*!brief
	*/
	void GetSFaceInfo(SetOfstp_face* stpFace);
	void StepConversionAndOutput(stp_representation_item* item, string shapeName, int& m, int& n);
	
	void NatlHalfVector(stp_advanced_face* adFace);
	void GetAxisData(stp_axis2_placement_3d* axis, GeometryData& data);
	stp_cartesian_point* EdgeCurveStartOrEnd(stp_vertex* ver);
	
public:

	bool IsPartitionFace(vector<SFace*> faceList);
	void OcctSplit(vector<SFace*> faceList, SFace* splitFace);
	
	/*! \brief judge splitFace 
	*/
	bool JudgeIntersection(SFace* Fa, SFace* Fb, Standard_CString curveName, orientationFaceA oriA, orientationFaceB oriB,
		EdgeCurveVertex curveA, EdgeCurveVertex curveB, CPoint3D pointA, CPoint3D pointB);
	void FindPartitionFace(SFace* Fa, SFace* Fb);
	SFace* ChoosePartitionFace();
	
	/*! \brief structSFace->OCCT
	*/
	void CurrentStructToOCCT( SFace* face, TopoDS_Face& aFace );
	
	/*! \brief struct OCCT->SFace
	*/
	vector<SFace*> OcctToCurrentStruct(TopoDS_Shape aShape);

	void AddNewSplit(TopoDS_Shape Stock, Handle(Geom_Surface)& plane1);
	
	/*! \brief get face info
	\param vec: face vec
	\param start: edge start
	\param end: edge end
	\param orient: edge dir
	\param axis: axis
	*/
	void GetFaceAxis(TopoDS_Face face, TopoDS_Edge edge, 
		gp_Vec& vec, gp_Pnt& start, gp_Pnt& end, 
		TopAbs_Orientation& wireOrient, TopAbs_Orientation& orient,
		gp_Pnt& axis);
	
	/*!brief get matrix
	*/
	void MatrixInformation(TopoDS_Solid solid, gp_Mat& mat, gp_XYZ& xyz);
	
	/*!brief find the intersection face in the shell
	*/
	bool FindTheIntersectionFace(TopoDS_Shell shell);
	
	/*!brief choose split face
	*/
	TopoDS_Face ChooseFace();
	
	/*!brief judge splitFace
	*/
	bool JudgeIntersection(TopoDS_Face Fa, TopoDS_Face Fb, TopoDS_Edge aEdge);

	/*!brief split
	SFace
	*/
	void OcctSplit(TopoDS_Shape shape, TopoDS_Face face);

	double CompareNum(double matrix);
	void DumpVertex(const TopoDS_Vertex& v);

	/*!brief split
	occt
	*/
	void SplitSolid(TopoDS_Shell& shell);

	/*!brief conversion
	*/
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

