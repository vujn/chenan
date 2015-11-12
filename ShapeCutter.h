#pragma once
#include "stdafx.h"
#include "TopTools_HSequenceOfShape.hxx"

class ShapeCutter
{
public:

	ShapeCutter();

	/*!brief theBox1(the world), theBox2 will be cut by theFace.
	*/
	ShapeCutter(const TopoDS_Shape& theBox1,
		const TopoDS_Shape& theBox2,
		const TopoDS_Face& theFace,
		Standard_Boolean Optimization=Standard_True);
	
	/*!brief theBox2 will be cutting by theFace. theBox1(the World) is calc From theBox2. 
	*/
	ShapeCutter(const TopoDS_Shape& theBox2, 
		const TopoDS_Face& theFace,
		Standard_Boolean Optimization=Standard_True);

	~ShapeCutter();

	
public:
	void Init(const TopoDS_Shape& theSolid, const TopoDS_Face& theExtFace);
	void Init1(const TopoDS_Shape& theBox1);
	void Init2(const TopoDS_Shape& theBox2);
	void Init3(const TopoDS_Face& theFace);
	void SetOptimization(Standard_Boolean Optimization=Standard_True);
	void Perform();
	const TopoDS_Shape& CalcResult1();
	const TopoDS_Shape& CalcResult2();

	Standard_Boolean IsLastCut() const;

protected:
	/*!brief split the solid
	*/
	const TopTools_ListOfShape& SplitShape(const TopoDS_Shape& shape1, const TopoDS_Shape& shape2);
	
	void Normal(const TopoDS_Face&  aFace, gp_Pnt& point, gp_Dir& NorDir);
	
	/*!brief check edge is on the face?
	*/
	Standard_Boolean IsAllEdgeOnFace(const TopoDS_Edge& anEdge, const TopoDS_Face& eachFace );
	
	/*!brief get the bnd box
	*/
	TopoDS_Shape GetBndBox(const TopoDS_Shape& theBox2);//get Box1

public:
	TopTools_HSequenceOfShape myResultSolids_;

private:
	TopoDS_Shape theBox1_;
	TopoDS_Shape theBox2_; 
	TopoDS_Face theFace_;
	TopoDS_Shape theResult1_;
	TopoDS_Shape theResult2_;
	TopoDS_Shape halfWorld1_;
	TopoDS_Shape halfWorld2_;
	Standard_Boolean optimization_;
	
	//McCad
	TopoDS_Shape mySolid_;
	Standard_Boolean myIsDone_;
	TopoDS_Shape myExtFace_;
	TopoDS_Shape myPosPartSol_;
	TopoDS_Shape myNegPartSol_;
	Standard_Boolean myLastCut_;

};
