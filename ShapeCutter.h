#ifndef _MA_ShapeCutter_HeaderFile
#define _MA_ShapeCutter_HeaderFile

#ifndef _Standard_HeaderFile
#include <Standard.hxx>
#endif
#ifndef _Standard_DefineAlloc_HeaderFile
#include <Standard_DefineAlloc.hxx>
#endif
#ifndef _Standard_Macro_HeaderFile
#include <Standard_Macro.hxx>
#endif
#ifndef _Standard_Boolean_HeaderFile
#include <Standard_Boolean.hxx>
#endif
#ifndef _TopoDS_Shape_HeaderFile
#include <TopoDS_Shape.hxx>
#endif
//class TopoDS_Shape;

class Standard_EXPORT  ShapeCutter
{
public:
	DEFINE_STANDARD_ALLOC

	//Default Constructs
	ShapeCutter();

	//Constructs 1
	//theBox1 is the world, theBox2 will be cutting by theFace.
	ShapeCutter(const TopoDS_Shape& theBox1,
		const TopoDS_Shape& theBox2,
		const TopoDS_Face& theFace,
		Standard_Boolean Optimization=Standard_True);

	//Constructs 2
	//theBox2 will be cutting by theFace. theBox1(the World) is calc From theBox2.
	ShapeCutter(const TopoDS_Shape& theBox2, 
		const TopoDS_Face& theFace,
		Standard_Boolean Optimization=Standard_True);

	~ShapeCutter();

public:

	void Init1(const TopoDS_Shape& theBox1);
	void Init2(const TopoDS_Shape& theBox2);
	void Init3(const TopoDS_Face& theFace);
	void SetOptimization(Standard_Boolean Optimization=Standard_True);
	void Perform();
	const TopoDS_Shape& CalcResult1();
	const TopoDS_Shape& CalcResult2();
	
protected:
	const TopTools_ListOfShape& SplitShape(const TopoDS_Shape& shape1, const TopoDS_Shape& shape2);
	void Normal(const TopoDS_Face&  aFace, gp_Pnt& point, gp_Dir& NorDir);
	Standard_Boolean IsAllEdgeOnFace(const TopoDS_Edge& anEdge, const TopoDS_Face& eachFace );
	TopoDS_Shape GetBndBox(const TopoDS_Shape& theBox2);//get Box1

private:
	TopoDS_Shape theBox1_;
	TopoDS_Shape theBox2_; 
	TopoDS_Face theFace_;
	TopoDS_Shape theResult1_;
	TopoDS_Shape theResult2_;
	TopoDS_Shape halfWorld1_;
	TopoDS_Shape halfWorld2_;
	Standard_Boolean optimization_;
};

#endif

//Usage method 
/*
	//theFace cut theBox2
	ShapeCutter ShapeCutter(theBox2, theFace);
	ShapeCutter.Perform();
	TopoDS_Shape S1 = CalcResult1();
	TopoDS_Shape S2 = CalcResult2();
//*/