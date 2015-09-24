#include "StdAfx.h"
#include "MA_ShapeCutter.h"

ShapeCutter::ShapeCutter()
{
}

ShapeCutter::ShapeCutter(const TopoDS_Shape& theBox1,
							   const TopoDS_Shape& theBox2,
							   const TopoDS_Face& theFace,
							   Standard_Boolean Optimization )
: theBox1_(theBox1), theBox2_(theBox2), theFace_(theFace), optimization_(Optimization)
{

}

ShapeCutter::ShapeCutter( const TopoDS_Shape& theBox2,
							    const TopoDS_Face& theFace,
								Standard_Boolean optimization )
: theBox2_(theBox2), theFace_(theFace), optimization_(optimization)
{
	//得到最小外接box的坐标，并生成世界1
	theBox1_ = this->GetBndBox(theBox2);
}

ShapeCutter::~ShapeCutter()
{
}

void ShapeCutter::Init1( const TopoDS_Shape& theBox1 )
{
	theBox1_ = (theBox1);
}

void ShapeCutter::Init2( const TopoDS_Shape& theBox2 )
{
	theBox2_ = (theBox2);
	if (theBox1_.IsNull())
	{
		theBox1_ = this->GetBndBox(theBox2);
	}
}

void ShapeCutter::Init3( const TopoDS_Face& theFace )
{
	theFace_ = (theFace);
}

void ShapeCutter::SetOptimization( Standard_Boolean Optimization/*=Standard_True*/ )
{
	optimization_ = Optimization;
}

const TopTools_ListOfShape& ShapeCutter::SplitShape(const TopoDS_Shape& shape1, const TopoDS_Shape& shape2)
{
	TopoDS_Shape S = shape1;

	//gp_Pln theplane(gp_Pnt(90,90,90),gp_Dir(1,1,1));
	//gp_Pln theplane = plane1;

	//TopoDS_Face theface = BRepBuilderAPI_MakeFace(theplane);
	BRepAlgoAPI_Section asect(S, shape2, Standard_False);
	asect.ComputePCurveOn1(Standard_True);
	asect.Approximation(Standard_True);
	asect.Build();
	TopoDS_Shape R = asect.Shape();

	BRepFeat_SplitShape asplit(S);

	for (TopExp_Explorer Ex1(S,TopAbs_FACE); Ex1.More(); Ex1.Next())
	{
		Handle(Geom_Surface) tempSurface1 = BRep_Tool::Surface( TopoDS::Face(Ex1.Current()) );//当前面
		Handle(Geom_Surface) tempSurface2 = BRep_Tool::Surface( TopoDS::Face(shape2) );//切割面
		GeomAPI_IntSS tempIntSS;
		tempIntSS.Perform(tempSurface1, tempSurface2, 1.0e-7);
		Standard_Boolean isIntersection = tempIntSS.IsDone();
		Standard_Integer nbIntersection = tempIntSS.NbLines();
		TopoDS_Edge tempEdge;
		if ( nbIntersection != 0 )
		{
			tempEdge = BRepBuilderAPI_MakeEdge( tempIntSS.Line(1) );
			//asplit.Add( tempEdge , TopoDS::Face(Ex1.Current()) );
			DisplayShape(tempEdge,Quantity_NOC_RED);
		}
	}

	for (TopExp_Explorer Ex(R,TopAbs_EDGE); Ex.More(); Ex.Next()) 
	{
		TopoDS_Edge anEdge = TopoDS::Edge(Ex.Current());

//  		TopoDS_Shape aFace;
//  		if (asect.HasAncestorFaceOn1(anEdge,aFace)) 
// 		{
//  			TopoDS_Face F = TopoDS::Face(aFace);
//  			TopoDS_Edge E = TopoDS::Edge(anEdge);
//  			asplit.Add(E,F);
//  		}

	for (TopExp_Explorer Ex1(S,TopAbs_FACE); Ex1.More(); Ex1.Next())
	{
		TopoDS_Face eachFace = TopoDS::Face(Ex1.Current());
		Standard_Boolean isAllEdgeOnFace = this->IsAllEdgeOnFace(anEdge, eachFace);
		if ( isAllEdgeOnFace ) 
		{
			asplit.Add(anEdge, eachFace);
			//ShapeUpgrade_FaceDivide myFaceDivide;
			break;
		}
	}
	//DisplayShape(anEdge,Quantity_NOC_RED);
	}

	asplit.Build();
	TopoDS_Shape shape = asplit.Shape();
	TopAbs_ShapeEnum tempShapeEnum = shape.ShapeType();
	//DisplayShape(shape,Quantity_NOC_RED);

	// 	//输出各个面
	// 	Standard_Integer j=0;
	// 	for (TopExp_Explorer Ex1(shape,TopAbs_FACE); Ex1.More(); Ex1.Next(),j++)
	// 	{
	// 		TopoDS_Face eachFace = TopoDS::Face(Ex1.Current());
	// 		DisplayShape(eachFace,Quantity_NOC_RED);
	// // 		if (j==2)
	// // 		{
	// // 			break;
	// // 		}
	// 	}

	//组成TopoDS_Compound试验
	// 	TopoDS_Compound occCompoundS;
	// 	BRep_Builder occBuilder;
	// 	occBuilder.MakeCompound(occCompoundS);
	// 	for(TopExp_Explorer exp(shape, TopAbs_SHELL); exp.More(); exp.Next()){
	// 		TopoDS_Shell shell = TopoDS::Shell(exp.Current());
	// 
	// 		ShapeFix_Solid solid;
	// 		//solid.LimitTolerance(0.01);
	// 		TopoDS_Shape tmpS = solid.SolidFromShell(shell);
	// 		occBuilder.Add(occCompoundS, tmpS);
	// 	}
	// 	DisplayShape(occCompoundS, Quantity_NOC_RED);

	BRepOffsetAPI_Sewing aSewing_1;//将面拼成体
	BRepOffsetAPI_Sewing aSewing_2;//将面拼成体
	//然后遍历这个shape的所有face
	int i = 0;
	for (TopExp_Explorer Ex1(shape,TopAbs_FACE); Ex1.More(); Ex1.Next(),i++) 
	{
		TColStd_ListOfReal theListOfDistance;
		theListOfDistance.Clear();
		TopoDS_Face aCurrentFace = TopoDS::Face(Ex1.Current());

		//遍历每个face下的所有顶点
		for (TopExp_Explorer Ex2(aCurrentFace,TopAbs_VERTEX); Ex2.More(); Ex2.Next()) 
		{
			TopoDS_Vertex aCurrentVertex = TopoDS::Vertex(Ex2.Current());
			gp_Pnt aCurrentPoint = BRep_Tool::Pnt(aCurrentVertex);
			Standard_Real theDistance;

			//计算顶点和 切割平面的距离 根据距离的 正值 和 负值 就可以知道 该face属于平面的哪边了
			//theDistance = theplane.Distance(aCurrentPoint);
			BRepExtrema_DistShapeShape Extrema_DistShapeShape(shape2,aCurrentVertex);
			Extrema_DistShapeShape.Perform();
			theDistance = Extrema_DistShapeShape.Value();//得到点到面的距离
			Standard_Real tempTolerance = BRep_Tool::Tolerance(aCurrentVertex);
			if (theDistance>tempTolerance)
			{
				gp_Pnt pnt_intersect_1 = Extrema_DistShapeShape.PointOnShape1(1);
				gp_Dir dir_1(gp_Vec(pnt_intersect_1,aCurrentPoint));
				Handle(Geom_Surface) tempSurface = BRep_Tool::Surface(TopoDS::Face(shape2));
				//Handle(Geom_Plane) tempPlane = Handle(Geom_Plane)::DownCast(tempSurface);
				gp_Dir dir_2;
				//gp_Dir dir_2(tempPlane->Pln().Axis().Direction());
				//gp_Dir dir_2(theplane.Axis().Direction());
				Normal(TopoDS::Face(shape2), pnt_intersect_1, dir_2);
				Standard_Real tempAngle = dir_1.Angle(dir_2);
				//if ( dir_1.IsOpposite(dir_2,Precision::Angular()) )//若向量方向相反
				if (tempAngle>M_PI/2)
					theDistance = - theDistance;
				theListOfDistance.Append(theDistance);
			}
			else
			{
				theListOfDistance.Append(0.);
			}
		}

		//遍历该面的每个点的距离（有正有负有零）
		TColStd_ListIteratorOfListOfReal distanceListIterator;
		Standard_Integer length = theListOfDistance.Extent();
		Standard_Boolean isAllLargerThanZero = Standard_True;
		for (distanceListIterator.Initialize(theListOfDistance);distanceListIterator.More();distanceListIterator.Next())
		{
			if (distanceListIterator.Value()<0)
			{
				isAllLargerThanZero = Standard_False;
				break;
			}
		}
		if (isAllLargerThanZero)
			aSewing_1.Add(Ex1.Current());
		else
			aSewing_2.Add(Ex1.Current());

	}

	//再加上相交横截面
	TopoDS_Shape ShapeCommon = BRepAlgoAPI_Common(S, shape2);
	aSewing_1.Add(ShapeCommon);
	aSewing_2.Add(ShapeCommon);

	aSewing_1.Perform();
	aSewing_2.Perform();
	TopoDS_Shape sewedShape_1 = aSewing_1.SewedShape();
	TopoDS_Shape sewedShape_2 = aSewing_2.SewedShape();
	TopAbs_ShapeEnum shapeEnum_1 = sewedShape_1.ShapeType();
	TopAbs_ShapeEnum shapeEnum_2 = sewedShape_2.ShapeType();

	//转换成Compound
	TopoDS_Compound occCompoundS_1;
	TopoDS_Compound occCompoundS_2;
	BRep_Builder occBuilder_1;
	BRep_Builder occBuilder_2;
	occBuilder_1.MakeCompound(occCompoundS_1);
	occBuilder_2.MakeCompound(occCompoundS_2);
	for(TopExp_Explorer exp(sewedShape_1, TopAbs_SHELL); exp.More(); exp.Next())
	{
		TopoDS_Shell shell = TopoDS::Shell(exp.Current());

		ShapeFix_Solid solid;
		//solid.LimitTolerance(0.01);
		TopoDS_Shape tmpS = solid.SolidFromShell(shell);
		occBuilder_1.Add(occCompoundS_1, tmpS);
	}

	for(TopExp_Explorer exp(sewedShape_2, TopAbs_SHELL); exp.More(); exp.Next())
	{
		TopoDS_Shell shell = TopoDS::Shell(exp.Current());

		ShapeFix_Solid solid;
		//solid.LimitTolerance(0.01);
		TopoDS_Shape tmpS = solid.SolidFromShell(shell);
		occBuilder_2.Add(occCompoundS_2, tmpS);
	}

	static TopTools_ListOfShape shapeList;
	shapeList.Clear();
	shapeList.Append(occCompoundS_1);
	shapeList.Append(occCompoundS_2);

	return shapeList;
}

void ShapeCutter::Normal(const TopoDS_Face&  aFace,
						  gp_Pnt& point,
						  gp_Dir& NorDir)
{
	BRepAdaptor_Surface S;
	//Standard_Integer i;
	TopLoc_Location l;
	Handle(Geom_Surface) GS = BRep_Tool::Surface(aFace, l);

	Standard_Boolean OK = Standard_True;
	gp_Vec D1U,D1V;
	gp_Vec D2U,D2V,D2UV;
	gp_Pnt P;
	Standard_Real U, V;
	CSLib_DerivativeStatus Status;
	CSLib_NormalStatus NStat;

	GeomLib_Tool::Parameters(GS, point, 0.01, U, V);
	S.Initialize(aFace, Standard_False);
	if (!S.GetType() == GeomAbs_Plane) 
	{
		S.D1(U,V,P,D1U,D1V);
		CSLib::Normal(D1U,D1V,Precision::Angular(),Status,NorDir);
		if (Status != CSLib_Done) {
			S.D2(U,V,P,D1U,D1V,D2U,D2V,D2UV);
			CSLib::Normal(D1U,D1V,D2U,D2V,D2UV,Precision::Angular(),OK,NStat,NorDir);
		}
		if (aFace.Orientation() == TopAbs_REVERSED) (NorDir).Reverse();
	}
	else {
		gp_Dir NPlane;
		S.D1(U,V,P,D1U,D1V);
		CSLib::Normal(D1U,D1V,Precision::Angular(),Status,NPlane);
		if (Status != CSLib_Done) {
			S.D2(U,V,P,D1U,D1V,D2U,D2V,D2UV);
			CSLib::Normal(D1U,D1V,D2U,D2V,D2UV,Precision::Angular(),OK,NStat,NPlane);
		}
		if (aFace.Orientation() == TopAbs_REVERSED) NPlane.Reverse();
		NorDir = (NPlane);

	}
//*/
}

Standard_Boolean ShapeCutter::IsAllEdgeOnFace(const TopoDS_Edge& anEdge, const TopoDS_Face& eachFace )
{
	for (TopExp_Explorer ex1(anEdge,TopAbs_VERTEX); ex1.More(); ex1.Next())
	{
		TopoDS_Vertex aVertex = TopoDS::Vertex( ex1.Current() );
		BRepExtrema_DistShapeShape Extrema_DistShapeShape(aVertex, eachFace);
		Extrema_DistShapeShape.Perform();
		Standard_Real tempTolerance = BRep_Tool::Tolerance(TopoDS::Edge(anEdge));
		Standard_Real theDistance = Extrema_DistShapeShape.Value();//得到点到面的距离
		if (theDistance>tempTolerance)
		{
			return Standard_False;
		}
	}
	return Standard_True;
}

TopoDS_Shape ShapeCutter::GetBndBox( const TopoDS_Shape& theBox2 )
{
	Standard_Real Xmin,Xmax,Ymin,Ymax,Zmin,Zmax;
	Bnd_Box Boite;
	BRepBndLib::Add(theBox2, Boite,Standard_True);
	Boite.Get(Xmin, Ymin, Zmin, Xmax, Ymax, Zmax);
	TopoDS_Shape theBox1 = BRepPrimAPI_MakeBox( gp_Pnt(Xmin,Ymin,Zmin), Xmax-Xmin,Ymax-Ymin,Zmax-Zmin ).Solid();
	return theBox1;
}

void ShapeCutter::Perform()
{
	//若有优化选项，进行Mesh操作
	if (optimization_)
	{
		//BRepMesh::Mesh(m_theBox2,1);
	}

	const TopTools_ListOfShape& theBox1_SplitShape_list = this->SplitShape(theBox1_, theFace_);
	TopTools_ListIteratorOfListOfShape SplitShape_ite(theBox1_SplitShape_list);
	SplitShape_ite.More();
	halfWorld1_ = SplitShape_ite.Value();
	SplitShape_ite.Next();
	halfWorld2_ = SplitShape_ite.Value();

	//this->m_theResult1 = BRepAlgoAPI_Cut(m_theBox2, halfWorld_1);
	//this->m_theResult2 = BRepAlgoAPI_Cut(m_theBox2, halfWorld_2);
}

const TopoDS_Shape& ShapeCutter::CalcResult1()
{
	theResult1_ = BRepAlgoAPI_Cut(theBox2_, halfWorld1_);
	return theResult1_;
}

const TopoDS_Shape& ShapeCutter::CalcResult2()
{
	theResult2_ = BRepAlgoAPI_Cut(theBox2_, halfWorld2_);
	return theResult2_;
}

