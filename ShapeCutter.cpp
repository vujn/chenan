#include "stdafx.h"
#include "ShapeCutter.h"


ShapeCutter::ShapeCutter()
{

}

ShapeCutter::ShapeCutter(const TopoDS_Shape& theBox1,
	const TopoDS_Shape& theBox2,
	const TopoDS_Face& theFace,
	Standard_Boolean optimization )
: theBox1_(theBox1), theBox2_(theBox2), theFace_(theFace), optimization_(optimization)
{

}

ShapeCutter::ShapeCutter( const TopoDS_Shape& theBox2,
	const TopoDS_Face& theFace,
	Standard_Boolean optimization )
: theBox2_(theBox2), theFace_(theFace), optimization_(optimization)
{
	//得到最小外接box的坐标，并生成世界1
//	theBox1_ = GetBndBox(theBox2);
	Init2(theBox2);
}

ShapeCutter::~ShapeCutter()
{
}

void ShapeCutter::Init1( const TopoDS_Shape& theBox1 )
{
	theBox1_ = theBox1;
}

void ShapeCutter::Init2( const TopoDS_Shape& theBox2 )
{
	theBox2_ = theBox2;
	if (theBox1_.IsNull())
		theBox1_ = GetBndBox(theBox2_);
}

void ShapeCutter::Init3( const TopoDS_Face& theFace )
{
	theFace_ = theFace;
}

void ShapeCutter::SetOptimization( Standard_Boolean optimization/*=Standard_True*/ )
{
	optimization_ = optimization;
}

Standard_Boolean ShapeCutter::IsLastCut() const
{
	return myLastCut_;
}

void ShapeCutter::Init(const TopoDS_Shape& theSolid, const TopoDS_Shape& theExtFace)
{
	myIsDone_ = Standard_False;
	mySolid_ = theSolid;
	gp_Pnt posPnt(0, 0, 0);
//	posPnt = myExtFace_->GetPosPnt();
	gp_Pnt negPnt(0, 0, 0);
//	negPnt = myExtFace_->GetNegPnt();


	///////////////////////////////////////////////////////////////////////////////////////////////////
	//  this is a test for extents and evaluation
	TopoDS_Face aFace;
//	TopoDS_Face aFace = myExtFace_->GetFace();

	BRepAdaptor_Surface BS(aFace, Standard_True);
	GeomAdaptor_Surface theAdaptFaceSurface = BS.Surface();

	////////////////////////////////////////////////////////////////
	// construct cutting half spaces
	TopoDS_Shape PosHSol = BRepPrimAPI_MakeHalfSpace(aFace, posPnt).Solid(); //positive cutting half space SOLID
	if(PosHSol.IsNull())
	{
		printf("\n_#_ShapeCutter.cxx :: Solid of positive cutting half space is empty!!!");
		return;
	}
	TopoDS_Shape NegHSol = BRepPrimAPI_MakeHalfSpace(aFace, negPnt).Solid(); //negative cutting half space SOLID
	if(NegHSol.IsNull())
	{
		printf("\n_#_ShapeCutter.cxx :: Solid of negative cutting half space is empty!!!");
		return;
	}

	Standard_Boolean pHalfCut = Standard_False;
	Standard_Boolean nHalfCut = Standard_False;

	////////////////////////////////////////////////////////////////////////////
	//
	Bnd_Box B;
	BRepBndLib::Add(mySolid_, B);
	B.SetGap(1000.0);
	Standard_Real aXmin, aYmin, aZmin, aXmax, aYmax, aZmax;
	B.Get(aXmin, aYmin, aZmin, aXmax, aYmax, aZmax);
	TopoDS_Shape aBox = BRepPrimAPI_MakeBox(gp_Pnt(aXmin, aYmin, aZmin), gp_Pnt(aXmax, aYmax, aZmax)).Shape();

	//////////////////////////////////////////////////////////////////////////
	// create pseudo half spaces (shrink to suitable size)
	TopoDS_Shape PosBox, NegBox;
	try
	{
		BRepAlgoAPI_Common posHCommon(PosHSol, aBox);
		if(posHCommon.IsDone())
		{
			PosBox = posHCommon.Shape();
			PosHSol = PosBox;
		}
		posHCommon.Destroy();
	}
	catch(Standard_DomainError)
	{
		Standard_Failure::Caught()->Print(cout); cout << endl;
	}
	try
	{
		BRepAlgoAPI_Common negHCommon(NegHSol, aBox);
		if(negHCommon.IsDone())
		{
			NegBox = negHCommon.Shape();
			NegHSol = NegBox;
		}
		negHCommon.Destroy();
	}
	catch(Standard_DomainError)
	{
		Standard_Failure::Caught()->Print(cout); cout << endl;
	}


	int itol = 0, highTolCount = 0;

	//////////////////////////////////////////////////////////////
	// JUMP-MASTER
MASTER: //goto label; if cutting fails: try again using a higher tolerance

	////////////////////////////////////////////////////////////////////////
	// cutting with negative half space
	if(NegHSol.IsNull())
	{
		printf("_#_ShapeCutter.cxx :: A negative  Null Cut appears!!");
		return;
	}
	else
	{
		try
		{
			BRepAlgoAPI_Cut posCut(mySolid_, NegHSol);

			if(posCut.IsDone())
			{
				pHalfCut = Standard_True;
				myPosPartSol_ = posCut.Shape();
			}
			else
			{
				pHalfCut = Standard_False;
				printf("_#_ShapeCutter.cxx :: Positive side solid Cutting failed !!");
			}
			posCut.Destroy();
		}
		catch(Standard_DomainError)
		{
			pHalfCut = Standard_False;
			printf("_#_ShapeCutter.cxx :: Boolean Operation on a halfspace has failed !!!");
			cout << "OCC error message:  ";
			Standard_Failure::Caught()->Print(cout); cout << endl;
		}
	}

	//////////////////////////////////////////////////////////////////////////////
	// cutting with positive half space
	if(PosHSol.IsNull())
	{
		printf("_#_ShapeCutter.cxx :: A positive  Null Cut appears!!");
		return;
	}
	else
	{
		try
		{
			BRepAlgoAPI_Cut negCut(mySolid_, PosHSol);

			if(negCut.IsDone())
			{
				nHalfCut = Standard_True;
				myNegPartSol_ = negCut.Shape();
			}
			else
			{
				nHalfCut = Standard_False;
				printf("_#_ShapeCutter.cxx :: Negative side solid cutting failed !!");
				return;
			}
			negCut.Destroy();
		}
		catch(Standard_DomainError)
		{
			nHalfCut = Standard_False;
			printf("_#_ShapeCutter.cxx :: Boolean Operation on a halfspace failed !!!");
			cout << "OCC error :  ";
			Standard_Failure::Caught()->Print(cout); cout << endl;
		}
	}


	int sCount = 0;

	////////////////////////////////////////////////////////////////
	// check if positive part is valid
	Standard_Boolean posPartNotValid(Standard_False);

	for(TopExp_Explorer ex(myPosPartSol_, TopAbs_SOLID); ex.More(); ex.Next())
	{
		sCount++;
		TopoDS_Solid solid = TopoDS::Solid(ex.Current());

		if(solid.IsNull())
		{
			TCollection_AsciiString warningMessage("_#_ShapeCutter.cxx :: NULL Cut: P Cutting solid failed !!\n    Trying to use Boolean Common operation...");
			printf(warningMessage.ToCString());
			posPartNotValid = Standard_True;
			break;
		}

		BRepCheck_Analyzer BA(solid, Standard_True);

		if(!BA.IsValid())
		{
			posPartNotValid = Standard_True;
			break;
		}
	}

	///////////////////////////////////////////////////////////
	// if cutting failed, try common operation
	if(posPartNotValid)
	{
		sCount = 0;

		try
		{
			BRepAlgoAPI_Common negCommon(mySolid_, NegHSol);

			if(negCommon.IsDone())
			{
				pHalfCut = Standard_True;
				myPosPartSol_ = negCommon.Shape();
			}
			else
			{
				pHalfCut = Standard_False;
				printf("_#_ShapeCutter.cxx :: Positive side solid Common Operation failed !!");
			}
			negCommon.Destroy();
		}
		catch(Standard_DomainError)
		{
			pHalfCut = Standard_False;
			cout << "OCC error message:  ";
			Standard_Failure::Caught()->Print(cout); cout << endl;
		}

		for(TopExp_Explorer ex(myPosPartSol_, TopAbs_SOLID); ex.More(); ex.Next())
		{
			sCount++;
			TopoDS_Solid solid = TopoDS::Solid(ex.Current());

			if(solid.IsNull())
			{
				printf("_#_ShapeCutter.cxx :: NULL Common: P Common solid failed !!");
				pHalfCut = Standard_False;
				break;
			}

			BRepCheck_Analyzer BA(solid, Standard_True);

			if(!BA.IsValid())
			{
				pHalfCut = Standard_False;
				break;
			}
		}
	}

	if(sCount == 0)
		pHalfCut = Standard_False;

	sCount = 0;

	//////////////////////////////////////////////////////////
	// check if negative part is valid
	Standard_Boolean negPartNotValid(Standard_False);

	for(TopExp_Explorer ex(myNegPartSol_, TopAbs_SOLID); ex.More(); ex.Next())
	{
		sCount++;
		TopoDS_Solid solid = TopoDS::Solid(ex.Current());

		if(solid.IsNull())
		{
			printf("_#_ShapeCutter.cxx :: NULL Cut: N Cutting solid failed !!");
			negPartNotValid = Standard_True;
			break;
		}

		BRepCheck_Analyzer BA(solid, Standard_True);

		if(!BA.IsValid())
		{
			negPartNotValid = Standard_True;

			break;
		}
	}
	////////////////////////////////////////////////////////
	// if cutting failed, try common operation
	if(negPartNotValid)
	{
		sCount = 0;
		try
		{
			BRepAlgoAPI_Common negCommon(mySolid_, NegHSol);

			if(negCommon.IsDone())
			{
				nHalfCut = Standard_True;
				myNegPartSol_ = negCommon.Shape();
			}
			else
			{
				nHalfCut = Standard_False;
				printf("_#_ShapeCutter.cxx :: Negative side solid Common Operation failed !!");
			}
			negCommon.Destroy();
		}
		catch(Standard_DomainError)
		{
			nHalfCut = Standard_False;
			printf("_#_ShapeCutter.cxx :: Boolean Operation on a halfspace has failed !!!");
			cout << "OCC error message:  ";
			Standard_Failure::Caught()->Print(cout); cout << endl;
		}

		for(TopExp_Explorer ex(myNegPartSol_, TopAbs_SOLID); ex.More(); ex.Next())
		{
			sCount++;
			TopoDS_Solid solid = TopoDS::Solid(ex.Current());

			if(solid.IsNull())
			{
				printf("_#_ShapeCutter.cxx :: NULL Common: N Common solid failed !!");
				nHalfCut = Standard_False;
				break;
			}

			BRepCheck_Analyzer BA(solid, Standard_True);

			if(!BA.IsValid())
			{
				nHalfCut = Standard_False;
				break;
			}
		}
	}

	if(sCount == 0)
		nHalfCut = Standard_False; // if check is run on compound containing no solid

	///////////////////////////////////////////////////////////////////
	// Test if a boolean operation failed
	sCount = 0;
	itol++;
	Standard_Real highTol(1.e-3);

	if((!nHalfCut || !pHalfCut) && itol <= 3)
	{
		Standard_Real locTol = 1.e-07*(pow(10., itol));
		highTol = locTol;
		cout << "_#_ShapeCutter.cxx :: Tolerance correction due to failed boolean operation.  --- " << itol << endl;
		cout << "                              Tolerance is :  " << locTol << endl;
		
		ShapeFix_ShapeTolerance setter; 
		Standard_Boolean sett;
		//sett = setter.LimitTolerance(mySolid,locTol,0.0,TopAbs_SOLID);
		sett = setter.LimitTolerance(PosHSol, locTol, 0.0, TopAbs_SOLID);
		sett = setter.LimitTolerance(NegHSol, locTol, 0.0, TopAbs_SOLID);
		goto MASTER;
	}

	////////////////////////////////////////////////////////////////
	if((!pHalfCut || !nHalfCut) && !IsLastCut())
	{
		printf("_#_ShapeCutter.cxx :: Warning: Cutting failed may try again !!");
		myIsDone_ = Standard_False;
		return;
	}
	////////////////////////////////////////////////////////////////
	// if cutting certainly fails we try common;
	// hihg tolerance BOP
	if((!pHalfCut || !nHalfCut) && IsLastCut() && highTolCount < 1)
	{
		highTolCount++;
		Standard_Real locTol = highTol;
		highTol = locTol*(pow(10., highTolCount));

		cout << "******************************************************************************" << endl;
		cout << highTolCount << ".  High Tolerance correction due to failed boolean operation. " << endl;
		cout << "Warning: Tolerance is high may introduce error :  " << highTol << endl;
		cout << "******************************************************************************" << endl;

		ShapeFix_ShapeTolerance setter;
		Standard_Boolean sett;
		sett = setter.LimitTolerance(PosHSol, highTol, 0.0, TopAbs_SOLID);
		sett = setter.LimitTolerance(NegHSol, highTol, 0.0, TopAbs_SOLID);
		goto MASTER;
	}

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//expand by  shell shells

	for(TopExp_Explorer ex(myPosPartSol_, TopAbs_SHELL); ex.More(); ex.Next())
	{
		TopoDS_Shell tmpShell = TopoDS::Shell(ex.Current());
		//////////////////////////////////////////////////////////////////////////
		BRepCheck_Shell shellCheck(tmpShell);
		//BRepCheck::Print(shellCheck.Orientation(),cout);cout<< endl;
		if(shellCheck.Closed(Standard_False) == BRepCheck_NoError)
		{
			BRepBuilderAPI_MakeSolid Bu(tmpShell);
			if(Bu.IsDone())
			{
				TopoDS_Solid aSolid = Bu.Solid();
				if(aSolid.IsNull())
					printf("_#_ShapeCutter.cxx :: Warning: Null solid in Positive part!!");
				BRepClass3d_SolidClassifier bsc3d(aSolid);
				Standard_Real t = Precision::Confusion();
				bsc3d.PerformInfinitePoint(t);

				if(bsc3d.State() == TopAbs_IN)
					cout << "Infinity in Solid, therefore discaded !! " << endl;
				else
					myResultSolids_.Append(aSolid);
			}
			else
			{
				printf("_#_ShapeCutter.cxx :: Model may be invalid: Solid computation from shell failed!!");
			}
		}
		else
			printf("_#_ShapeCutter.cxx :: Model may be invalid - open shell");
	}

	//  cout << " Total number of shells after cutting   " << endl;

	for(TopExp_Explorer ex(myNegPartSol_, TopAbs_SHELL); ex.More(); ex.Next())
	{

		TopoDS_Shell tmpShell = TopoDS::Shell(ex.Current());
		BRepCheck_Shell shellCheck(tmpShell);
		//	BRepCheck::Print(shellCheck.Orientation(),cout); cout << endl;
		if(shellCheck.Closed(Standard_False) == BRepCheck_NoError)
		{
			BRepBuilderAPI_MakeSolid Bu(tmpShell);
			if(Bu.IsDone())
			{
				TopoDS_Solid aSolid = Bu.Solid();
				if(aSolid.IsNull())
					printf("_#_ShapeCutter.cxx :: Warning: Null solid in Positive part!!");

				BRepClass3d_SolidClassifier bsc3d(aSolid);
				Standard_Real t = Precision::Confusion();
				bsc3d.PerformInfinitePoint(t);

				if(bsc3d.State() == TopAbs_IN)
					printf("_#_ShapeCutter.cxx :: Infinity in Solid, therefore discarded !! ");
				else
					myResultSolids_.Append(aSolid);
			}
			else
			{
				printf("_#_ShapeCutter.cxx :: Model may be invalid: Solid computation from shell failed!! ");
			}
		}
		else
		{
			printf("_#_ShapeCutter.cxx :: Model may be invalid - open shell!!");
		}
		/////////////////////////////////////////////////////////////////////////
	}

	////////////////////////////////////////////////////////////////////////////////////////////////////
	// content analysis of result solids
	//cout << "////////////////////////////////////////////////////////////////////////////////// " << endl;
	//cout << " Cut Result =  " << myResultSolids->Length() << endl;
	
	if(!IsLastCut() && myResultSolids_.Length() <= 1)
	{
		printf("_#_ShapeCutter.cxx :: Warning: Cut result may be degenerate !!");
		myIsDone_ = Standard_False;
		return;
	}
	TopExp_Explorer ex;
	int iv;
	for(int i = 1; i <= myResultSolids_.Length(); i++)
	{
		for(ex.Init(myResultSolids_.Value(i), TopAbs_FACE), iv = 1; ex.More(); ex.Next(), iv++)
		{
			TopoDS_Face theFace = TopoDS::Face(ex.Current());
			BRepAdaptor_Surface BS(theFace, Standard_True);
			GeomAdaptor_Surface theAdaptFaceSurface = BS.Surface();
			if(theAdaptFaceSurface.GetType() != GeomAbs_Plane)
			{
				cout << "Solid:  "<< i << " Face =  " << iv << "    non-planar face .....................!!"<< endl;
			}
			else
			{
				gp_Pln aPln = theAdaptFaceSurface.Plane();
				Standard_Real A, B, C, D;
				aPln.Coefficients(A, B, C, D);
				//	cout << "Solid:  "<< i << " Face =  " << iv << "  A =  " << A << " B =  " << B << " C =  " << C <<" D =  " << D << endl;
			}
		}
	}
	myIsDone_ = Standard_True;
}


const TopTools_ListOfShape& ShapeCutter::SplitShape(const TopoDS_Shape& shape1, const TopoDS_Shape& shape2)
{
	TopoDS_Shape S = shape1;
	////////////////////////////test///////////////////////////////////
// 	gp_Pln thePlane(gp_Pnt(90,90,90),gp_Dir(1,1,1));
// 	TopoDS_Shape SS = BRepBuilderAPI_MakeFace(thePlane);
// 	BRepTools::Write(shape1, "E:\\test.txt");
	////////////////////////////test///////////////////////////////////
//	BRepTools::Write(S, "E:\\test.txt");

// 	BRepAlgoAPI_Cut jjj(shape1, shape2);
// 	BRepTools::Write(jjj, "E:\\test.txt");
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
		}
	}

	for (TopExp_Explorer Ex(R,TopAbs_EDGE); Ex.More(); Ex.Next()) 
	{
		TopoDS_Edge anEdge = TopoDS::Edge(Ex.Current());
		TopoDS_Shape aFace;
// 		if(asect.HasAncestorFaceOn1(anEdge, aFace))
// 		{
// 			TopoDS_Face F = TopoDS::Face(aFace);
// 			TopoDS_Edge E = TopoDS::Edge(anEdge);
// 			asplit.Add(E, F);
// 		}

		for (TopExp_Explorer Ex1(S,TopAbs_FACE); Ex1.More(); Ex1.Next())
		{
			TopoDS_Face eachFace = TopoDS::Face(Ex1.Current());
			BRepTools::Write(eachFace, "E:\\test.txt");

			Standard_Boolean isAllEdgeOnFace = IsAllEdgeOnFace(anEdge, eachFace);
			if ( isAllEdgeOnFace ) 
			{
				asplit.Add(anEdge, eachFace);
				break;
			}
		}
	}

	asplit.Build();
	TopoDS_Shape shape = asplit.Shape();
	TopAbs_ShapeEnum tempShapeEnum = shape.ShapeType();

// 	//输出各个面
// 	Standard_Integer j=0;
// 	for (TopExp_Explorer Ex1(shape,TopAbs_FACE); Ex1.More(); Ex1.Next(),j++)
// 	{
// 		TopoDS_Face eachFace = TopoDS::Face(Ex1.Current());
// 		DisplayShape(eachFace,Quantity_NOC_RED);
// 		if (j==2)
// 		{
// 			break;
// 		}
// 	}

	//组成TopoDS_Compound试验
// 	TopoDS_Compound occCompoundS;
// 	BRep_Builder occBuilder;
// 	occBuilder.MakeCompound(occCompoundS);
// 	for(TopExp_Explorer exp(shape, TopAbs_SHELL); exp.More(); exp.Next())
// 	{
// 		TopoDS_Shell shell = TopoDS::Shell(exp.Current());
// 		ShapeFix_Solid solid;
// 		//solid.LimitTolerance(0.01);
// 		TopoDS_Shape tmpS = solid.SolidFromShell(shell);
// 		occBuilder.Add(occCompoundS, tmpS);
// 	}

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
				theListOfDistance.Append(0.);
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

void ShapeCutter::Normal(const TopoDS_Face&  aFace,gp_Pnt& point, gp_Dir& norDir)
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
		CSLib::Normal(D1U,D1V,Precision::Angular(),Status,norDir);
		if (Status != CSLib_Done)
		{
			S.D2(U,V,P,D1U,D1V,D2U,D2V,D2UV);
			CSLib::Normal(D1U,D1V,D2U,D2V,D2UV,Precision::Angular(),OK,NStat,norDir);
		}
		if (aFace.Orientation() == TopAbs_REVERSED) (norDir).Reverse();
	}
	else 
	{
		gp_Dir nPlane;
		S.D1(U,V,P,D1U,D1V);
		CSLib::Normal(D1U,D1V,Precision::Angular(),Status,nPlane);
		if (Status != CSLib_Done)
		{
			S.D2(U,V,P,D1U,D1V,D2U,D2V,D2UV);
			CSLib::Normal(D1U,D1V,D2U,D2V,D2UV,Precision::Angular(),OK,NStat,nPlane);
		}
		if (aFace.Orientation() == TopAbs_REVERSED) nPlane.Reverse();
		norDir = (nPlane);

	}
}

Standard_Boolean ShapeCutter::IsAllEdgeOnFace(const TopoDS_Edge& anEdge, const TopoDS_Face& eachFace )
{
	for (TopExp_Explorer ex1(anEdge,TopAbs_VERTEX); ex1.More(); ex1.Next())
	{
		TopoDS_Vertex aVertex = TopoDS::Vertex( ex1.Current() );
		BRepExtrema_DistShapeShape extremaDistShapeShape(aVertex, eachFace);
		extremaDistShapeShape.Perform();
		Standard_Real tempTolerance = BRep_Tool::Tolerance(TopoDS::Edge(anEdge));
		Standard_Real theDistance = extremaDistShapeShape.Value();//得到点到面的距离
		if (theDistance>tempTolerance)
			return Standard_False;
	}
	return Standard_True;
}

TopoDS_Shape ShapeCutter::GetBndBox( const TopoDS_Shape& theBox2 )
{

	gp_Ax2 axe(gp_Pnt(10, 10, 10), gp_Dir(1, 2, 1));
	TopoDS_Shape theBox = BRepPrimAPI_MakeBox(axe, 60, 80, 100);
	TopoDS_Shape theWedge = BRepPrimAPI_MakeWedge(60., 100., 80., 20.);
	TopoDS_Shape theCommonSurface = BRepAlgoAPI_Common(theBox, theWedge);

	BRepTools::Write(theBox2_, "E:\\test.txt");
	Standard_Real Xmin,Xmax,Ymin,Ymax,Zmin,Zmax;
	Bnd_Box boite;
	BRepBndLib::Add(theCommonSurface, boite);
	boite.SetGap(1000.0);
	boite.Get(Xmin, Ymin, Zmin, Xmax, Ymax, Zmax);
	TopoDS_Shape theBox1 =
		BRepPrimAPI_MakeBox(gp_Pnt(Xmin, Ymin, Zmin), gp_Pnt(Xmax, Ymax, Zmax)).Shape();
	if(theBox1.IsNull())
	{
		printf("Get world failed!\n");
	}
	return theBox1;
}

void ShapeCutter::Perform()
{
	//若有优化选项，进行Mesh操作
	if (optimization_)
	{
		//BRepMesh::Mesh(theBox2_,1);
	}
	const TopTools_ListOfShape& theBox1SplitShapelist = SplitShape(theBox2_, theFace_);
	TopTools_ListIteratorOfListOfShape splitShapeIte(theBox1SplitShapelist);
	splitShapeIte.More();
	halfWorld1_ = splitShapeIte.Value();
	splitShapeIte.Next();
	halfWorld2_ = splitShapeIte.Value();
}

const TopoDS_Shape& ShapeCutter::CalcResult1()
{
	theResult1_ = halfWorld1_;
	return theResult1_;
}

const TopoDS_Shape& ShapeCutter::CalcResult2()
{
	theResult2_ = halfWorld2_;
	return theResult2_;
}

