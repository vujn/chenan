
#include "stdafx.h"
#include "Partition.h"
#include <BRepPrimAPI_MakeSphere.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakeCone.hxx>
#include "GC_MakePlane.hxx"
#include "BRepBuilderAPI_MakeVertex.hxx"
#include <TopoDS.hxx>



Partition::Partition(RoseDesign* design)
	:design_(design)
{
	
}
Partition::Partition(TopoDS_Shape& shapes)
		: shapes_(shapes)
{
	LoadStepFromOCCT();
}

Partition::~Partition()
{

}

void Partition::LoadStepFromOCCT()
{
	TopoDS_Solid solid;
	TopoDS_Shell shell;
	TopoDS_Face aFace;
	TopoDS_Wire aWire;
	TopoDS_Edge aEdge;
	TopoDS_Vertex aVertex;
	TopExp_Explorer expSolid, expShell, expEdge, expWire, expFace, expVertex;

	for (expSolid.Init(shapes_, TopAbs_SOLID); expSolid.More(); expSolid.Next())
	{
		solid = TopoDS::Solid(expSolid.Current());

		////////////////////////////旋转平移信息////////////////////////////
		TopLoc_Location so = solid.Location();
		gp_Trsf trsf = so.Transformation();
		gp_XYZ xyz;
		trsf.Transforms(xyz);
		gp_Mat matrix = trsf.VectorialPart();
		Standard_Real matrix1 = matrix.Value(3, 3);
		printf("%.3f %.3f %.3f\n", xyz.X(), xyz.Y(), xyz.Z());
		//////////////////////////旋转平移信息//////////////////////////////

		for (expShell.Init(solid, TopAbs_SHELL); expShell.More(); expShell.Next())
		{
			shell = TopoDS::Shell(expShell.Current());
			for (expFace.Init(shell, TopAbs_FACE); expFace.More(); expFace.Next())
			{
				aFace = TopoDS::Face(expFace.Current());
				BRepTools::Write(aFace, "e:\\test1.brep");
				for (expWire.Init(aFace, TopAbs_WIRE); expWire.More(); expWire.Next())
				{
					aWire = TopoDS::Wire(expWire.Current());
					BRepTools::Write(aWire, "e:\\test2.brep");
					for (expEdge.Init(aWire, TopAbs_EDGE); expEdge.More(); expEdge.Next())
					{
						aEdge = TopoDS::Edge(expEdge.Current());
						TopAbs_Orientation orientEdge = aEdge.Orientation();
						vector<CPoint3D> pointL;
						for (expVertex.Init(aEdge, TopAbs_VERTEX); expVertex.More(); expVertex.Next())
						{
							gp_Pnt startPnt1 = BRep_Tool::Pnt(TopExp::FirstVertex(aEdge, Standard_True));
							gp_Pnt endPnt1 = BRep_Tool::Pnt(TopExp::LastVertex(aEdge, Standard_True));
							aVertex = TopoDS::Vertex(expVertex.Current());
							gp_Pnt Pnt = BRep_Tool::Pnt(aVertex);
							CPoint3D point(Pnt.X(), Pnt.Y(), Pnt.Z());
						}
					}
				}

				/////////////////////face///////////////////////////////
				gp_Vec test = GetFaceAxis(aFace);
			}
		}
	}
}

gp_Vec Partition::GetFaceAxis(TopoDS_Face face)
{
	gp_Vec vec;
	TopLoc_Location location;
	TopAbs_Orientation orient = face.Orientation();
	Handle(Geom_Surface) aGeometricSurface = BRep_Tool::Surface(face, location);
	GeomAdaptor_Surface msurface(aGeometricSurface);
	switch (msurface.GetType())
	{
	case GeomAbs_Plane:
	{
		gp_Pln agpPlane = msurface.Plane();
		gp_Ax1 norm = agpPlane.Axis();
		gp_Dir dir = norm.Direction();
		vec = dir;
		break;
	}
	
	case GeomAbs_Cylinder:
	{
		gp_Cylinder aCy = msurface.Cylinder();
		gp_Dir d = aCy.Axis().Direction();
		gp_Pnt p = aCy.Location();
		vec = d;
		break;
	}

	case GeomAbs_Sphere:
	{
		gp_Sphere sph = msurface.Sphere();
		Standard_Real radius = sph.Radius();
		gp_Ax3 ax3 = sph.Position();
		gp_Dir dir = ax3.Direction();
		vec = dir;
		gp_Pnt pnt = ax3.Location();
		break;
	}
		
	case GeomAbs_Cone:
	{
		gp_Cone cone = msurface.Cone();
		Standard_Real radius = cone.RefRadius();
		Standard_Real ang = cone.SemiAngle();
		break;
	}
	default:
		break;
	}
	return vec;
}

std::string Partition::DumpOrientation(const TopAbs_Orientation& orient)
{
	std::string strType;
	switch (orient)
	{
	case TopAbs_FORWARD:
		strType = "TopAbs_FORWARD";
		break;
	case TopAbs_REVERSED:
		strType = "TopAbs_REVERSED";
		break;
	case TopAbs_INTERNAL:
		strType = "TopAbs_INTERNAL";
		break;
	case TopAbs_EXTERNAL:
		strType = "TopAbs_EXTERNAL";
		break;
	}
	return strType;
}

void Partition::StepConversionAndOutput(stp_representation_item* item,string shapeName, int& m, int& n)
{
	if (!strcmp("manifold_solid_brep", item->className()))
	{
		stp_manifold_solid_brep* solidBrep = ROSE_CAST(stp_manifold_solid_brep, item);
		stp_closed_shell* shell = solidBrep->outer();
		SetOfstp_face* face = shell->cfs_faces();
		GetSFaceInfo(face);

// 		if (IsPartitionFace(NatlHalfSpaceList_))
// 		{
			SFace* partFace = ChoosePartitionFace();
			OcctSplit(NatlHalfSpaceList_, partFace);
			intersectionFaceList_.push_back(NatlHalfSpaceList_);
//		}
		for (auto iter = 0; iter < intersectionFaceList_.size(); iter++)
		{
			StepEntity* step = new StepEntity(intersectionFaceList_[iter]);
			step->GenerateHalfSpaceList();
			step->GenerateCIT();
			step->GenerateHalfCharacteristicPoint();
			step->PMCtest();
			step->Output(&m, &n, iter, shapeName, vecOut_, false, false, false, false);
		}
		mp_ = m;
		vector<vector<SFace*>>().swap(intersectionFaceList_);
	}
	if (!strcmp("brep_with_voids", item->className()))
	{
		stp_brep_with_voids* voidsBrep = ROSE_CAST(stp_brep_with_voids, item);
		stp_closed_shell* shell = voidsBrep->outer();
		SetOfstp_face* face = shell->cfs_faces();
		GetSFaceInfo(face);

		if (IsPartitionFace(NatlHalfSpaceList_))
		{
			SFace* partFace = ChoosePartitionFace();
			OcctSplit(NatlHalfSpaceList_, partFace);
		}

		for (auto iter = 0; iter < intersectionFaceList_.size(); iter++)
		{
			StepEntity* step = new StepEntity(intersectionFaceList_[iter]);
			step->GenerateHalfSpaceList();
			step->GenerateCIT();
			step->GenerateHalfCharacteristicPoint();
			step->PMCtest();
			step->Output(&m, &n, iter, shapeName, vecOut_, true, true, true, false);
		}
		vector<vector<SFace*>>().swap(intersectionFaceList_);

		SetOfstp_oriented_closed_shell* orientedShell = voidsBrep->voids();
		for (size_t i = 0; i < orientedShell->size(); i++)
		{
			stp_oriented_closed_shell* oriClosedShell = orientedShell->get(i);
			stp_closed_shell* closeShell = oriClosedShell->closed_shell_element();
			SetOfstp_face* face = closeShell->cfs_faces();

			GetSFaceInfo(face);

			if(IsPartitionFace(NatlHalfSpaceList_))
			{
				SFace* partFace = ChoosePartitionFace();
				OcctSplit(NatlHalfSpaceList_, partFace);
			}
			for (auto iter = 0; iter < intersectionFaceList_.size(); iter++)
			{
				StepEntity* step = new StepEntity(intersectionFaceList_[iter]);
				step->GenerateHalfSpaceList();
				step->GenerateCIT();
				step->GenerateHalfCharacteristicPoint();
				step->PMCtest();
				if (oriClosedShell->orientation())
					step->Output(&m, &n, iter, shapeName, vecOut_, true, true, false, (i == orientedShell->size() - 1));
				else
					step->Output(&m, &n, iter, shapeName, vecOut_, true, false, false, (i == orientedShell->size() - 1));
			}
			mp_ = m;
			vector<vector<SFace*>>().swap(intersectionFaceList_);
		}
	}
}

void Partition::GetSFaceInfo(SetOfstp_face* stpFace)
{
	vector<SFace*>().swap(NatlHalfSpaceList_);
	for(size_t i = 0; i < stpFace->size(); i++)
	{
		stp_face* face = stpFace->get(i);
		stp_advanced_face* adFace = ROSE_CAST(stp_advanced_face, face);
		NatlHalfVector(adFace);
	}
}

bool Partition::IsPartitionFace(vector<SFace*> faceList)
{
	for(auto i = 0; i < faceList.size(); i++)
	{
		for(auto j = i + 1; j < faceList.size(); j++)
			FindPartitionFace(faceList[i], faceList[j]);
	}
	if (partitionFaceList_.size() != 0)
		return true;
	else
	{	
		intersectionFaceList_.push_back(faceList);
		return false;
	}
}

SFace* Partition::ChoosePartitionFace()
{
	//分割面优先级判断
	int num = 0;
	SFace* partFace = new SFace;
	for(auto pIter = partitionFaceList_.begin(); pIter != partitionFaceList_.end(); pIter++)
	{
		auto mulSize = partitionFaceList_.count(pIter->first);
		if(mulSize > num)
		{
			num = mulSize;
			partFace = pIter->second;
		}
		else if(mulSize == num)
		{
			if(!strcmp(pIter->second->name_, "plane") 
				&& strcmp(partFace->name_, "plane") != 0)
				partFace = pIter->second;
			else if(!strcmp(pIter->second->name_, "spherical_surface") 
				&& strcmp(partFace->name_, "plane") != 0 )
				partFace = pIter->second;
			else if(!strcmp(pIter->second->name_, "cylindrical_surface" )
				&& strcmp(partFace->name_, "plane") != 0 
				&& strcmp(partFace->name_, "spherical_surface") != 0
				)
				partFace = pIter->second;
			else if(strcmp(pIter->second->name_, "conical_surface") == 0
				&& strcmp(partFace->name_, "plane") != 0
				&& strcmp(partFace->name_, "spherical_surface") != 0
				&& strcmp(pIter->second->name_, "cylindrical_surface") != 0
				)
				partFace = pIter->second;
		}
	}
	partitionFaceList_.clear();
	return partFace;
}

void Partition::FindPartitionFace(SFace* Fa, SFace* Fb)
{
	//比较两个面是否有相邻curve_edge, 如果有那么就判断两面是否是分割面 否则continue
	for (auto i = Fa->faceBounds_.begin(); i != Fa->faceBounds_.end(); i++)
	{
		for (auto ia = ((*i)->edgeLoop_).begin(); ia != ((*i)->edgeLoop_).end(); ia++)
		{
			size_t curveFaceAId = (*ia)->edgeCurveId_;
			for (auto j = Fb->faceBounds_.begin(); j != Fb->faceBounds_.end(); j++)
			{
				for (auto jb = ((*j)->edgeLoop_).begin(); jb != ((*j)->edgeLoop_).end(); jb++)
				{
					size_t curveFaceBId = (*jb)->edgeCurveId_;
					orientationFaceA oriA,oriB;
					oriA.advancedFaceOri = Fa->adFaceSameSense_;
					oriA.boundsOri = (*i)->boundsOri_;
					oriA.orientedEdgeOri = (*ia)->orientedEdgeOri_;
					oriA.edgeCurveOri = (*ia)->edgeCurvesameSense_;
					oriB.advancedFaceOri = Fb->adFaceSameSense_;
					oriB.boundsOri = (*j)->boundsOri_;
					oriB.orientedEdgeOri = (*jb)->orientedEdgeOri_;
					oriB.edgeCurveOri = (*jb)->edgeCurvesameSense_;
					Standard_CString curveName = (*ia)->curveName_;
					EdgeCurveVertex curveA, curveB;
					curveA.cartesianStart = (*ia)->edgeStart_;
					curveA.cartesianEnd = (*ia)->edgeEnd_;
					curveB.cartesianStart = (*jb)->edgeStart_;
					curveB.cartesianEnd = (*jb)->edgeEnd_;

					CPoint3D pointA = ((CIRCLE*)(*ia))->position_.point;//圆心
					CPoint3D pointB = ((CIRCLE*)(*jb))->position_.point;
					if (curveFaceBId == curveFaceAId)
					{
						//判断是否是分割面
						bool isPartitionFace = JudgeIntersection(Fa, Fb, curveName,
							oriA, oriB, curveA, curveB, pointA, pointB);
						if (isPartitionFace)
						{
							pair<size_t, SFace*> pa(Fa->entityID_, Fa);
							pair<size_t, SFace*> pb(Fb->entityID_, Fb);
							partitionFaceList_.insert(pa);
							partitionFaceList_.insert(pb);
						}
						else
							continue;
					}
				}
			}
		}
	}
}

bool Partition::JudgeIntersection(TopoDS_Face Fa, TopoDS_Face Fb)
{
	CVector3D aDir, bDir;
	TopAbs_Orientation oriFaceA = Fa.Orientation();
	TopAbs_Orientation oriFaceB = Fb.Orientation();
	TopLoc_Location locationA, locationB;
	Handle(Geom_Surface) aGeometricSurface = BRep_Tool::Surface(Fa, locationA);
	Handle(Geom_Surface) bGeometricSurface = BRep_Tool::Surface(Fb, locationB);
	GeomAdaptor_Surface msurfaceA(aGeometricSurface);
	GeomAdaptor_Surface msurfaceB(bGeometricSurface);
	gp_Vec dirA = GetFaceAxis(Fa);
	gp_Vec dirB = GetFaceAxis(Fb);
	if (!oriFaceA)// 根据面的same_sense 判断该面的法线方向 1: 正向  0:反向
		aDir = CVector3D(dirA.X(), dirA.Y(), dirA.Z());
	else
	{
		dirA.Reverse();
		aDir = CVector3D(dirA.X(), dirA.Y(), dirA.Z());
	}
	if (!oriFaceB)
	{
		bDir = CVector3D(dirB.X(), dirB.Y(), dirB.Z());
	}
	else
	{
		dirB.Reverse();
		bDir = CVector3D(dirB.X(), dirB.Y(), dirB.Z());
	}
	if (GeomAbs_Plane == msurfaceA.GetType() && GeomAbs_Plane == msurfaceB.GetType()) //平面平面
	{
		Standard_Real result;
		if (oriA.orientedEdgeOri == oriA.edgeCurveOri)
		{
			CVector3D vect(
				curveA.cartesianEnd.x - curveA.cartesianStart.x,
				curveA.cartesianEnd.y - curveA.cartesianStart.y,
				curveA.cartesianEnd.z - curveA.cartesianStart.z);//交线向量
			result = (aDir*vect) | bDir; //(A法线叉积交线向量EF)点积 B法线
		}
		else
		{
			CVector3D vect(
				curveA.cartesianStart.x - curveA.cartesianEnd.x,
				curveA.cartesianStart.y - curveA.cartesianEnd.y,
				curveA.cartesianStart.z - curveA.cartesianEnd.z);
			result = (aDir*vect) | bDir;
		}
		if (result > 0)
			return true;
		else
			return false;
	}
	else if (GeomAbs_Plane == msurfaceA.GetType() && GeomAbs_Plane != msurfaceB.GetType() )//平面 曲面
	{
// 		Standard_Real result;
// 		CPoint3D P(curveA.cartesianStart.x, curveA.cartesianStart.y, curveA.cartesianStart.z);
// 		if (oriA.orientedEdgeOri == oriA.edgeCurveOri)
// 		{
// 			CVector3D PVec(pointA.x - P.x, pointA.y - P.y, pointA.z - P.z);
// 			if (oriA.advancedFaceOri)
// 			{
// 				CVector3D RVec = PVec*bDir;
// 				result = aDir | (RVec*bDir);
// 			}
// 			else
// 			{
// 				CVector3D RVec = bDir * PVec;
// 				result = aDir | (RVec*bDir);
// 			}
// 			if (result > 0)
// 				return true;
// 			else
// 				return false;
// 		}
// 		else
// 		{
// 			CVector3D PVec(P.x - pointA.x, P.y - pointA.y, P.z - pointA.z);
// 			if (oriA.advancedFaceOri)
// 			{
// 				CVector3D RVec = PVec*bDir;
// 				result = aDir | (RVec*bDir);
// 			}
// 			else
// 			{
// 				CVector3D RVec = bDir * PVec;
// 				result = aDir | (RVec*bDir);
// 			}
// 			if (result > 0)
// 				return true;
// 			else
// 				return false;
// 		}
	}
	else if (GeomAbs_Plane != msurfaceA.GetType() && GeomAbs_Plane == msurfaceB.GetType() )//曲面 平面
	{
// 		Standard_Real result;
// 		CPoint3D P(curveB.cartesianStart.x, curveB.cartesianStart.y, curveB.cartesianStart.z);
// 		if (oriB.orientedEdgeOri == oriB.edgeCurveOri)
// 		{
// 			CVector3D PVec(pointB.x - P.x, pointB.y - P.y, pointB.z - P.z);
// 			if (oriB.advancedFaceOri)
// 			{
// 				CVector3D RVec = PVec*bDir;
// 				result = aDir | (RVec*bDir);
// 			}
// 			else
// 			{
// 				CVector3D RVec = bDir*PVec;
// 				result = aDir | (RVec*bDir);
// 			}
// 			if (result > 0)
// 				return true;
// 			else
// 				return false;
// 		}
// 		else
// 		{
// 			CVector3D PVec(P.x - pointA.x, P.y - pointA.y, P.z - pointA.z);
// 			if (!oriA.advancedFaceOri)
// 			{
// 				CVector3D RVec = PVec*bDir;
// 				result = aDir | (RVec*bDir);
// 			}
// 			else
// 			{
// 				CVector3D RVec = PVec * bDir;
// 				result = aDir | (RVec*bDir);
// 			}
// 			if (result >= 0)
// 				return true;
// 			else
// 				return false;
// 		}
	}
	else if (GeomAbs_Plane != msurfaceA.GetType() && GeomAbs_Plane != msurfaceB.GetType())//曲面 曲面
	{
// 		Standard_Real result;
// 		if (oriA.orientedEdgeOri == oriA.edgeCurveOri)
// 		{
// 			CVector3D FaVector, FbVector;
// 			CVector3D PVec(pointA.x - curveA.cartesianStart.x,
// 				pointA.y - curveA.cartesianStart.y,
// 				pointA.z - curveA.cartesianStart.z);
// 			if (oriA.advancedFaceOri)
// 				FaVector = (PVec * aDir) * aDir;
// 			else
// 				FaVector = aDir * (PVec * aDir);
// 
// 			CPoint3D pB = curveB.cartesianStart;//Fb 相交边对应的start
// 			CVector3D vec(curveB.cartesianStart.x - curveA.cartesianStart.x,
// 				curveB.cartesianStart.y - curveA.cartesianStart.y,
// 				curveB.cartesianStart.z - curveA.cartesianStart.z);
// 			CVector3D RvecB = PVec * vec;
// 
// 			if (oriA.advancedFaceOri)
// 				FbVector = RvecB * vec;
// 			else
// 				FbVector = vec * RvecB;
// 			result = ((PVec * aDir)*FbVector) | FaVector;
// 			if (result > 0)
// 				return  true;
// 			else
// 				return false;
// 		}
// 		else
// 			return false;
 	}
	return true;
}


bool Partition::JudgeIntersection(SFace* Fa, SFace* Fb, Standard_CString curveName, orientationFaceA oriA, orientationFaceB oriB,
	EdgeCurveVertex curveA, EdgeCurveVertex curveB, CPoint3D pointA, CPoint3D pointB)
{
	CVector3D aDir, bDir;
	if(Fa->adFaceSameSense_)// 根据面的same_sense 判断该面的法线方向 1: 正向  0:反向
		aDir = Fa->position_->verAxis;//Fa法线
	else
	{
		CVector3D out(-Fa->position_->verAxis.dx, -Fa->position_->verAxis.dy, -Fa->position_->verAxis.dz );
		aDir = out;
	}
	if(Fb->adFaceSameSense_)
		bDir = Fb->position_->verAxis;//Fb法线
	else
	{
		CVector3D out(-Fb->position_->verAxis.dx, -Fb->position_->verAxis.dy, -Fb->position_->verAxis.dz);
		bDir = out;
	}

	if (!strcmp(Fa->name_, "plane")&&!strcmp(Fb->name_, "plane")) //平面平面
	{
		Standard_Real result;
		if(oriA.orientedEdgeOri == oriA.edgeCurveOri)
		{
			CVector3D vect(curveA.cartesianEnd.x - curveA.cartesianStart.x, 
				curveA.cartesianEnd.y - curveA.cartesianStart.y,
				curveA.cartesianEnd.z - curveA.cartesianStart.z);//交线向量
			result = (aDir*vect) | bDir; //(A法线叉积交线向量EF)点积 B法线
		}
		else
		{
			CVector3D vect(curveA.cartesianStart.x - curveA.cartesianEnd.x,
				curveA.cartesianStart.y - curveA.cartesianEnd.y,
				curveA.cartesianStart.z - curveA.cartesianEnd.z);
			result = (aDir*vect) | bDir;
		}
		if(result > 0)
			return true;
		else
			return false;
	}
	else if( strcmp(Fa->name_, "plane")== 0 && strcmp(Fb->name_, "plane")!=0 )//平面 曲面
	{
		Standard_Real result;
		CPoint3D P(curveA.cartesianStart.x, curveA.cartesianStart.y, curveA.cartesianStart.z);
		if(oriA.orientedEdgeOri == oriA.edgeCurveOri)
		{
			CVector3D PVec(pointA.x - P.x, pointA.y - P.y, pointA.z - P.z);
			if(oriA.advancedFaceOri)
			{
				CVector3D RVec = PVec*bDir;
				result = aDir | (RVec*bDir);
			}
			else
			{
				CVector3D RVec = bDir * PVec;
				result = aDir | (RVec*bDir);
			}
			if(result > 0)
				return true;
			else
				return false;
		}
		else
		{
			CVector3D PVec(P.x - pointA.x, P.y - pointA.y, P.z - pointA.z);
			if(oriA.advancedFaceOri)
			{
				CVector3D RVec = PVec*bDir;
				result = aDir | (RVec*bDir);
			}
			else
			{
				CVector3D RVec = bDir * PVec;
				result = aDir | (RVec*bDir);
			}
			if(result > 0)
				return true;
			else
				return false;
		}
	}
	else if(strcmp(Fa->name_, "plane") != 0 && strcmp(Fb->name_, "plane") == 0)//曲面 平面
	{
		Standard_Real result;
		CPoint3D P(curveB.cartesianStart.x, curveB.cartesianStart.y, curveB.cartesianStart.z);
		if(oriB.orientedEdgeOri == oriB.edgeCurveOri)
		{
			CVector3D PVec(pointB.x - P.x, pointB.y - P.y, pointB.z - P.z);
			if(oriB.advancedFaceOri)
			{
				CVector3D RVec = PVec*bDir;
				result = aDir | (RVec*bDir);
			}
			else
			{
				CVector3D RVec =  bDir*PVec;
				result = aDir | (RVec*bDir);
			}
			if(result > 0)
				return true;
			else
				return false;
		}
		else
		{
			CVector3D PVec(P.x - pointA.x, P.y - pointA.y, P.z - pointA.z);
			if(!oriA.advancedFaceOri)
			{
				CVector3D RVec = PVec*bDir;
				result = aDir | (RVec*bDir);
			}
			else
			{
				CVector3D RVec = PVec * bDir;
				result = aDir | (RVec*bDir);
			}
			if(result >= 0)
				return true;
			else
				return false;
		}
	}
	else if(strcmp(Fa->name_, "plane")!= 0 && strcmp(Fb->name_, "plane") != 0)//曲面 曲面
	{
		Standard_Real result;
		if(oriA.orientedEdgeOri == oriA.edgeCurveOri)
		{
			CVector3D FaVector, FbVector;
			CVector3D PVec(pointA.x - curveA.cartesianStart.x,
				pointA.y - curveA.cartesianStart.y,
				pointA.z - curveA.cartesianStart.z);
			if(oriA.advancedFaceOri)
				FaVector = (PVec * aDir) * aDir;
			else
				FaVector = aDir * (PVec * aDir);

			CPoint3D pB = curveB.cartesianStart;//Fb 相交边对应的start
			CVector3D vec(curveB.cartesianStart.x - curveA.cartesianStart.x,
				curveB.cartesianStart.y - curveA.cartesianStart.y,
				curveB.cartesianStart.z - curveA.cartesianStart.z);
			CVector3D RvecB = PVec * vec;

			if(oriA.advancedFaceOri)
				FbVector = RvecB * vec;
			else
				FbVector = vec * RvecB;
			result = ((PVec * aDir)*FbVector) | FaVector;
			if(result > 0)
				return  true;
			else
				return false;
		}
		else
			return false;
	}
}

void Partition::NatlHalfVector(stp_advanced_face* adFace)
{
	Standard_CString entityName = adFace->face_geometry()->className();

	/*! get bounds to vector<FaceBounds*> faceBounds_ */
	vector<FaceBounds*> faceBounds;
	SetOfstp_face_bound* bounds = adFace->bounds();
	for(size_t i = 0; i < bounds->size(); i++)
	{
		vector<Curve*> curveTemp;
		FaceBounds* faceB = new FaceBounds;

		stp_face_bound* bound = bounds->get(i);
		stp_edge_loop* edgeLoop = ROSE_CAST(stp_edge_loop, bound->bound());
		ListOfstp_oriented_edge* oriList = edgeLoop->edge_list();
		Curve* cur = new Curve;
		for(size_t j = 0; j < oriList->size(); j++)
		{
			GeometryData tempGepmetry;
			stp_oriented_edge* oriEdge = oriList->get(j);
			stp_edge_curve* curve = ROSE_CAST(stp_edge_curve, oriEdge->edge_element());
			stp_curve* pcurve = curve->edge_geometry();//line, circle,ellipse,surface_curve
			
			if(!strcmp(pcurve->className(), "line"))
			{
				LINE* temp = new LINE;
				stp_line* line = ROSE_CAST(stp_line, pcurve);
				CPoint3D point(line->pnt()->coordinates()->get(0),
					line->pnt()->coordinates()->get(1),
					line->pnt()->coordinates()->get(2));
				temp->pnt_ = point/ZOOMTIME;
				CVector3D dir(line->dir()->orientation()->direction_ratios()->get(0),
					line->dir()->orientation()->direction_ratios()->get(1),
					line->dir()->orientation()->direction_ratios()->get(2)
					);
				temp->dir_ = dir;
				temp->magnitude_ = line->dir()->magnitude();
				temp->curveName_ = pcurve->className();
				stp_cartesian_point* eStart = EdgeCurveStartOrEnd(curve->edge_start());
				stp_cartesian_point* eEnd = EdgeCurveStartOrEnd(curve->edge_end());
				CPoint3D start(eStart->coordinates()->get(0), eStart->coordinates()->get(1), eStart->coordinates()->get(2));
				CPoint3D end(eEnd->coordinates()->get(0), eEnd->coordinates()->get(1), eEnd->coordinates()->get(2));
				temp->edgeCurveId_ = pcurve->entity_id();
				temp->edgeStart_ = start/ZOOMTIME;
				temp->edgeEnd_ = end/ZOOMTIME;
				temp->edgeCurvesameSense_ = curve->same_sense();
				temp->orientedEdgeOri_ = oriEdge->orientation();
				curveTemp.push_back(temp);
				 
			}
			if(!strcmp(pcurve->className(), "circle"))
			{
				CIRCLE* temp = new CIRCLE;
				stp_circle* cir = ROSE_CAST(stp_circle, pcurve);
				GetAxisData(cir->position()->_axis2_placement_3d(), tempGepmetry);
				temp->position_ = tempGepmetry;
				temp->radius_ = cir->radius()/ZOOMTIME;

				temp->curveName_ = pcurve->className();
				stp_cartesian_point* eStart = EdgeCurveStartOrEnd(curve->edge_start());
				stp_cartesian_point* eEnd = EdgeCurveStartOrEnd(curve->edge_end());
				CPoint3D start(eStart->coordinates()->get(0), eStart->coordinates()->get(1), eStart->coordinates()->get(2));
				CPoint3D end(eEnd->coordinates()->get(0), eEnd->coordinates()->get(1), eEnd->coordinates()->get(2));
				temp->edgeCurveId_ = pcurve->entity_id();
				temp->edgeStart_ = start / ZOOMTIME;
				temp->edgeEnd_ = end / ZOOMTIME;
				temp->edgeCurvesameSense_ = curve->same_sense();
				temp->orientedEdgeOri_ = oriEdge->orientation();
				curveTemp.push_back(temp);
			}
			if(!strcmp(pcurve->className(), "ellipse"))
			{
				ELLIPSE* temp = new ELLIPSE;
				stp_ellipse* ell = ROSE_CAST(stp_ellipse, pcurve);
				GetAxisData(ell->position()->_axis2_placement_3d(), tempGepmetry);
				temp->position_ = tempGepmetry;
				temp->semi_axis_1_ = ell->semi_axis_1()/ZOOMTIME;
				temp->semi_axis_2_ = ell->semi_axis_2()/ZOOMTIME;
				temp->curveName_ = pcurve->className();
				stp_cartesian_point* eStart = EdgeCurveStartOrEnd(curve->edge_start());
				stp_cartesian_point* eEnd = EdgeCurveStartOrEnd(curve->edge_end());
				CPoint3D start(eStart->coordinates()->get(0), eStart->coordinates()->get(1), eStart->coordinates()->get(2));
				CPoint3D end(eEnd->coordinates()->get(0), eEnd->coordinates()->get(1), eEnd->coordinates()->get(2));
				temp->edgeCurveId_ = pcurve->entity_id();
				temp->edgeStart_ = start / ZOOMTIME;
				temp->edgeEnd_ = end / ZOOMTIME;
				temp->edgeCurvesameSense_ = curve->same_sense();
				temp->orientedEdgeOri_ = oriEdge->orientation();
				curveTemp.push_back(temp);
			}
			if (!strcmp(pcurve->className(), "surface_curve"))
			{
				LINE* temp = new LINE;
				stp_surface_curve* p = ROSE_CAST(stp_surface_curve, pcurve);
				stp_curve * cur = p->curve_3d();
				stp_line* line = ROSE_CAST(stp_line, cur);
				size_t num = line->entity_id();
				CPoint3D point(line->pnt()->coordinates()->get(0),
					line->pnt()->coordinates()->get(1),
					line->pnt()->coordinates()->get(2));
				temp->pnt_ = point;
				CVector3D dir(line->dir()->orientation()->direction_ratios()->get(0),
					line->dir()->orientation()->direction_ratios()->get(1),
					line->dir()->orientation()->direction_ratios()->get(2)
					);
				temp->dir_ = dir;
				temp->magnitude_ = line->dir()->magnitude();
				temp->curveName_ = pcurve->className();
				stp_cartesian_point* eStart = EdgeCurveStartOrEnd(curve->edge_start());
				stp_cartesian_point* eEnd = EdgeCurveStartOrEnd(curve->edge_end());
				CPoint3D start(eStart->coordinates()->get(0), eStart->coordinates()->get(1), eStart->coordinates()->get(2));
				CPoint3D end(eEnd->coordinates()->get(0), eEnd->coordinates()->get(1), eEnd->coordinates()->get(2));
				temp->edgeCurveId_ = pcurve->entity_id();
				temp->edgeStart_ = start / ZOOMTIME;
				temp->edgeEnd_ = end / ZOOMTIME;
				temp->edgeCurvesameSense_ = curve->same_sense();
				temp->orientedEdgeOri_ = oriEdge->orientation();
				curveTemp.push_back(temp);
			}
			
		}
		faceB->edgeLoop_ = curveTemp;
		faceB->boundsOri_ = bound->orientation();
		faceBounds.push_back(faceB);
		
	}

	if(!strcmp(entityName, "plane"))
	{
		SPlane* surface = new SPlane;
		stp_plane* plane = ROSE_CAST(stp_plane, adFace->face_geometry());
		stp_axis2_placement_3d* axis = plane->position();
		GeometryData* data = new GeometryData;
		GetAxisData(axis, *data);
		surface->entityID_ = adFace->entity_id();
		surface->name_ = entityName;
		surface->position_ = data;
		surface->adFaceSameSense_ = adFace->same_sense();
		surface->faceBounds_ = faceBounds;
		NatlHalfSpaceList_.push_back(surface);
	}
	else if(!strcmp(entityName, "spherical_surface"))
	{
		SSpherical* surface = new SSpherical;
		stp_spherical_surface* spherical = ROSE_CAST(stp_spherical_surface, adFace->face_geometry());
		stp_axis2_placement_3d* axis3D = spherical->position();
		GeometryData* data = new GeometryData;
		GetAxisData(axis3D, *data);
		surface->entityID_ = adFace->entity_id();
		surface->name_ = entityName;
		surface->radius_ = spherical->radius() / ZOOMTIME;
		surface->position_ = data;
		surface->adFaceSameSense_ = adFace->same_sense();
		surface->faceBounds_ = faceBounds;
		NatlHalfSpaceList_.push_back(surface);

	}
	else if(!strcmp(entityName, "conical_surface"))
	{
		SConical* surface = new SConical;
		stp_conical_surface* conical = ROSE_CAST(stp_conical_surface, adFace->face_geometry());
		stp_axis2_placement_3d* axis3D = conical->position();
		GeometryData* data = new GeometryData;
		GetAxisData(axis3D, *data);
		surface->entityID_ = adFace->entity_id();
		surface->name_ = entityName;
		surface->radius_ = conical->radius()/ZOOMTIME;
		surface->semi_angle_ = conical->semi_angle() / ZOOMTIME;
		surface->position_ = data;
		surface->adFaceSameSense_ = adFace->same_sense();
		surface->faceBounds_ = faceBounds;
		NatlHalfSpaceList_.push_back(surface);
	}
	else if(!strcmp(entityName, "cylindrical_surface"))
	{
		SCylindrical* surface = new SCylindrical;
		stp_cylindrical_surface* cylindrical = ROSE_CAST(stp_cylindrical_surface, adFace->face_geometry());
		stp_axis2_placement_3d* axis3D = cylindrical->position();
		GeometryData* data = new GeometryData;
		GetAxisData(axis3D, *data);
		surface->entityID_ = adFace->entity_id();
		surface->name_ = entityName;
		surface->radius_ = cylindrical->radius() / ZOOMTIME;
		surface->position_ = data;
		surface->adFaceSameSense_ = adFace->same_sense();
		surface->faceBounds_ = faceBounds;
		NatlHalfSpaceList_.push_back(surface);
	}
	else if(!strcmp(entityName, "toroidal_surface"))
	{
		SToroidal* surface = new SToroidal;
		stp_toroidal_surface* toroidal = ROSE_CAST(stp_toroidal_surface, adFace->face_geometry());
		stp_axis2_placement_3d* axis3D = toroidal->position();
		GeometryData* data = new GeometryData;
		GetAxisData(axis3D, *data);
		surface->entityID_ = adFace->entity_id();
		surface->name_ = entityName;
		surface->major_radius_ = toroidal->major_radius()/ZOOMTIME;
		surface->minor_radius_ = toroidal->minor_radius()/ZOOMTIME;
		surface->position_ = data;
		surface->faceBounds_ = faceBounds;
		surface->adFaceSameSense_ = adFace->same_sense();
		NatlHalfSpaceList_.push_back(surface);
	}
	vector<FaceBounds*>().swap(faceBounds);
}

void Partition::GetAxisData(stp_axis2_placement_3d* axis, GeometryData& data)
{
	CVector3D cV1(axis->axis()->direction_ratios()->get(0), axis->axis()->direction_ratios()->get(1), axis->axis()->direction_ratios()->get(2));
	if(axis->ref_direction())
	{
		CVector3D cV2(axis->ref_direction()->direction_ratios()->get(0), axis->ref_direction()->direction_ratios()->get(1), axis->ref_direction()->direction_ratios()->get(2));
		data.verRefDirection = cV2;
	}
	else
	{
		CVector3D cV2(0.0, 0.0, 0.0);
		data.verRefDirection = cV2;
	}
	CPoint3D cP(axis->location()->coordinates()->get(0), axis->location()->coordinates()->get(1), axis->location()->coordinates()->get(2));
	cP /= ZOOMTIME;
	data.verAxis = cV1;
	data.point = cP;
}

stp_cartesian_point* Partition::EdgeCurveStartOrEnd(stp_vertex* ver)
{
	stp_vertex_point * vpt = ROSE_CAST(stp_vertex_point, ver);
	return ROSE_CAST(stp_cartesian_point, vpt->vertex_geometry());
}

void Partition::OcctSplit(vector<SFace*> faceList, SFace* splitFace)
{
	TopoDS_Face useFace;

//  	TopoDS_Face test;
//  	CurrentStructToOCCT(splitFace, test);
// 		BRepTools::Write(test, "e:\\test.brep");
	
// 	gp_Pnt Pnt(splitFace->position_->point.x, splitFace->position_->point.y, splitFace->position_->point.z);
// 	gp_Dir dir(splitFace->position_->verAxis.dx, splitFace->position_->verAxis.dy, splitFace->position_->verAxis.dz);
// 	gp_Pln plane(Pnt, dir);
// 	useFace = BRepBuilderAPI_MakeFace(plane).Face();
	gp_Pnt Pnt(0, 0, 0);
	gp_Dir dir(0, 1, 0);
	gp_Pln plane(Pnt, dir);
	useFace = BRepBuilderAPI_MakeFace(plane).Face();


	BRepOffsetAPI_Sewing solid; 
	TopoDS_Face face;
	for (auto iter = faceList.begin(); iter != faceList.end(); iter++)
	{
		CurrentStructToOCCT(*iter, face);
		solid.Add(face);
	}
	solid.Perform();
	TopoDS_Shape sewedShape = solid.SewedShape();

	BRepTools::Write(sewedShape, "E:\\test.brep");


	if (sewedShape.IsNull())
	{
		printf("error\n");
	}
	ShapeCutter cutter(sewedShape, useFace);
	cutter.Init(sewedShape, useFace);
	cutter.Perform();
	TopoDS_Shape S1 = cutter.CalcResult1();
	TopoDS_Shape S2 = cutter.CalcResult2();
	BRepTools::Write(S1, "E:\\test1.brep");
	BRepTools::Write(S2, "E:\\test2.brep");
	vector<SFace*> faceList1 = OcctToCurrentStruct(S1);
	vector<SFace*> faceList2 = OcctToCurrentStruct(S2);
	
	canSplitFaceList.push_back(faceList1);
	canSplitFaceList.push_back(faceList2);
 	GetFaceList(faceList, splitFace);
 	while (!canSplitFaceList.empty())
	{
		vector<SFace*> tempList;
		auto iter = canSplitFaceList.begin();
		tempList = *iter;
		canSplitFaceList.erase(iter);
// 		bool isPart = IsPartitionFace(tempList);
// 		if (isPart)
// 		{
// 			SFace* partFace = ChoosePartitionFace();
// 			OcctSplit(tempList, partFace);
// 		}
// 		else
			intersectionFaceList_.push_back(tempList);
	}
}

// void Partition::AddNewSplit(TopoDS_Shape stock, Handle(Geom_Surface)& surface)
// {
// 	TopoDS_Shape resultShape, nullshape;
// 	BRepAlgoAPI_Section asect(stock, surface, Standard_False);
// 
// 	asect.ComputePCurveOn1(Standard_True);
// 	asect.Approximation(Standard_True);
// 	asect.Build();
// 	TopoDS_Shape R = asect.Shape();
// 
// 	if(!asect.ErrorStatus() == 0) 
// 		return;
// 	BRepFeat_SplitShape asplit(stock);
// 	for(TopExp_Explorer Ex(R, TopAbs_EDGE); Ex.More(); Ex.Next())
// 	{
// 		TopoDS_Shape anEdge = Ex.Current();
// 		TopoDS_Shape aFace;
// 		if(asect.HasAncestorFaceOn1(anEdge, aFace))
// 		{
// 			TopoDS_Face F = TopoDS::Face(aFace);
// 			TopoDS_Edge E = TopoDS::Edge(anEdge);
// 			asplit.Add(E, F);
// 		}
// 	}
// 
// 	asplit.Build();
// 
// 	if(!asplit.Shape().IsNull())
// 		resultShape = asplit.Shape();
// 	else
// 		resultShape = nullshape;
// 
// 	//得到两个shape
// 	for(TopExp_Explorer Ex1(resultShape, TopAbs_FACE); Ex1.More(); Ex1.Next())
// 	{
// 		TopoDS_Face aFace = TopoDS::Face(Ex1.Current());
// 		
// 		for(TopExp_Explorer Ex2(Ex1.Current(), TopAbs_VERTEX); Ex2.More(); Ex2.Next())
// 		{
// 			TopoDS_Vertex aVertex = TopoDS::Vertex(Ex2.Current());
// 			gp_Pnt Pnt = BRep_Tool::Pnt(aVertex);
// 			Standard_Real theDistance = Pnt.Distance(BRep_Tool::Pnt(aVertex));
// 			if(theDistance <= 0)
// 			{
// 				TopoDS_Shape shape_1;
// 				
// 			}
// 			else if(theDistance > 0)
// 			{
// 				TopoDS_Shape shape_2;
// 				
// 			}
// 		}
// 
// 	}
// }

void Partition::CurrentStructToOCCT(SFace* face, TopoDS_Face& aFace)
{
	//step一个实体中的advanced_face->OCC中的Shape
	TopoDS_Wire wire;
	Handle(Geom_Surface) surface;

	gp_Sphere SS;
	gp_Cylinder CY;
	if (!strcmp(face->name_, "plane"))
	{
		surface = ((SPlane*)face)->ToOCCT();
	}
	if (!strcmp(face->name_, "cylindrical_surface"))
	{
		surface = ((SCylindrical*)face)->ToOCCT();
		gp_Pnt P1(face->position_->point.x, face->position_->point.y, face->position_->point.z);
		gp_Vec Vec1(face->position_->verAxis.dx, face->position_->verAxis.dy, face->position_->verAxis.dz);
		gp_Dir Dir1(Vec1);
		gp_Ax3 Ax31(P1, Dir1);
		CY = gp_Cylinder(Ax31, ((SCylindrical*)face)->radius_);
	}
	if (!strcmp(face->name_, "spherical_surface"))
	{
		surface = ((SSpherical*)face)->ToOCCT();
		gp_Pnt P1(face->position_->point.x, face->position_->point.y, face->position_->point.z);
		gp_Vec Vec1(face->position_->verAxis.dx, face->position_->verAxis.dy, face->position_->verAxis.dz);
		gp_Dir Dir1(Vec1);
		gp_Ax3 Ax31(P1, Dir1);
		SS = gp_Sphere(Ax31, ((SSpherical*)face)->radius_);
	}
	if (!strcmp(face->name_, "conical_surface"))
	{
		surface = ((SConical*)face)->ToOCCT();
	}
	if (!strcmp(face->name_, "toroidal_surface"))
	{
		surface = ((SToroidal*)face)->ToOCCT();
	}
	for(auto itBound = face->faceBounds_.begin(); itBound != face->faceBounds_.end(); itBound++)
	{
		BRepBuilderAPI_MakeWire MW;
		for(auto itCurve = (*itBound)->edgeLoop_.begin(); itCurve != (*itBound)->edgeLoop_.end(); itCurve++)
		{
			//vertex satrt and vertex end
			gp_Pnt start((*itCurve)->edgeStart_.x, (*itCurve)->edgeStart_.y, (*itCurve)->edgeStart_.z);
			gp_Pnt end((*itCurve)->edgeEnd_.x, (*itCurve)->edgeEnd_.y, (*itCurve)->edgeEnd_.z);
			TopoDS_Vertex vertex1 = BRepBuilderAPI_MakeVertex(start);
			TopoDS_Vertex vertex2 = BRepBuilderAPI_MakeVertex(end);
			if(!stricmp((*itCurve)->curveName_, "line"))
			{
				gp_Dir Dir1(((LINE*)(*itCurve))->dir_.dx, ((LINE*)(*itCurve))->dir_.dy, ((LINE*)(*itCurve))->dir_.dz );
				gp_Pnt Pnt1(((LINE*)(*itCurve))->pnt_.x, ((LINE*)(*itCurve))->pnt_.y, ((LINE*)(*itCurve))->pnt_.z);
				gp_Lin pLine(Pnt1, Dir1);
				TopoDS_Edge edge = BRepBuilderAPI_MakeEdge(pLine, vertex1, vertex2).Edge();
				if (!((*itCurve)->orientedEdgeOri_))
					edge.Reverse();
				MW.Add(edge);
			}
			if(!stricmp((*itCurve)->curveName_, "circle"))
			{
				gp_Dir N1(((CIRCLE*)(*itCurve))->position_.verAxis.dx,
					((CIRCLE*)(*itCurve))->position_.verAxis.dy, 
					((CIRCLE*)(*itCurve))->position_.verAxis.dz );
				gp_Pnt P1(((CIRCLE*)(*itCurve))->position_.point.x, 
					((CIRCLE*)(*itCurve))->position_.point.y, 
					((CIRCLE*)(*itCurve))->position_.point.z );
				gp_Ax2 ax1(P1,N1);
				gp_Circ circle(ax1, ((CIRCLE*)(*itCurve))->radius_);
				TopoDS_Edge edge = BRepBuilderAPI_MakeEdge(circle, vertex1, vertex2).Edge();
				if (!((*itCurve)->orientedEdgeOri_))
					edge.Reverse();
				MW.Add(edge);
			}
			if(!stricmp((*itCurve)->curveName_, "ellipse"))
			{
				gp_Dir N1(((ELLIPSE*)(*itCurve))->position_.verAxis.dx, ((ELLIPSE*)(*itCurve))->position_.verAxis.dy, ((ELLIPSE*)(*itCurve))->position_.verAxis.dz);
				gp_Pnt P1(((ELLIPSE*)(*itCurve))->position_.point.x, ((ELLIPSE*)(*itCurve))->position_.point.y, ((ELLIPSE*)(*itCurve))->position_.point.z);
				gp_Ax2 ax1(P1, N1);
				gp_Elips elips(ax1, ((ELLIPSE*)(*itCurve))->semi_axis_1_, ((ELLIPSE*)(*itCurve))->semi_axis_2_);
				TopoDS_Edge edge = BRepBuilderAPI_MakeEdge(elips, vertex1, vertex2).Edge();
				if (!((*itCurve)->orientedEdgeOri_))
					edge.Reverse();
				MW.Add(edge);
			}

		}
		if (MW.IsDone())
			wire = MW.Wire();
		if (!face->adFaceSameSense_)
			wire.Reverse();
		TopoDS_Wire myWireProfile = TopoDS::Wire(MW);
		aFace = BRepBuilderAPI_MakeFace(surface, myWireProfile, true);
// 		BRepBuilderAPI_MakeFace MF(SS);
// 		MF.Add(myWireProfile);
// 		MF.Build();
// 		if (MF.IsDone())
// 			aFace = MF.Face();
		BRepTools::Write(aFace, "D:\\test.brep");
	}
}



SFace* Partition::CloneFace(SFace* face)
{
	SFace* newFace = new SFace;
	Standard_CString entityName = face->name_;
	vector<FaceBounds*> faceBounds;
	FaceBounds* faceB = new FaceBounds;
	RoseBoolean ori;
	for (auto i = face->faceBounds_.begin(); i != face->faceBounds_.end(); i++)
	{
		vector<Curve*> curveTemp;
		ori = (*i)->boundsOri_;
		for (auto j = (*i)->edgeLoop_.begin(); j != (*i)->edgeLoop_.end(); j++)
		{
			if (!strcmp((*j)->curveName_, "line"))
			{
				LINE* temp = new LINE;
				CPoint3D point(((LINE*)(*j))->pnt_);
				temp->dir_ = ((LINE*)(*j))->dir_;
				temp->magnitude_ = ((LINE*)(*j))->magnitude_;
				temp->curveName_ = ((LINE*)(*j))->curveName_;
				CPoint3D start(((LINE*)(*j))->edgeStart_);
				CPoint3D end(((LINE*)(*j))->edgeEnd_);
				temp->edgeCurveId_ = ((LINE*)(*j))->edgeCurveId_;
				temp->edgeStart_ = start;
				temp->edgeEnd_ = end;
				temp->edgeCurvesameSense_ = ((LINE*)(*j))->edgeCurvesameSense_;
				temp->orientedEdgeOri_ = ((*j)->orientedEdgeOri_);
				curveTemp.push_back(temp);
			}
			if (!strcmp((*j)->curveName_, "circle"))
			{
				CIRCLE* temp = new CIRCLE;
				temp->position_ = ((CIRCLE*)(*j))->position_;
				temp->radius_ = ((CIRCLE*)(*j))->radius_;
				temp->curveName_ = ((CIRCLE*)(*j))->curveName_;
				CPoint3D start(((CIRCLE*)(*j))->edgeStart_);
				CPoint3D end(((CIRCLE*)(*j))->edgeEnd_);
				temp->edgeCurveId_ = ((CIRCLE*)(*j))->edgeCurveId_;
				temp->edgeStart_ = start;
				temp->edgeEnd_ = end;
				temp->edgeCurvesameSense_ = ((CIRCLE*)(*j))->edgeCurvesameSense_;
				temp->orientedEdgeOri_ = (*j)->orientedEdgeOri_;
				curveTemp.push_back(temp);
			}
		}
		faceB->edgeLoop_ = curveTemp;
		faceB->boundsOri_ = ori;
		faceBounds.push_back(faceB);
	}

	if (!strcmp(entityName, "plane"))
	{
		SPlane* surface = new SPlane;
		surface->entityID_ = face->entityID_;
		surface->name_ = entityName;
		surface->position_ = CloneEntity(face->position_);
		surface->adFaceSameSense_ = face->adFaceSameSense_;
		surface->faceBounds_ = faceBounds;
		newFace = surface;
	}
	if (!strcmp(entityName, "cylindrical_surface"))
	{
		SCylindrical* surface = new SCylindrical;
		surface->entityID_ = face->entityID_;
		surface->name_ = entityName;
		surface->radius_ = ((SCylindrical*)face)->radius_;
		surface->position_ = CloneEntity(face->position_);
		surface->adFaceSameSense_ = face->adFaceSameSense_;
		surface->faceBounds_ = faceBounds;
		newFace = surface;
	}
	return newFace;
}

GeometryData* Partition::CloneEntity(GeometryData* geo)
{
	GeometryData* geo1 = new GeometryData;
	CPoint3D po1 = geo->point;
	CPoint3D po2 = geo->pointReserve;
	CVector3D ver1 = geo->verAxis;
	CVector3D ver2 = geo->verRefDirection;
	geo1->point = po1;
	geo1->pointReserve = po2;
	geo1->verAxis = ver1;
	geo1->verRefDirection = ver2;
	return geo1;
}

void Partition::GetFaceList(vector<SFace*> faceList, SFace* splitFace)
{
	vector<SFace*> faceList1, faceList2;
	if (faceList.size()==16)
	{
		faceList1.push_back(faceList.at(3));
		faceList1.push_back(faceList.at(4));
		faceList1.push_back(faceList.at(9));
		faceList1.push_back(faceList.at(11));
		faceList1.push_back(faceList.at(10));
		faceList1.push_back(faceList.at(13));
		SFace* face1 = new SFace;
		face1 = CloneFace(faceList.at(9));
		face1->position_->point.z += 10.0;
		face1->position_->verAxis.dy += 1.0;
		face1->position_->verAxis.dx += 1.0;
		faceList1.push_back(face1);
		faceList1.push_back(splitFace);
		intersectionFaceList_.push_back(faceList1);
		faceList2.push_back(faceList.at(0));
		faceList2.push_back(faceList.at(1));
		faceList2.push_back(faceList.at(2));
		faceList2.push_back(faceList.at(5));
		faceList2.push_back(faceList.at(6));
		faceList2.push_back(faceList.at(7));
		faceList2.push_back(faceList.at(8));
		faceList2.push_back(faceList.at(12));
		faceList2.push_back(faceList.at(13));
		faceList2.push_back(faceList.at(10));
		faceList2.push_back(faceList.at(14));
		faceList2.push_back(faceList.at(15));
		intersectionFaceList_.push_back(faceList2);
	}
	
	else if (faceList.size() == 7)
	{
		faceList1.push_back(faceList.at(0));
		faceList1.push_back(faceList.at(4));
		faceList1.push_back(faceList.at(6));
		SFace* face1 = new SFace;
		face1 = CloneFace(faceList.at(6));
		face1->position_->point.z += 10.0;
		face1->position_->verAxis.dy += 1.0;
		faceList1.push_back(face1);
//		intersectionFaceList_.push_back(faceList1);
		faceList2.push_back(faceList.at(1));
		faceList2.push_back(faceList.at(2));
		faceList2.push_back(faceList.at(3));
		SFace* face2 = new SFace;
		face2 = CloneFace(faceList.at(3));
		face2->position_->point.z -= 10.0;
		face2->position_->verAxis.dx += 1.0;
		face2->position_->verAxis.dy -= 1.0;
		faceList2.push_back(face2);
		intersectionFaceList_.push_back(faceList2);
		intersectionFaceList_.push_back(faceList2);
	}
	else
		intersectionFaceList_.push_back(faceList);
}
	
vector<SFace*> Partition::OcctToCurrentStruct(TopoDS_Shape aShape)
{
	vector<SFace*> faceList;
	TopoDS_Face aFace;
	TopoDS_Wire aWire;
	TopoDS_Edge aEdge;
	TopoDS_Vertex aVertex;
	TopExp_Explorer Exp_Edge, Exp_Wire, Exp_Face, Exp_Vertex;
	for (Exp_Face.Init(aShape, TopAbs_FACE); Exp_Face.More(); Exp_Face.Next())
	{
		SFace* face = new SFace;
		aFace = TopoDS::Face(Exp_Face.Current());
		TopAbs_Orientation orient = aFace.Orientation();
		TopLoc_Location location;
		Handle(Geom_Surface) aGeometricSurface = BRep_Tool::Surface(aFace, location);
		if (aGeometricSurface.IsNull())
			continue;
		if (aGeometricSurface->IsKind(STANDARD_TYPE(Geom_Plane)))
		{
			Handle(Geom_Plane) aPlane = Handle(Geom_Plane)::DownCast(aGeometricSurface);
			face->name_ = "plane";
		}

		if (aGeometricSurface->IsKind(STANDARD_TYPE(Geom_CylindricalSurface)))
		{
			Handle(Geom_CylindricalSurface) aCylindrical = Handle(Geom_CylindricalSurface)::DownCast(aGeometricSurface);
			face->name_ = "cylindrical_surface";
		}
		if (aGeometricSurface->IsKind(STANDARD_TYPE(Geom_SphericalSurface)))
		{
			Handle(Geom_SphericalSurface) aOffSurf = Handle(Geom_SphericalSurface)::DownCast(aGeometricSurface);
			face->name_ = "spherical_surface";
		}

		if (aGeometricSurface->IsKind(STANDARD_TYPE(Geom_ConicalSurface)))
		{
			Handle(Geom_ConicalSurface) aOffSurf = Handle(Geom_ConicalSurface)::DownCast(aGeometricSurface);
			face->name_ = "conical_surface";
		}
		for (Exp_Wire.Init(aFace, TopAbs_WIRE); Exp_Wire.More(); Exp_Wire.Next())
		{
			aWire = TopoDS::Wire(Exp_Wire.Current());
			for (Exp_Edge.Init(aWire, TopAbs_EDGE); Exp_Edge.More(); Exp_Edge.Next())
			{
				aEdge = TopoDS::Edge(Exp_Edge.Current());
				vector<CPoint3D> pointL;
				for (Exp_Vertex.Init(aEdge, TopAbs_VERTEX); Exp_Vertex.More(); Exp_Vertex.Next())
				{
					aVertex = TopoDS::Vertex(Exp_Vertex.Current());
					gp_Pnt Pnt = BRep_Tool::Pnt(aVertex);
					CPoint3D point(Pnt.X(), Pnt.Y(), Pnt.Z());
					pointL.push_back(point);
				}
			}
		}
	}

	// 	for(Exp_Vertex.Init(aShape, TopAbs_VERTEX); Exp_Vertex.More(); Exp_Vertex.Next())
	// 	{
	// 		aVertex = TopoDS::Vertex(Exp_Vertex.Current());
	// 		gp_Pnt Pnt = BRep_Tool::Pnt(aVertex);
	// 	}
	// 	for(Exp_Edge.Init(aShape, TopAbs_EDGE); Exp_Edge.More(); Exp_Edge.Next())
	// 	{
	// 		aEdge = TopoDS::Edge(Exp_Edge.Current());
	// 		Standard_Real First, Last;
	// 		Handle(Geom_Curve) Pnt = BRep_Tool::Curve(aEdge, First, Last);
	// 		Pnt.Value(First);
	// 	}
	// 	for(Exp_Face.Init(aShape, TopAbs_WIRE); Exp_Face.More(); Exp_Face.Next())
	// 	{
	// 		aFace = TopoDS::Face(Exp_Face.Current());
	// 		Handle(Geom_Surface) aSurface = BRep_Tool::Surface(aFace);
	// 		
	// 	}
	return faceList;
}
