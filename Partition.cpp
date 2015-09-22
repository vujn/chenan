#include "stdafx.h"
#include "Partition.h"
#include <algorithm>



Partition::Partition(RoseDesign* design)
	:design_(design)
{
	StepConversionAndOutput();
}


Partition::~Partition()
{
	vector<string>().swap(vecOut_);
}

void Partition::StepConversionAndOutput()
{
	RoseCursor objects;
	RoseObject* obj;
	objects.traverse(design_);
	objects.domain(ROSE_DOMAIN(stp_advanced_brep_shape_representation));
	int m = 1;
	int n = 1;
	while(obj = objects.next())
	{
		stp_advanced_brep_shape_representation* pt = ROSE_CAST(stp_advanced_brep_shape_representation, obj);
		SetOfstp_representation_item* items = pt->items();
		stp_representation_context* item = pt->context_of_items();
		for(size_t i = 0; i < items->size(); i++)
		{
			stp_representation_item* it = items->get(i);
			if(!strcmp("axis2_placement_3d", it->className()))
				stp_axis2_placement_3d *axis = ROSE_CAST(stp_axis2_placement_3d, it);
			if(!strcmp("manifold_solid_brep", it->className()))
			{
				stp_manifold_solid_brep* solidBrep = ROSE_CAST(stp_manifold_solid_brep, it);
				stp_closed_shell* shell = solidBrep->outer();
				SetOfstp_face* face = shell->cfs_faces();
				GetSFaceInfo(face);

				StepEntity* step = new StepEntity(NatlHalfSpaceList_);
				step->GenerateHalfSpaceList();
				step->GenerateCIT();
				step->GenerateHalfCharacteristicPoint();
				step->PMCtest();
				step->Output(&m, &n, "", vecOut_, false, false, false, false);
			}
			if(!strcmp("brep_with_voids", it->className()))
			{
				stp_brep_with_voids* voidsBrep = ROSE_CAST(stp_brep_with_voids, it);
				stp_closed_shell* shell = voidsBrep->outer();
				SetOfstp_face* face = shell->cfs_faces();
				GetSFaceInfo(face);

				StepEntity* step = new StepEntity(NatlHalfSpaceList_);
				step->GenerateHalfSpaceList();
				step->GenerateCIT();
				step->GenerateHalfCharacteristicPoint();
				step->PMCtest();
				step->Output(&m, &n, "", vecOut_, false, false, false, false);

 				SetOfstp_oriented_closed_shell* orientedShell = voidsBrep->voids();
				for(size_t i = 0; i < orientedShell->size(); i++)
				{
					stp_oriented_closed_shell* oriClosedShell = orientedShell->get(i);
					stp_closed_shell* closeShell = oriClosedShell->closed_shell_element();
					SetOfstp_face* face = closeShell->cfs_faces();
					GetSFaceInfo(face);

					StepEntity* step = new StepEntity(NatlHalfSpaceList_);
					step->GenerateHalfSpaceList();
					step->GenerateCIT();
					step->GenerateHalfCharacteristicPoint();
					step->PMCtest();
					step->Output(&m, &n, "", vecOut_, false, false, false, false);
				}
			}
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

void Partition::PartitionFace()
{
	for(auto iter = 0; iter < NatlHalfSpaceList_.size(); iter++)
	{
		FindPartitionFace(NatlHalfSpaceList_[iter], NatlHalfSpaceList_[iter + 1]);
	}
	SFace* partFace = ChoosePartitionFace();
	if (partFace != nullptr)
		OcctSplit();
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
	return partFace;
}


void Partition::CurrentStructToOCCT(TopoDS_Shape& aShape, SFace* face)
{

}

bool Partition::JudgeIntersection(SFace* Fa, SFace* Fb, char* curveName, orientationFaceA oriA,
	EdgeCurveVertex curveA, EdgeCurveVertex curveB, CPoint3D pointA)
{
	CVector3D aDir = Fa->position_->verAxis;//Fa法线
	CVector3D bDir = Fb->position_->verAxis;

	if (!strcmp(Fa->name_, "plane")&&!strcmp(Fb->name_, "plane")) //平面平面
	{
		double result;
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
		double result;
		CPoint3D P(curveA.cartesianStart.x, curveA.cartesianStart.y, curveA.cartesianStart.z);
		if(oriA.orientedEdgeOri == oriA.edgeCurveOri)
		{
			CVector3D PVec(pointA.x - P.x, pointA.y - P.y, pointA.z - P.z);
			if(oriA.advancedFaceOri == 'T')
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
			if(oriA.advancedFaceOri == 'T')
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
	else if(strcmp(Fa->name_, "plane")!= 0 && strcmp(Fb->name_, "plane") != 0)//曲面 曲面
	{
		double result;
		if(oriA.orientedEdgeOri == oriA.edgeCurveOri)
		{
			CVector3D FaVector, FbVector;
			CVector3D PVec(pointA.x - curveA.cartesianStart.x, 
				pointA.y - curveA.cartesianStart.y, 
				pointA.z - curveA.cartesianStart.z);
			if (oriA.advancedFaceOri == 'T')
				FaVector = (PVec * aDir) * aDir;
			else
				FaVector = aDir * (PVec * aDir);

			CPoint3D pB = curveB.cartesianStart;//Fb 相交边对应的start
			CVector3D vec(curveB.cartesianStart.x - curveA.cartesianStart.x,
				curveB.cartesianStart.y - curveA.cartesianStart.y,
				curveB.cartesianStart.z - curveA.cartesianStart.z);
			CVector3D RvecB = PVec * vec;

			if (oriA.advancedFaceOri == 'T')
				FbVector = RvecB * vec;
			else
				FbVector = vec * RvecB;
			result = ((PVec * aDir)*FbVector) | FaVector;
			if (result > 0)
				return  true;
			else
				return false;
		}
	}
}

void Partition::FindPartitionFace(SFace* Fa, SFace* Fb)
{
	//比较两个面是否有相邻curve_edge, 如果有那么就判断两面是否是分割面 否则continue
	for(auto i = Fa->faceBounds_.begin(); i != Fa->faceBounds_.end(); i++)
	{
		for(auto ia = ((*i)->edgeLoop_).begin(); ia != ((*i)->edgeLoop_).end(); ia++)
		{
			size_t curveFaceAId = (*ia)->edgeCurveId_;
			for(auto j = Fb->faceBounds_.begin(); j != Fb->faceBounds_.end(); j++)
			{
				for(auto jb = ((*j)->edgeLoop_).begin(); jb != ((*j)->edgeLoop_).end(); jb++)
				{
					size_t curveFaceBId = (*jb)->edgeCurveId_;
					orientationFaceA oriA;
					oriA.advancedFaceOri = Fa->adFaceSameSense_;
					oriA.boundsOri = (*i)->boundsOri_;
					oriA.orientedEdgeOri = (*ia)->orientedEdgeOri_;
					oriA.edgeCurveOri = (*ia)->edgeCurvesameSense_;

					char* curveName = (*ia)->curveName_;
					EdgeCurveVertex curveA, curveB;
					curveA.cartesianStart = (*ia)->edgeStart_;
					curveA.cartesianEnd = (*ia)->edgeEnd_;
					curveA.cartesianStart = (*jb)->edgeStart_;
					curveA.cartesianEnd = (*jb)->edgeEnd_;
					CPoint3D pointA = ((CIRCLE*)(*ia))->position_.point;//圆心
					if(curveFaceBId == curveFaceAId)
					{
						//判断是否是分割面
						bool isPartitionFace = JudgeIntersection(Fa, Fb, curveName,
							oriA, curveA, curveB, pointA);
						if(isPartitionFace)
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


void Partition::NatlHalfVector(stp_advanced_face* adFace)
{
	char* entityName = adFace->face_geometry()->className();

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

		for(size_t j = 0; j < oriList->size(); j++)
		{
			GeometryData tempGepmetry;
			Curve* cur = new Curve;
			stp_oriented_edge* oriEdge = oriList->get(j);
			stp_edge_curve* curve = ROSE_CAST(stp_edge_curve, oriEdge->edge_element());
			stp_curve* pcurve = curve->edge_geometry();//line, circle,ellipse,surface_curve
			if(!strcmp(pcurve->className(), "line"))
			{
				stp_line* line = ROSE_CAST(stp_line, pcurve);
				CPoint3D point(line->pnt()->coordinates()->get(0),
					line->pnt()->coordinates()->get(1),
					line->pnt()->coordinates()->get(2));
				((LINE*)cur)->pnt_ = point;
				CVector3D dir(line->dir()->orientation()->direction_ratios()->get(0),
					line->dir()->orientation()->direction_ratios()->get(1),
					line->dir()->orientation()->direction_ratios()->get(2)
					);
				((LINE*)cur)->dir_ = dir;
				((LINE*)cur)->magnitude_ = line->dir()->magnitude();
				 
			}
			if(!strcmp(pcurve->className(), "circle"))
			{
				stp_circle* cir = ROSE_CAST(stp_circle, pcurve);
				GetAxisData(cir->position()->_axis2_placement_3d(), tempGepmetry);
				((CIRCLE*)cur)->position_ = tempGepmetry;
				((CIRCLE*)cur)->radius_ = cir->radius();
			}
			if(!strcmp(pcurve->className(), "ellipse"))
			{
				stp_ellipse* ell = ROSE_CAST(stp_ellipse, pcurve);
				GetAxisData(ell->position()->_axis2_placement_3d(), tempGepmetry);
				((ELLIPSE*)cur)->position_ = tempGepmetry;
				((ELLIPSE*)cur)->semi_axis_1_ = ell->semi_axis_1();
				((ELLIPSE*)cur)->semi_axis_2_ = ell->semi_axis_2();
			}
			cur->curveName_ = pcurve->className();
			stp_cartesian_point* eStart = EdgeCurveStartOrEnd(curve->edge_start());
			stp_cartesian_point* eEnd = EdgeCurveStartOrEnd(curve->edge_end());
			CPoint3D start(eStart->coordinates()->get(0), eStart->coordinates()->get(1), eStart->coordinates()->get(2));
			CPoint3D end(eEnd->coordinates()->get(0), eEnd->coordinates()->get(1), eEnd->coordinates()->get(2));
			cur->edgeStart_ = start;
			cur->edgeEnd_ = end;
			cur->edgeCurvesameSense_ = curve->same_sense();
			cur->orientedEdgeOri_ = oriEdge->orientation();
			curveTemp.push_back(cur);
		}
		faceB->edgeLoop_ = curveTemp;
		faceB->boundsOri_ = bound->orientation();
		faceBounds.push_back(faceB);
		vector<Curve*>().swap(curveTemp);
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
		surface->radius_ = conical->radius();
		surface->semi_angle_ = conical->semi_angle() / ZOOMTIME;
		surface->position_ = data;
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
		surface->major_radius_ = toroidal->major_radius();
		surface->minor_radius_ = toroidal->minor_radius();
		surface->position_ = data;
		surface->faceBounds_ = faceBounds;
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

vector<SFace*> Partition::OcctSplit()
{
	vector<SFace*> faceTemp;

//	BRepAlgoAPI_Section(const TopoDS_Shape& Sh,
//	const Handle(Geom_Surface)& Sf, const Standard_Boolean PerformNow = Standard_True);

// 	BRepAlgoAPI_Section(TopoDS_Shape Sh, gp_Pln Pl, bool PerformNow);
// 	gp_Pln aplane = new gp_Pln(1, 0.25, 3, 4);
// 	Geom_Plane thePlane = new Geom_Plane(aplane);
// 	BRepAlgoAPI_Section section = new BRepAlgoAPI_Section(theTorus, thePlane, false);
// 	section.ComputePCurveOn1(true);
// 	section.Approximation(true);
// 	section.Build();
// 	AIS_Shape asection = new AIS_Shape(section.Shape());

// 	BRepAlgoAPI_Section mkCut(_cTopoShape, cutting_plane, Standard_False);
// 	mkCut.ComputePCurveOn1(Standard_True);
// 	mkCut.Approximation(Standard_True);
// 	mkCut.Build();
	partitionFaceList_.clear();

	return faceTemp;
}

void Partition::AddNewSplit(TopoDS_Shape stock, Handle(Geom_Surface)& surface)
{
	TopoDS_Shape resultShape, nullshape;
	BRepAlgoAPI_Section asect(stock, surface, Standard_False);

	asect.ComputePCurveOn1(Standard_True);
	asect.Approximation(Standard_True);
	asect.Build();
	TopoDS_Shape R = asect.Shape();

	if(!asect.ErrorStatus() == 0) 
		return;
	BRepFeat_SplitShape asplit(stock);
	for(TopExp_Explorer Ex(R, TopAbs_EDGE); Ex.More(); Ex.Next())
	{
		TopoDS_Shape anEdge = Ex.Current();
		TopoDS_Shape aFace;
		if(asect.HasAncestorFaceOn1(anEdge, aFace))
		{
			TopoDS_Face F = TopoDS::Face(aFace);
			TopoDS_Edge E = TopoDS::Edge(anEdge);
			asplit.Add(E, F);
		}
	}

	asplit.Build();

	if(!asplit.Shape().IsNull())
		resultShape = asplit.Shape();
	else
		resultShape = nullshape;

	//得到两个shape
	for(TopExp_Explorer Ex1(resultShape, TopAbs_FACE); Ex1.More(); Ex1.Next())
	{
		TopoDS_Face aFace = TopoDS::Face(Ex1.Current());
		
		for(TopExp_Explorer Ex2(Ex1.Current(), TopAbs_VERTEX); Ex2.More(); Ex2.Next())
		{
			TopoDS_Vertex aVertex = TopoDS::Vertex(Ex2.Current());
			gp_Pnt Pnt = BRep_Tool::Pnt(aVertex);
			Standard_Real theDistance = Pnt.Distance(BRep_Tool::Pnt(aVertex));
			if(theDistance <= 0)
			{
				TopoDS_Shape shape_1;
				
			}
			else if(theDistance > 0)
			{
				TopoDS_Shape shape_2;
				
			}
		}

	}
}

//
void Partition::OcctToCurrentStruct(TopoDS_Shape& aShape)
{
	TopoDS_Face aFace;
	TopoDS_Wire aWire;
	TopoDS_Edge aEdge;
	TopoDS_Vertex aVertex;
	TopExp_Explorer Exp_Edge, Exp_Wire, Exp_Face, Exp_Vertex;
	for (Exp_Face.Init(aShape, TopAbs_FACE); Exp_Face.More(); Exp_Face.Next())
	{
		aFace = TopoDS::Face(Exp_Face.Current());
		for (Exp_Wire.Init(aFace, TopAbs_WIRE); Exp_Wire.More(); Exp_Wire.Next())
		{
			aWire = TopoDS::Wire(Exp_Wire.Current());
			for (Exp_Edge.Init(aWire, TopAbs_EDGE); Exp_Edge.More(); Exp_Edge.Next())
			{
				aEdge = TopoDS::Edge(Exp_Edge.Current());
				for (Exp_Vertex.Init(aEdge, TopAbs_VERTEX); Exp_Vertex.More(); Exp_Vertex.Next())
				{
					aVertex = TopoDS::Vertex(Exp_Vertex.Current());
					gp_Pnt Pnt = BRep_Tool::Pnt(aVertex);
				}
			}
		}
	}
	 
	for(Exp_Vertex.Init(aShape, TopAbs_VERTEX); Exp_Vertex.More(); Exp_Vertex.Next())
	{
		aVertex = TopoDS::Vertex(Exp_Vertex.Current());
		gp_Pnt Pnt = BRep_Tool::Pnt(aVertex);
	}
	for(Exp_Edge.Init(aShape, TopAbs_EDGE); Exp_Edge.More(); Exp_Edge.Next())
	{
		aEdge = TopoDS::Edge(Exp_Edge.Current());
		Standard_Real First, Last;
		Handle(Geom_Curve) Pnt = BRep_Tool::Curve(aEdge, First, Last);

	}
	for(Exp_Wire.Init(aShape, TopAbs_WIRE); Exp_Wire.More(); Exp_Wire.Next())
	{
		aWire = TopoDS::Wire(Exp_Wire.Current());
	}
	for(Exp_Face.Init(aShape, TopAbs_WIRE); Exp_Face.More(); Exp_Face.Next())
	{

		aFace = TopoDS::Face(Exp_Face.Current());
		Handle(Geom_Surface) aSurface = BRep_Tool::Surface(aFace);
		
	}
}
