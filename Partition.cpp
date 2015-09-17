#include "stdafx.h"
#include "Partition.h"
#include <algorithm>



Partition::Partition(RoseDesign* design)
	:design_(design)
{

}


Partition::~Partition()
{

}

void Partition::StepConversionAndOutput()
{
	RoseCursor objects;
	RoseObject* obj;
	objects.traverse(design_);
	objects.domain(ROSE_DOMAIN(stp_advanced_brep_shape_representation));
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
				/*
					StepEntity class 
				*/
			}
			if(!strcmp("brep_with_voids", it->className()))
			{
				stp_brep_with_voids* voidsBrep = ROSE_CAST(stp_brep_with_voids, it);
				stp_closed_shell* shell = voidsBrep->outer();
				SetOfstp_face* face = shell->cfs_faces();
				/* 
					StepEntity class
				*/

 				SetOfstp_oriented_closed_shell* orientedShell = voidsBrep->voids();
				for(size_t i = 0; i < orientedShell->size(); i++)
				{
					stp_oriented_closed_shell* oriClosedShell = orientedShell->get(i);
					stp_closed_shell* closeShell = oriClosedShell->closed_shell_element();
					SetOfstp_face* face = closeShell->cfs_faces();
					/*
						StepEntity class
					*/
				}
			}
		}
	}
}

void Partition::PartitionFace(stp_closed_shell* closeShell)
{
	SetOfstp_face* stpFace = closeShell->cfs_faces();
	for(size_t i = 0; i < stpFace->size(); i++)
	{
		stp_face* face = stpFace->get(i);
		stp_advanced_face* adFace = ROSE_CAST(stp_advanced_face, face);
		NatlHalfVector(adFace);// 保存每个面结构的信息 faceInfors_;
	}
	IsPartitionFace();
}

void Partition::IsPartitionFace()
{
	for(auto iter = 0; iter < NatlHalfSpaceList_.size(); iter++)
	{
		FindPartitionFace(NatlHalfSpaceList_[iter], NatlHalfSpaceList_[iter + 1]);
	}
	//分割面优先级判断

	OcctSplit();
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
	else if(!strcmp(Fa->name_, "plane") && strcmp(Fb->name_, "plane") )//平面 曲面
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
	else if(strcmp(Fa->name_, "plane") && strcmp(Fb->name_, "plane"))//曲面 曲面
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
					CPoint3D pointA = ((Circle*)(*ia))->position_.point;//圆心
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
				((Line*)cur)->pnt_ = point;
				CVector3D dir(line->dir()->orientation()->direction_ratios()->get(0),
					line->dir()->orientation()->direction_ratios()->get(1),
					line->dir()->orientation()->direction_ratios()->get(2)
					);
				((Line*)cur)->dir_ = dir;
				((Line*)cur)->magnitude_ = line->dir()->magnitude();
				 
			}
			if(!strcmp(pcurve->className(), "circle"))
			{
				stp_circle* cir = ROSE_CAST(stp_circle, pcurve);
				GetAxisData(cir->position()->_axis2_placement_3d(), tempGepmetry);
				((Circle*)cur)->position_ = tempGepmetry;
				((Circle*)cur)->radius_ = cir->radius();
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
// 	mkCut.Build(
	partitionFaceList_.clear();

	return faceTemp;
}