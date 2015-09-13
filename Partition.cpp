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

	//比较两个面是否有相交的边 如果有->判断是否是分割面; 否则继续下两个面的判断
	SFace* Fa = new SFace;
	SFace* Fb = new SFace;
	JudgeIntersection(Fa, Fb);
	
	if (IsPartitionFace())
	{
		OcctSplit();
	}
	else
	{

	}
}

bool Partition::IsPartitionFace()
{
	vector<stp_edge_curve*> edgeCurveList;

	TraverseEdgeCurve(edgeCurveList);

	return false;
}

vector<SFace*> Partition::OcctSplit()
{
	vector<SFace*> faceTemp;
//	BRepAlgoAPI_Section

	return faceTemp;
}

void Partition::JudgeIntersection(SFace* Fa, SFace* Fb)
{
	if (strcmp(Fa->name_, "plane") && strcmp(Fb->name_, "plane"))
	{
		for (auto i = Fa->faceBounds_.begin(); i != Fa->faceBounds_.end(); i++)
		{
			for (auto j = Fb->faceBounds_.begin(); j != Fb->faceBounds_.end(); j++)
			{
				((*i)->edgeLoop_)
			}
		}
		//for_each(Fa->faceBounds_.begin(), Fa->faceBounds_.end(), myfunction);
	}
	else if (strcmp(Fa->name_, "plane") && strcmp(Fb->name_, ""))//Fa为平面,Fb为曲面 
	{

	}
	else if (strcmp(Fa->name_, "") && strcmp(Fb->name_, "plane"))//Fa为曲面,Fb为平面
	{

	}
	else if (strcmp(Fa->name_, "") && strcmp(Fb->name_, ""))//Fa为曲面,Fb为曲面,相交为圆锥曲线的情况
	{

	}
}

void Partition::TraverseEdgeCurve(vector<stp_edge_curve*>& edgeList)
{

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
			cur->edgeStart_ = EdgeCurveStartOrEnd(curve->edge_start());
			cur->edgeEnd_ = EdgeCurveStartOrEnd(curve->edge_start());
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