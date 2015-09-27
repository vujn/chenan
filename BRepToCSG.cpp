#include "stdafx.h"
#include "BRepToCSG.h"
#include "Partition.h"

BRepToCSG::BRepToCSG(RoseDesign* roseDesign)
	:roseDesign_(roseDesign)
{
	stix_find_root_products( &roots_, roseDesign_ );
	for(size_t i = 0, sz = roots_.size(); i < sz; i++)
	{
		StixMtrx rootPlacement;
		GetProductInformation(roots_[i], rootPlacement);
	}
}

BRepToCSG::~BRepToCSG(void)
{
	//vector<Repetition>().swap(repetition_);
}

void BRepToCSG::GetProductInformation(stp_product_definition* proDefinition,
	StixMtrx& sMtrx )
{
	if (!proDefinition) 
		return;
	StixMgrAsmProduct* pm = StixMgrAsmProduct::find(proDefinition);

	printf("ROOT PRODUCT #%lu\n", proDefinition->entity_id());
	
	for(size_t i = 0; i < pm->shapes.size(); i++)
	{
		stp_shape_representation * rep = pm->shapes[i];
		GetShapeInformation(rep, sMtrx, 0, 0, 0);
	}
}

void BRepToCSG::GetShapeInformation(stp_representation* rep,
	StixMtrx& stixMtrx,
	RoseObject* rep_rel_or_mapped_item, 
	stp_product_definition* proDefinition, 
	size_t nestDepth )
{
	if ( !rep ) 
		return;
	
	if ( proDefinition ) 
	{
		stp_product_definition_formation* pdf = proDefinition-> formation();
		stp_product* p = pdf? pdf-> of_product(): 0;
		MatrixMess(rep->entity_id(), stixMtrx);
	}

	SetOfstp_representation_item* items = rep->items();
	Partition*	part = new Partition(roseDesign_);
	int i, sz;
	for (i = 0, sz = items->size(); i < sz; i++)
	{
		stp_representation_item* item = items->get(i);
		part->StepConversionAndOutput(item);
	}

	StixMgrAsmShapeRep* rep_mgr = StixMgrAsmShapeRep::find(rep);
	if (!rep_mgr) 
		return;

	for(size_t i = 0; i < rep_mgr->child_rels.size(); i++)
	{
		stp_shape_representation_relationship* rel = rep_mgr->child_rels[i];
		stp_representation* child = stix_get_shape_usage_child_rep (rel);
		stp_product_definition* cpd = stix_get_shape_usage_child_product (rel);

		StixMtrx child_xform = stix_get_shape_usage_xform (rel);
		child_xform = child_xform* stixMtrx;
		GetShapeInformation(child, child_xform, rel, cpd, nestDepth);
	}

	for(size_t i = 0;  i < rep_mgr->child_mapped_items.size(); i++)
	{
		stp_mapped_item* rel = rep_mgr->child_mapped_items[i];
		stp_representation* child = stix_get_shape_usage_child_rep (rel);
		stp_product_definition* cpd = stix_get_shape_usage_child_product (rel);

		StixMtrx child_xform = stix_get_shape_usage_xform (rel);
		child_xform = child_xform * stixMtrx;
		GetShapeInformation(child, child_xform, rel, cpd, nestDepth);
	}
}

void BRepToCSG::MatrixMess(size_t entityId, StixMtrx& stixMtrx)
{
	// ƽ����Ϣ  
	//			stixMtrx.get(0,3);stixMtrx.get(1,3);stixMtrx.get(2,3); 

	// ��ת��Ϣ
	//			x��::stixMtrx.get(0,0), stixMtrx.get(1,0), stixMtrx.get(2,0))
	//			y��::stixMtrx.get(0,1), stixMtrx.get(1,1), stixMtrx.get(2,1))
	//			z��::stixMtrx.get(0,2), stixMtrx.get(1,2), stixMtrx.get(2,2))

	// �ظ��ṹ
	//			Repetition;
	Repetition repe;
	repe.entityId = entityId;
	repe.stixMtrx = stixMtrx;
	repetition_.push_back(repe);
}

