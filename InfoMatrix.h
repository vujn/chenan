#pragma once

#include <stp_schema.h>
#include <stix.h>
#include <stixmesh.h>
#include <stix_xform.h>
#include <iostream>
#include <vector>
#include <list>
#include <RoseString.h>
#include <map>

using namespace std;


struct Repetition 
{
	size_t entityId;
	StixMtrx stixMtrx;
};

class InfoMatrix
{

public:

	InfoMatrix(const char * stepFileName);

	virtual ~InfoMatrix(void);

	void MatrixMess(size_t entityId, StixMtrx &stixMtrx);

	typedef std::vector<Repetition> repetitionStructure;
	repetitionStructure repetition_;

private:
	
	void GetProductInformation( stp_product_definition* proDefinition, 
		StixMtrx& sMtrx );

	void GetShapeInformation( stp_representation* rep, 
		StixMtrx& stixMtrx, 
		RoseObject* rep_rel_or_mapped_item,
		stp_product_definition* proDefinition,
		size_t nestDepth );

	RoseDesign* roseDesign_;
	StpAsmProductDefVec roots_;
	StixMtrx stixMtrx_;
	const char* stepFileName_;
};

