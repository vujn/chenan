#pragma once
#include "stdafx.h"
#include <set>


using namespace std;


struct Repetition
{
	size_t entityId;
	StixMtrx stixMtrx;
};

class BRepToCSG
{

public:

	BRepToCSG(RoseDesign* roseDesign);

	virtual ~BRepToCSG(void);

	void MatrixMess(size_t entityId, StixMtrx &stixMtrx);

	typedef std::vector<Repetition> repetitionStructure;
	repetitionStructure repetition_;
	
private:

	void GetProductInformation(stp_product_definition* proDefinition,
		StixMtrx& sMtrx);

	void GetShapeInformation(stp_representation* rep,
		StixMtrx& stixMtrx,
		RoseObject* rep_rel_or_mapped_item,
		stp_product_definition* proDefinition,
		size_t nestDepth);

	void replace_all(std::string & s, std::string const & t, std::string const & w);
	void SplitString(const char* str, const char* c, vector<string>& vecSplit);

	RoseDesign* roseDesign_;
	StpAsmProductDefVec roots_;
	StixMtrx stixMtrx_;
	vector<string> outFile_;
	int m_p;

	template < class T>
	string ConvertToString(T value)
	{
		stringstream ss;
		ss << value;
		return ss.str();
	}

	
};

