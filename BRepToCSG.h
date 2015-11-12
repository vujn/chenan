#pragma once
#include "stdafx.h"
#include <set>


using namespace std;

#define VALUE_M 

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

	Repetition repe_;//Matrix struct
// 	typedef std::vector<Repetition> repetitionStructure;
// 	repetitionStructure repetition_;
	
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
	void CompareString(string str1, string str2);
	double CompareNum(double matrix);
	RoseDesign* roseDesign_;
	StpAsmProductDefVec roots_;
	StixMtrx stixMtrx_;
	vector<string> outFile_;
	vector<string> TR_;
	int intex_;
	int m_p;
	string nameShape_;
	int productId_;
	set<int> checkRepetitionStructure_;
	map<int, int> repeNum_;
	int count_;
	template < class T>
	string ConvertToString(T value)
	{
		stringstream ss;
		ss << value;
		return ss.str();
	}

public:
	int mNum_;
	int nNum_;
};

