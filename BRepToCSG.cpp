#include "stdafx.h"
#include "BRepToCSG.h"
#include "Partition.h"

static int NUM = 1;

BRepToCSG::BRepToCSG(RoseDesign* roseDesign)
:roseDesign_(roseDesign), intex_(1)
{
	mNum_ = 1;
	nNum_ = 1;
	stix_find_root_products( &roots_, roseDesign_ );
	for(size_t i = 0, sz = roots_.size(); i < sz; i++)
	{
		StixMtrx rootPlacement;
		GetProductInformation(roots_[i], rootPlacement);
	}

	//////////////////////////////////////////////////////////////////////////////////////
	//输出到文件中
	//////////////////////////////////////////////////////////////////////////////////////
	ofstream result;
	result.open("result.txt");
	string voidExpression = "";
	voidExpression += ConvertToString(m_p) + " 0 ";
	string str2;
	string str1;
	for (int i = 1; i < m_p; i++)
	{
		voidExpression += "#" + ConvertToString(i);
		if (i != m_p - 1)
			voidExpression += " ";
		else
			voidExpression += " $ NULL \n";
	}
	str1 += voidExpression;
	str2 += " \n";
	outFile_.push_back(str1);
	outFile_.push_back(str2);
	int iSize = outFile_.size() / 2;
	for (int i = 0; i < iSize; i++)
	{
		std::string& str = outFile_[2 * i];
		replace_all(str, "\n", "");
		string outfile;
		vector<string> vecSplitOut;
		SplitString(str.c_str(), " ", vecSplitOut);
		int ivecSize = vecSplitOut.size();
		int iCount = ivecSize / 15;
		int imod = ivecSize % 15;
		for (int n = 0; n < iCount; n++)
		{
			for (int k = 0; k < 15; k++)
			{
				outfile.append(vecSplitOut[n * 15 + k]);
				outfile.append(" ");
			}

			if ((0 == imod) && (n == iCount - 1))
				outfile += "\n";
			else
				outfile += "&\n     ";
		}

		int iStart = iCount * 15;
		for (int s = iStart; s < ivecSize; s++)
		{
			outfile.append(vecSplitOut[s]);
			outfile.append(" ");
		}

		if (0 != imod)
			outfile += "\n";
		result << outfile;
	}
	result << endl;

	for (int i = 0; i < iSize; i++)
	{
		result << outFile_[2 * i + 1];
	}
// 	for (int i = 0; i < TR_.size(); i++)
// 	{
// 		result << TR_[i];
// 	}
// 	set<string>::iterator it;
// 	for (it = TR_.begin(); it != TR_.end(); it++)
// 		result << *it;
	result.close();
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
		const char * pname = p ? p->name() : 0;
		nameShape_ = pname;
		productId_ = p->entity_id();
		MatrixMess(p->entity_id(), stixMtrx);
		
	}
	SetOfstp_representation_item* items = rep->items();
	Partition part(roseDesign_);
	int i, sz;

	for (i = 0, sz = items->size(); i < sz; i++)
	{
		stp_representation_item* item = items->get(i);
		if (!strcmp("axis2_placement_3d", item->className()))
			continue;
		else
		{
			part.StepConversionAndOutput(item, nameShape_, mNum_, nNum_);
			m_p = part.mp_;
			outFile_.insert(outFile_.end(), part.vecOut_.begin(), part.vecOut_.end());
			vector<string>().swap(part.vecOut_);

			pair<set<int>::iterator, bool> iter = checkRepetitionStructure_.insert(productId_);
			if (!iter.second)
			{
				string str;
				auto isTrue = repeNum_.insert(pair<int, int>(productId_, NUM));
				if (isTrue.second)
					NUM++;
				auto resultNum = repeNum_.find(productId_);
				int id = repe_.entityId;
				str = "TR"
					+ ConvertToString(intex_) + " "
					+ ConvertToString(CompareNum(repe_.stixMtrx.get(0, 3) / ZOOMTIME)) + " "
					+ ConvertToString(CompareNum(repe_.stixMtrx.get(1, 3) / ZOOMTIME)) + " "
					+ ConvertToString(CompareNum(repe_.stixMtrx.get(2, 3) / ZOOMTIME)) + " "
					+ ConvertToString(CompareNum(repe_.stixMtrx.get(0, 0))) + " "
					+ ConvertToString(CompareNum(repe_.stixMtrx.get(1, 0))) + " "
					+ ConvertToString(CompareNum(repe_.stixMtrx.get(2, 0))) + " "
					+ ConvertToString(CompareNum(repe_.stixMtrx.get(0, 1))) + " "
					+ ConvertToString(CompareNum(repe_.stixMtrx.get(1, 1))) + " "
					+ ConvertToString(CompareNum(repe_.stixMtrx.get(2, 1))) + " "
					+ ConvertToString(CompareNum(repe_.stixMtrx.get(0, 2))) + " "
					+ ConvertToString(CompareNum(repe_.stixMtrx.get(1, 2))) + " "
					+ ConvertToString(CompareNum(repe_.stixMtrx.get(2, 2))) + " " + "1";
				str += "\n";
				intex_++;
				TR_.push_back(str);
			}
		}
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


void BRepToCSG::replace_all(std::string & s, std::string const & t, std::string const & w)
{
	string::size_type pos = s.find(t), t_size = t.size(), r_size = w.size();
	while (pos != std::string::npos)
	{ // found   
		s.replace(pos, t_size, w);
		pos = s.find(t, pos + r_size);
	}
}


void BRepToCSG::SplitString(const char* str, const char* c, vector<string>& vecSplit)
{
	char *p = strtok(const_cast<char*>(str), c);
	while (p)
	{
		vecSplit.push_back(p);
		p = strtok(NULL, c);
	}
}

double BRepToCSG::CompareNum(double matrix)
{
	if (IS_ZERO(matrix))
		return 0.0;
	else
		return matrix;
}

void BRepToCSG::MatrixMess(size_t entityId, StixMtrx& stixMtrx)
{
	// 平移信息  
	//			stixMtrx.get(0,3);stixMtrx.get(1,3);stixMtrx.get(2,3); 

	// 旋转信息
	//			x轴::stixMtrx.get(0,0), stixMtrx.get(1,0), stixMtrx.get(2,0))
	//			y轴::stixMtrx.get(0,1), stixMtrx.get(1,1), stixMtrx.get(2,1))
	//			z轴::stixMtrx.get(0,2), stixMtrx.get(1,2), stixMtrx.get(2,2))

	// 重复结构
	//			Repetition;
	printf("Loc: (%.6g %.6g %.6g)\n", stixMtrx.get(0, 3), stixMtrx.get(1, 3), stixMtrx.get(2, 3));//平移信息
	printf("xdir (%.3f %.3f %.3f)\n", stixMtrx.get(0, 0), stixMtrx.get(1, 0), stixMtrx.get(2, 0));//新的X轴
	printf("ydir (%.3f %.3f %.3f)\n", stixMtrx.get(0, 1), stixMtrx.get(1, 1), stixMtrx.get(2, 1));//新的Y轴
	printf("zdir (%.3f %.3f %.3f)\n", stixMtrx.get(0, 2), stixMtrx.get(1, 2), stixMtrx.get(2, 2));//新的Z轴
	repe_.entityId = entityId;
	
	repe_.stixMtrx = stixMtrx;
}
