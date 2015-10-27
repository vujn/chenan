#include "stdafx.h"
#include "StepEntity.h"
#include "GeomCalc.h"
#include "curve.h"

static int testNum = 0;
set<string> setString_;
StepEntity::StepEntity(vector<SFace*> natlHalfSpaceList)
	:NatlHalfSpaceList_(natlHalfSpaceList)
{

}

StepEntity::~StepEntity(void)
{
	
}

void StepEntity::GenerateCIT()
{
	CIT_ = (HalfSpaceOrientation**)malloc(pow(((double)2.0), ((int)CSGHalfSpaceList_.size()))*sizeof(HalfSpaceOrientation*)); //第一维 
	for (int i = 0; i < pow(((double)2.0), ((int)CSGHalfSpaceList_.size())); i++)
		CIT_[i] = (HalfSpaceOrientation*)malloc(CSGHalfSpaceList_.size()* sizeof(HalfSpaceOrientation));//第二维

	CITIndex_ = (TermIndex*)malloc(pow(((double)2.0), ((int)CSGHalfSpaceList_.size()))* sizeof(TermIndex));//第一维

	for (int i = 0; i < pow(((double)2.0), ((int)CSGHalfSpaceList_.size())); i++)
	{
		for (int j = 0; j < CSGHalfSpaceList_.size(); j++)
		{
			if (i&((int)pow(2.0, j)))
				CIT_[i][CSGHalfSpaceList_.size() - j - 1] = POSITIVE;
			else
				CIT_[i][CSGHalfSpaceList_.size() - j - 1] = NEGATIVE;
		}

		CITIndex_[i].tListIndex = -1;
		CITIndex_[i].isNotNull = false;
		CITIndex_[i].tPosition = Undetermined;
	}
}

void StepEntity::GenerateHalfCharacteristicPoint()
{
	vector<SFace*>().swap(HalfSpaceList_);
	HalfSpaceList_.insert(HalfSpaceList_.end(), ForwardHalfSpaceList_.begin(), ForwardHalfSpaceList_.end());
	HalfSpaceList_.insert(HalfSpaceList_.end(), BackwardHalfSpaceList_.begin(), BackwardHalfSpaceList_.end());
	vector<CPoint3D>().swap(CITCPList_);
	CVector3D VZero(0, 0, 0);
	SPlane * assistPlane = new SPlane;//辅助平面系数数组
	assistPlane->name_ = "plane";
	for(size_t i = 0; i < HalfSpaceList_.size(); i++)//需要添加辅助平面而生成的特征点
	{
		for(size_t j = i + 1; j < HalfSpaceList_.size(); j++)
		{
			if (j == i + HalfSpaceList_.size() / 2)
				continue;
			if(!strcmp(HalfSpaceList_[i]->name_, "plane") && !strcmp(HalfSpaceList_[j]->name_, "plane"))
				continue;
			if(!strcmp(HalfSpaceList_[i]->name_, "spherical_surface") && !strcmp(HalfSpaceList_[j]->name_, "spherical_surface"))
				//两个球面，生成一个过两球心的平面，作为生成特征点的辅助平面
			{
				CVector3D pVector3D1(
					((SSpherical*)HalfSpaceList_[i])->position_->point.x - ((SSpherical*)HalfSpaceList_[j])->position_->point.x,
					((SSpherical*)HalfSpaceList_[i])->position_->point.y - ((SSpherical*)HalfSpaceList_[j])->position_->point.y,
					((SSpherical*)HalfSpaceList_[i])->position_->point.z - ((SSpherical*)HalfSpaceList_[j])->position_->point.z);
				pVector3D1.Normalize();
				if (VZero == pVector3D1)
					continue;
				CPoint3D pPoint3D1 = ((SSpherical*)HalfSpaceList_[i])->position_->point;
				pVector3D1.GenerateTFace(assistPlane->coefficient_, pPoint3D1, 1);
				SetSurfaceIntersectPoints(HalfSpaceList_[i], HalfSpaceList_[j], assistPlane);
			}
			if (!strcmp(HalfSpaceList_[i]->name_, "cylindrical_surface") && !strcmp(HalfSpaceList_[j]->name_, "cylindrical_surface"))
				//两个柱面，分别生成过每一个柱面轴线的平面，作为生成特征点的辅助平面
			{
				CVector3D pVector3D1 = ((SCylindrical*)HalfSpaceList_[i])->position_->verAxis;
				pVector3D1.Normalize();
				CPoint3D pPoint3D1 = ((SCylindrical*)HalfSpaceList_[i])->position_->point;
				pVector3D1.GenerateTFace(assistPlane->coefficient_, pPoint3D1, 1);
				SetSurfaceIntersectPoints(HalfSpaceList_[i], HalfSpaceList_[j], assistPlane);

				CVector3D pVector3D2 = ((SCylindrical*)HalfSpaceList_[j])->position_->verAxis;
				pVector3D2.Normalize();
				CPoint3D pPoint3D2 = ((SCylindrical*)HalfSpaceList_[j])->position_->point;
				pVector3D2.GenerateTFace(assistPlane->coefficient_, pPoint3D2, 1);
				SetSurfaceIntersectPoints(HalfSpaceList_[i], HalfSpaceList_[j], assistPlane);

			}
			if (!strcmp(HalfSpaceList_[i]->name_, "conical_surface") && !strcmp(HalfSpaceList_[j]->name_, "conical_surface"))
				//两个锥面，分别生成过每一个锥面轴线的平面，作为生成特征点的辅助平面
			{
				CVector3D pVector3D1 = ((SConical*)HalfSpaceList_[i])->position_->verAxis;
				pVector3D1.Normalize();
				CPoint3D pPoint3D1 = ((SConical*)HalfSpaceList_[i])->position_->point;
				pVector3D1.GenerateTFace(assistPlane->coefficient_, pPoint3D1, 1);
				SetSurfaceIntersectPoints(HalfSpaceList_[i], HalfSpaceList_[j], assistPlane);

				CVector3D pVector3D2 = ((SConical*)HalfSpaceList_[j])->position_->verAxis;
				pVector3D2.Normalize();
				CPoint3D pPoint3D2 = ((SConical*)HalfSpaceList_[j])->position_->point;
				pVector3D2.GenerateTFace(assistPlane->coefficient_, pPoint3D2, 1);
				SetSurfaceIntersectPoints(HalfSpaceList_[i], HalfSpaceList_[j], assistPlane);
			}

			if (!strcmp(HalfSpaceList_[i]->name_, "spherical_surface") && !strcmp(HalfSpaceList_[j]->name_, "plane"))
				//一个球面一个平面，生成过球心且与平面垂直的平面，作为生成特征点的辅助平面
			{
				CVector3D pVector3D1 = ((SPlane*)HalfSpaceList_[j])->position_->verAxis;
				pVector3D1.Normalize();
				CPoint3D pPoint3D1 = ((SSpherical*)HalfSpaceList_[i])->position_->point;
				pVector3D1.GenerateTFace(assistPlane->coefficient_, pPoint3D1, 1);
				SetSurfaceIntersectPoints(HalfSpaceList_[i], HalfSpaceList_[j], assistPlane);

			}
			if (!strcmp(HalfSpaceList_[i]->name_, "plane") && !strcmp(HalfSpaceList_[j]->name_, "spherical_surface"))
				//一个球面一个平面，生成过球心且与平面垂直的平面，作为生成特征点的辅助平面
			{
				CVector3D pVector3D1 = ((SPlane*)HalfSpaceList_[i])->position_->verAxis;
				pVector3D1.Normalize();
				CPoint3D pPoint3D1 = ((SSpherical*)HalfSpaceList_[j])->position_->point;
				pVector3D1.GenerateTFace(assistPlane->coefficient_, pPoint3D1, 1);
				SetSurfaceIntersectPoints(HalfSpaceList_[i], HalfSpaceList_[j], assistPlane);
			}

			if (!strcmp(HalfSpaceList_[i]->name_, "cylindrical_surface") && !strcmp(HalfSpaceList_[j]->name_, "plane"))
				//一个柱面一个平面，如果平面与轴线平行，生成垂直于轴线的平面，否则，生成过轴线的平面，作为生成特征点的辅助平面
			{
				CVector3D pVector3D1 = ((SCylindrical*)HalfSpaceList_[i])->position_->verAxis;
				pVector3D1.Normalize();
				CVector3D pVector3D2 = ((SPlane*)HalfSpaceList_[j])->position_->verAxis;
				pVector3D2.Normalize();
				CPoint3D pPoint3D1 = ((SCylindrical*)HalfSpaceList_[i])->position_->point;

				if (IS_ZERO(pVector3D1 | pVector3D2))
				{
					pVector3D1.GenerateVFace(assistPlane->coefficient_, pPoint3D1);
					SetSurfaceIntersectPoints(HalfSpaceList_[i], HalfSpaceList_[j], assistPlane);
				}
				else
				{
					pVector3D1.GenerateTFace(assistPlane->coefficient_, pPoint3D1, 1);
					SetSurfaceIntersectPoints(HalfSpaceList_[i], HalfSpaceList_[j], assistPlane);
				}
			}
			if (!strcmp(HalfSpaceList_[i]->name_, "plane") && !strcmp(HalfSpaceList_[j]->name_, "cylindrical_surface"))
				//一个柱面一个平面，如果平面与轴线平行，生成垂直于轴线的平面，否则，生成过轴线的平面，作为生成特征点的辅助平面
			{
				CVector3D pVector3D1 = ((SCylindrical*)HalfSpaceList_[j])->position_->verAxis;
				pVector3D1.Normalize();
				CVector3D pVector3D2 = ((SPlane*)HalfSpaceList_[i])->position_->verAxis;
				pVector3D2.Normalize();
				CPoint3D pPoint3D1 = ((SCylindrical*)HalfSpaceList_[j])->position_->point;
				if (IS_ZERO(pVector3D1 | pVector3D2))
				{
					pVector3D1.GenerateVFace(assistPlane->coefficient_, pPoint3D1);
					SetSurfaceIntersectPoints(HalfSpaceList_[i], HalfSpaceList_[j], assistPlane);
				}
				else
				{
					pVector3D1.GenerateTFace(assistPlane->coefficient_, pPoint3D1, 1);
					SetSurfaceIntersectPoints(HalfSpaceList_[i], HalfSpaceList_[j], assistPlane);
				}
			}

			if (!strcmp(HalfSpaceList_[i]->name_, "conical_surface") && !strcmp(HalfSpaceList_[j]->name_, "plane"))
				//一个锥面一个平面，如果平面与轴线平行，生成垂直于轴线的平面，否则，生成过轴线的平面，作为生成特征点的辅助平面
			{
				CVector3D pVector3D1 = ((SConical*)HalfSpaceList_[i])->position_->verAxis;
				pVector3D1.Normalize();
				CVector3D pVector3D2 = ((SPlane*)HalfSpaceList_[j])->position_->verAxis;
				pVector3D2.Normalize();
				CPoint3D pPoint3D1 = ((SConical*)HalfSpaceList_[i])->position_->point;
				if (IS_ZERO(pVector3D1 | pVector3D2))
				{
					pVector3D1.GenerateVFace(assistPlane->coefficient_, pPoint3D1);
					SetSurfaceIntersectPoints(HalfSpaceList_[i], HalfSpaceList_[j], assistPlane);
				}
				else
				{
					pVector3D1.GenerateTFace(assistPlane->coefficient_, pPoint3D1, 1);
					SetSurfaceIntersectPoints(HalfSpaceList_[i], HalfSpaceList_[j], assistPlane);
				}
			}
			if (!strcmp(HalfSpaceList_[i]->name_, "plane") && !strcmp(HalfSpaceList_[j]->name_, "conical_surface"))
				//一个锥面一个平面，如果平面与轴线平行，生成垂直于轴线的平面，否则，生成过轴线的平面，作为生成特征点的辅助平面
			{
				CVector3D pVector3D1 = ((SConical*)HalfSpaceList_[j])->position_->verAxis;
				pVector3D1.Normalize();
				CVector3D pVector3D2 = ((SPlane*)HalfSpaceList_[i])->position_->verAxis;
				pVector3D2.Normalize();
				CPoint3D pPoint3D1 = ((SConical*)HalfSpaceList_[j])->position_->point;

				if (IS_ZERO(pVector3D1 | pVector3D2))
				{
					pVector3D1.GenerateVFace(assistPlane->coefficient_, pPoint3D1);
					SetSurfaceIntersectPoints(HalfSpaceList_[i], HalfSpaceList_[j], assistPlane);
				}
				else
				{
					pVector3D1.GenerateTFace(assistPlane->coefficient_, pPoint3D1, 1);
					SetSurfaceIntersectPoints(HalfSpaceList_[i], HalfSpaceList_[j], assistPlane);
				}
			}

			if (!strcmp(HalfSpaceList_[i]->name_, "spherical_surface") && !strcmp(HalfSpaceList_[j]->name_, "cylindrical_surface"))
				//一个球面和一个柱面，生成过柱面轴线的平面和过圆心垂直于柱面轴线的平面，作为生成特征点的辅助平面
			{
				CVector3D pVector3D1 = ((SCylindrical*)HalfSpaceList_[j])->position_->verAxis;
				pVector3D1.Normalize();
				CPoint3D pPoint3D1 = ((SCylindrical*)HalfSpaceList_[j])->position_->point;
				pVector3D1.GenerateTFace(assistPlane->coefficient_, pPoint3D1, 1);
				SetSurfaceIntersectPoints(HalfSpaceList_[i], HalfSpaceList_[j], assistPlane);
				CPoint3D pPoint3D2 = ((SSpherical*)HalfSpaceList_[i])->position_->point;
				pVector3D1.GenerateVFace(assistPlane->coefficient_, pPoint3D2);
				SetSurfaceIntersectPoints(HalfSpaceList_[i], HalfSpaceList_[j], assistPlane);
			}

			if (!strcmp(HalfSpaceList_[i]->name_, "cylindrical_surface") && !strcmp(HalfSpaceList_[j]->name_, "spherical_surface"))
				//一个球面和一个柱面，生成过柱面轴线的平面和过圆心垂直于柱面轴线的平面，作为生成特征点的辅助平面
			{
				CVector3D pVector3D1 = ((SCylindrical*)HalfSpaceList_[i])->position_->verAxis;
				pVector3D1.Normalize();
				CPoint3D pPoint3D1 = ((SCylindrical*)HalfSpaceList_[i])->position_->point;
				pVector3D1.GenerateTFace(assistPlane->coefficient_, pPoint3D1, 1);
				SetSurfaceIntersectPoints(HalfSpaceList_[i], HalfSpaceList_[j], assistPlane);
				CPoint3D pPoint3D2 = ((SSpherical*)HalfSpaceList_[j])->position_->point;
				pVector3D1.GenerateVFace(assistPlane->coefficient_, pPoint3D2);
				SetSurfaceIntersectPoints(HalfSpaceList_[i], HalfSpaceList_[j], assistPlane);
			}

			if (!strcmp(HalfSpaceList_[i]->name_, "spherical_surface") && !strcmp(HalfSpaceList_[j]->name_, "conical_surface"))
				//一个球面和一个锥面，生成过锥面轴线的平面和过圆心垂直于锥面轴线的平面，作为生成特征点的辅助平面
			{
				CVector3D pVector3D1 = ((SConical*)HalfSpaceList_[j])->position_->verAxis;
				pVector3D1.Normalize();
				CPoint3D pPoint3D1 = ((SConical*)HalfSpaceList_[j])->position_->point;
				pVector3D1.GenerateTFace(assistPlane->coefficient_, pPoint3D1, 1);
				SetSurfaceIntersectPoints(HalfSpaceList_[i], HalfSpaceList_[j], assistPlane);
				CPoint3D pPoint3D2 = ((SSpherical*)HalfSpaceList_[i])->position_->point;
				pVector3D1.GenerateVFace(assistPlane->coefficient_, pPoint3D2);
				SetSurfaceIntersectPoints(HalfSpaceList_[i], HalfSpaceList_[j], assistPlane);

			}
			if (!strcmp(HalfSpaceList_[i]->name_, "conical_surface") && !strcmp(HalfSpaceList_[j]->name_, "spherical_surface"))
				//一个球面和一个锥面，生成过锥面轴线的平面和过圆心垂直于锥面轴线的平面，作为生成特征点的辅助平面
			{
				CVector3D pVector3D1 = ((SConical*)HalfSpaceList_[i])->position_->verAxis;
				pVector3D1.Normalize();
				CPoint3D pPoint3D1 = ((SConical*)HalfSpaceList_[i])->position_->point;
				pVector3D1.GenerateTFace(assistPlane->coefficient_, pPoint3D1, 1);
				SetSurfaceIntersectPoints(HalfSpaceList_[i], HalfSpaceList_[j], assistPlane);
				CPoint3D pPoint3D2 = ((SSpherical*)HalfSpaceList_[j])->position_->point;
				pVector3D1.GenerateVFace(assistPlane->coefficient_, pPoint3D2);
				SetSurfaceIntersectPoints(HalfSpaceList_[i], HalfSpaceList_[j], assistPlane);
			}

			if (!strcmp(HalfSpaceList_[i]->name_, "cylindrical_surface") && !strcmp(HalfSpaceList_[j]->name_, "conical_surface"))
				//一个柱面和一个锥面，生成过锥面轴线的平面和过柱面轴线的平面，作为生成特征点的辅助平面
			{
				CVector3D pVector3D1 = ((SCylindrical*)HalfSpaceList_[i])->position_->verAxis;
				pVector3D1.Normalize();
				CPoint3D pPoint3D1 = ((SCylindrical*)HalfSpaceList_[i])->position_->point;
				pVector3D1.GenerateTFace(assistPlane->coefficient_, pPoint3D1, 1);
				SetSurfaceIntersectPoints(HalfSpaceList_[i], HalfSpaceList_[j], assistPlane);

				CVector3D pVector3D2 = ((SConical*)HalfSpaceList_[j])->position_->verAxis;
				pVector3D2.Normalize();
				CPoint3D pPoint3D2 = ((SConical*)HalfSpaceList_[j])->position_->point;
				pVector3D2.GenerateTFace(assistPlane->coefficient_, pPoint3D2, 1);
				SetSurfaceIntersectPoints(HalfSpaceList_[i], HalfSpaceList_[j], assistPlane);
			}

			if (!strcmp(HalfSpaceList_[i]->name_, "conical_surface") && !strcmp(HalfSpaceList_[j]->name_, "cylindrical_surface"))
				//一个柱面和一个锥面，生成过锥面轴线的平面和过柱面轴线的平面，作为生成特征点的辅助平面
			{
				CVector3D pVector3D1 = ((SConical*)HalfSpaceList_[i])->position_->verAxis;
				pVector3D1.Normalize();
				CPoint3D pPoint3D1 = ((SConical*)HalfSpaceList_[i])->position_->point;
				pVector3D1.GenerateTFace(assistPlane->coefficient_, pPoint3D1, 1);
				SetSurfaceIntersectPoints(HalfSpaceList_[i], HalfSpaceList_[j], assistPlane);

				CVector3D pVector3D2 = ((SCylindrical*)HalfSpaceList_[j])->position_->verAxis;
				pVector3D2.Normalize();
				CPoint3D pPoint3D2 = ((SCylindrical*)HalfSpaceList_[j])->position_->point;
				pVector3D2.GenerateTFace(assistPlane->coefficient_, pPoint3D2, 1);
				SetSurfaceIntersectPoints(HalfSpaceList_[i], HalfSpaceList_[j], assistPlane);
			}
			CITCPList_.insert(CITCPList_.end(), MP_.interPointList_.begin(), MP_.interPointList_.end());
		}
	}

	for (size_t i = 0; i < HalfSpaceList_.size(); i++)//不需要添加辅助平面的特征点生成
	{
		for (size_t j = i+1; j < HalfSpaceList_.size(); j++)
		{
			if (j == i + HalfSpaceList_.size()/2)
				continue;

			if (!strcmp(HalfSpaceList_[j]->name_, "plane") && !strcmp(HalfSpaceList_[i]->name_, "plane"))
			{
				CVector3D pVector3D1 = ((SPlane*)HalfSpaceList_[i])->position_->verAxis;
				pVector3D1.Normalize();
				CVector3D pVector3D2 = ((SPlane*)HalfSpaceList_[j])->position_->verAxis;
				pVector3D2.Normalize();
				if ( fabs(pVector3D1 | pVector3D2) == fabs(pVector3D1 | pVector3D1) )
					continue;
			}
			for (size_t k = j + 1; k < HalfSpaceList_.size(); k++)
			{
				if ( k == j + HalfSpaceList_.size()/2 || k == i + HalfSpaceList_.size()/2)
					continue;
				if (!strcmp(HalfSpaceList_[j]->name_, "plane") && !strcmp(HalfSpaceList_[k]->name_, "plane"))
				{
					CVector3D pVector3D1 = ((SPlane*)HalfSpaceList_[k])->position_->verAxis;
					pVector3D1.Normalize();
					CVector3D pVector3D2 = ((SPlane*)HalfSpaceList_[j])->position_->verAxis;
					pVector3D2.Normalize();
					if (fabs(pVector3D1 | pVector3D2) == fabs(pVector3D1 | pVector3D1))
						continue;
				}
				if (!strcmp(HalfSpaceList_[i]->name_, "plane") && !strcmp(HalfSpaceList_[k]->name_, "plane"))
				{
					CVector3D pVector3D1 = ((SPlane*)HalfSpaceList_[k])->position_->verAxis;
					pVector3D1.Normalize();
					CVector3D pVector3D2 = ((SPlane*)HalfSpaceList_[i])->position_->verAxis;
					pVector3D2.Normalize();
					if (fabs(pVector3D1 | pVector3D2) == fabs(pVector3D1 | pVector3D1))
						continue;
				}
				//将三个面方程带入matlab求解
				SetSurfaceIntersectPoints(HalfSpaceList_[i], HalfSpaceList_[j], HalfSpaceList_[k]);
				CITCPList_.insert(CITCPList_.end(), MP_.interPointList_.begin(), MP_.interPointList_.end());
			}
		}
	}
	UniqueHalfCharacteristicPoint();//唯一化规范相交项组件特征点集合，同时也是生成规范相交项特征点集合的过程
}

GeometryData* StepEntity::CloneEntity(GeometryData* geo)
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

void StepEntity::GenerateOffsetHalfSpaceList()
{
	vector<SFace*>().swap(ForwardHalfSpaceList_);
	vector<SFace*>().swap(BackwardHalfSpaceList_);
	for (size_t iter = 0; iter < CSGHalfSpaceList_.size(); iter++)
	{
		Standard_CString name = CSGHalfSpaceList_[iter]->name_;
		if (!strcmp(name, "plane"))
		{
			SPlane* ForwardPlane = new SPlane;
			SPlane* BackwardPlane = new SPlane;
			ForwardPlane->name_ = name;
			BackwardPlane->name_ = name;
			ForwardPlane->position_ = CloneEntity(((SPlane*)CSGHalfSpaceList_[iter])->position_);
			BackwardPlane->position_ = CloneEntity(((SPlane*)CSGHalfSpaceList_[iter])->position_);
			CVector3D pVector3D1 = ((SPlane*)CSGHalfSpaceList_[iter])->position_->verAxis;
			pVector3D1 *= OFFSET;
			ForwardPlane->position_->point += pVector3D1;
			BackwardPlane->position_->point -= pVector3D1;
			ForwardPlane->GenerateCoefficient();
			BackwardPlane->GenerateCoefficient();
			ForwardHalfSpaceList_.push_back(ForwardPlane);
			BackwardHalfSpaceList_.push_back(BackwardPlane);
		}
		if (!strcmp(name, "spherical_surface"))
		{
			SSpherical*  ForwardSpherical_surface = new SSpherical;
			SSpherical*  BackwardSpherical_surface = new SSpherical;
			ForwardSpherical_surface->name_ = name;
			BackwardSpherical_surface->name_ = name;
			ForwardSpherical_surface->position_ = 
				CloneEntity(((SSpherical*)CSGHalfSpaceList_[iter])->position_);
			BackwardSpherical_surface->position_ =
				CloneEntity(((SSpherical*)CSGHalfSpaceList_[iter])->position_);
			double rads = ((SSpherical*)CSGHalfSpaceList_[iter])->radius_;
			ForwardSpherical_surface->radius_ = rads + OFFSET;
			BackwardSpherical_surface->radius_ = rads - OFFSET;
			ForwardSpherical_surface->GenerateCoefficient();
			BackwardSpherical_surface->GenerateCoefficient();
			ForwardHalfSpaceList_.push_back(ForwardSpherical_surface);
			BackwardHalfSpaceList_.push_back(BackwardSpherical_surface);
		}
		if (!strcmp(name, "conical_surface"))
		{
			SConical* ForwardConical_surface = new SConical;
			SConical* BackwardConical_surface = new SConical;
			ForwardConical_surface->name_ = name;
			BackwardConical_surface->name_ = name;
			ForwardConical_surface->radius_ = ((SConical*)CSGHalfSpaceList_[iter])->radius_;
			BackwardConical_surface->radius_ = ((SConical*)CSGHalfSpaceList_[iter])->radius_;
			ForwardConical_surface->semi_angle_ = ((SConical*)CSGHalfSpaceList_[iter])->semi_angle_;
			BackwardConical_surface->semi_angle_ = ((SConical*)CSGHalfSpaceList_[iter])->semi_angle_;
			ForwardConical_surface->position_ = 
				CloneEntity(((SConical*)CSGHalfSpaceList_[iter])->position_);
			BackwardConical_surface->position_ = 
				CloneEntity(((SConical*)CSGHalfSpaceList_[iter])->position_);
			CVector3D pVector3D1(((SConical*)CSGHalfSpaceList_[iter])->position_->verAxis.dx, 
				((SConical*)CSGHalfSpaceList_[iter])->position_->verAxis.dy,
				((SConical*)CSGHalfSpaceList_[iter])->position_->verAxis.dz);
			pVector3D1.Normalize();
			pVector3D1 *= OFFSET;
			ForwardConical_surface->position_->point += pVector3D1;
			BackwardConical_surface->position_->point -= pVector3D1;
			ForwardConical_surface->GenerateCoefficient();
			BackwardConical_surface->GenerateCoefficient();
			ForwardHalfSpaceList_.push_back(ForwardConical_surface);
			BackwardHalfSpaceList_.push_back(BackwardConical_surface);
		}
		if (!strcmp(name, "cylindrical_surface"))
		{
			SCylindrical* ForwardCylindrical_surface = new SCylindrical;
			SCylindrical* BackwardCylindrical_surface = new SCylindrical;
			ForwardCylindrical_surface->name_ = name;
			BackwardCylindrical_surface->name_ = name;
			ForwardCylindrical_surface->position_ =
				CloneEntity(((SCylindrical*)CSGHalfSpaceList_[iter])->position_);
			BackwardCylindrical_surface->position_ = 
				CloneEntity(((SCylindrical*)CSGHalfSpaceList_[iter])->position_);
			ForwardCylindrical_surface->radius_ =
				((SCylindrical*)CSGHalfSpaceList_[iter])->radius_ + OFFSET;
			BackwardCylindrical_surface->radius_ = 
				((SCylindrical*)CSGHalfSpaceList_[iter])->radius_ - OFFSET;
			ForwardCylindrical_surface->GenerateCoefficient();
			BackwardCylindrical_surface->GenerateCoefficient();
			ForwardHalfSpaceList_.push_back(ForwardCylindrical_surface);
			BackwardHalfSpaceList_.push_back(BackwardCylindrical_surface);
		}
		if(!strcmp(name, "toroidal_surface"))
		{
			SToroidal* ForwardToroidal_surface = new SToroidal;
			SToroidal* BackwardToroidal_surface = new SToroidal;
			ForwardToroidal_surface->name_ = name;
			BackwardToroidal_surface->name_ = name;
			ForwardToroidal_surface->position_ =
				CloneEntity(((SCylindrical*)CSGHalfSpaceList_[iter])->position_);
			BackwardToroidal_surface->position_ =
				CloneEntity(((SCylindrical*)CSGHalfSpaceList_[iter])->position_);
			ForwardToroidal_surface->major_radius_ = 
				((SToroidal*)CSGHalfSpaceList_[iter])->major_radius_ + OFFSET;
			ForwardToroidal_surface->minor_radius_ =
				((SToroidal*)CSGHalfSpaceList_[iter])->minor_radius_ + OFFSET;
			BackwardToroidal_surface->major_radius_ =
				((SToroidal*)CSGHalfSpaceList_[iter])->major_radius_ - OFFSET;
			BackwardToroidal_surface->minor_radius_ =
				((SToroidal*)CSGHalfSpaceList_[iter])->minor_radius_ - OFFSET;
			ForwardToroidal_surface->GenerateCoefficient();
			BackwardToroidal_surface->GenerateCoefficient();
			ForwardHalfSpaceList_.push_back(ForwardToroidal_surface);
			ForwardHalfSpaceList_.push_back(BackwardToroidal_surface);
		}
	}
}

void StepEntity::PMCtest()
{
	bool isTangency = false;
	bool isOnBound = false;
	int m = 0;
	int testwx = 0;
	for(size_t i = 0; i < pow(((double)2.0), ((int)CSGHalfSpaceList_.size())); i++)
	{
		if (CITIndex_[i].isNotNull == true)
		{
			int IntersetedNum = 0;
			testwx++;
			//如果交到曲面边界上，重新生成射线
			if (isOnBound == true)
			{
				m++;
				isOnBound = false;
			}
			else
				m = 0;
			for(size_t j = 0; j < NatlHalfSpaceList_.size(); j++)
			{
				SPlane* assistPlane1 = new SPlane;
				SPlane* assistPlane2 = new SPlane;
				assistPlane1->name_ = "plane";
				assistPlane2->name_ = "plane";
				CVector3D pVector3D1(0, m, 1);
				pVector3D1.GenerateTFace(assistPlane1->coefficient_, CITPList_[CITIndex_[i].tListIndex], 1);
				pVector3D1.GenerateTFace(assistPlane2->coefficient_, CITPList_[CITIndex_[i].tListIndex], 2);
				
				SetSurfaceIntersectPoints(assistPlane1, assistPlane2, NatlHalfSpaceList_[j]);
				delete assistPlane1,assistPlane2;
				//判断重根，如果曲面是球面，柱面，那么直线与其相交一次，必然是相切，如果交两次，必然不是相切；
				//对于锥面，如果交一次，且直线与锥面轴线相垂直，那么必然是相切，否则不相切
				//相切的情况算相交两次
				if (!strcmp(NatlHalfSpaceList_[j]->name_, "conical_surface"))
				{
					CVector3D pVector3D2 = ((SConical*)NatlHalfSpaceList_[j])->position_->verAxis;
					if (IS_ZERO(pVector3D1 | pVector3D2) && MP_.interPointList_.size() == 1)
						isTangency = true;
					else
						isTangency = false;
				}
				else if (!strcmp(NatlHalfSpaceList_[j]->name_, "plane"))
					isTangency = false;
				else
				{
					if(MP_.interPointList_.size() == 1)
						isTangency = true;
					else
						isTangency = false;
				}
	
				for (size_t k = 0; k < MP_.interPointList_.size(); k++)
				{
					//射线范围之外的点不计算
					CVector3D pVector3Ddir = CVector3D(
						MP_.interPointList_[k].x - CITPList_[CITIndex_[i].tListIndex].x,
						MP_.interPointList_[k].y - CITPList_[CITIndex_[i].tListIndex].y,
						MP_.interPointList_[k].z - CITPList_[CITIndex_[i].tListIndex].z);
					if (IS_NEGATIVE((pVector3Ddir | pVector3D1)))
						continue;
					if (CSGHalfSpaceList_.size() == 1)
					{
						IntersetedNum++;
						continue;
					}
					
					int isInBound = IsInBound(NatlHalfSpaceList_[j], MP_.interPointList_[k], MP2D_);
					if (isInBound == 1)
					{
						if (isTangency == true)
							IntersetedNum += 2;
						else
							IntersetedNum++;
					}
					else if (isInBound == 0)//交到曲面边界上，舍弃，重新生成射线，重新计算该点
					{
						isOnBound = true;
						break;
					}
				}
				if (isOnBound == true)
				{
					i--;
					break;
				}
			}
			if (isOnBound == false)
			{
				if (IntersetedNum % 2 == 1)
				{
					CITIndex_[i].tPosition = InSolid;
// 					cout<<CITPList_[CITIndex_[i].tListIndex].x<<endl<<CITPList_[CITIndex_[i].tListIndex].y<<endl<<CITPList_[CITIndex_[i].tListIndex].z
// 						<<endl<<endl;
				}
				else
					CITIndex_[i].tPosition = OutSolid;
			}
		}
	}
}

template < class T>
string ConvertToString(T value)
{
	stringstream ss;
	ss << value;
	return ss.str();
}

void StepEntity::Output(int * p_m, int * p_n,int splitRepetitionNum, string p_shpnm, vector<string>& vecOut, bool p_withvoid, bool p_spdir, bool is_outer, bool is_last)
{
	string str1, str2;
	if (p_withvoid && (!is_outer))
	{
		str2 = vecOut.back();
		vecOut.pop_back();
		str1 = vecOut.back();
		vecOut.pop_back();
	}
	bool firsttime = true;
	string BoolExpression = "";
	if (p_withvoid && (!is_outer) && (!p_spdir))
		BoolExpression += "#";
	if (p_withvoid)
		BoolExpression += "( ";

	int intimes = 0;
	for (int i = 0; i < pow(((double)2.0), ((int)CSGHalfSpaceList_.size())); i++)
	{
		if (CITIndex_[i].tPosition == InSolid)
		{
			intimes++;
			if (intimes >= 2)
			{
				muticp_ = true;
				break;
			}
		}
	}
	for (int i = 0; i < pow(((double)2.0), ((int)CSGHalfSpaceList_.size())); i++)
	{
		if (CITIndex_[i].tPosition == InSolid)
		{
			if (muticp_)
			{
				if (!firsttime)
					BoolExpression += ":";
				else
					firsttime = false;
				BoolExpression += "( ";
			}
			for (size_t j = 0; j < CSGHalfSpaceList_.size(); j++)
			{
				char test[5];
				if (CIT_[i][CSGHalfSpaceList_.size() - j - 1] == POSITIVE)
				{
					if (!strcmp(CSGHalfSpaceList_[j]->name_, "conical_surface"))
					{
						BoolExpression += "(+";
						_itoa((*p_n) + j, test, 10);
						BoolExpression += test;
						BoolExpression += ":-";
						_itoa((*p_n) + j + 1, test, 10);
						BoolExpression += test;
						BoolExpression += ")";
					}
					else
					{
						BoolExpression += "+";
						_itoa((*p_n) + j, test, 10);
						BoolExpression += test;
						BoolExpression += " ";
					}
				}
				else
				{
					if (!strcmp(CSGHalfSpaceList_[j]->name_, "conical_surface"))
					{
						BoolExpression += "(-";
						_itoa((*p_n) + j, test, 10);
						BoolExpression += test;
						BoolExpression += " +";
						_itoa((*p_n) + j + 1, test, 10);
						BoolExpression += test;
						BoolExpression += ")";
					}
					else
					{
						BoolExpression += "-";
						_itoa((*p_n) + j, test, 10);
						BoolExpression += test;
						BoolExpression += " ";
					}
				}
			}
			if (muticp_)
				BoolExpression += ") ";
		}
	}
	if (p_withvoid)
		BoolExpression += ") ";
	if ((p_withvoid&&is_outer) || (!p_withvoid))
	{
		str1 += ConvertToString((*p_m)) + " 0 ";
		(*p_m)++;
	}
	str1 += BoolExpression;
	if (is_last || (!p_withvoid))
		str1 += " $ " + p_shpnm + "\n";
	vecOut.push_back(str1);

	auto tt = setString_.insert(p_shpnm);

	string num;
	if (tt.second)
		num = " ";
	else
	{
		if(splitRepetitionNum == 0)
		{
			testNum++;
			num = " " + ConvertToString(testNum) + " ";
		}
		if(testNum == 0)
			num = " ";
		else		
			num = " " + ConvertToString(testNum) + " ";
	}


	for (size_t i = 0; i < CSGHalfSpaceList_.size(); i++)
	{

		if (!strcmp(CSGHalfSpaceList_[i]->name_, "plane"))
		{
			str2 += ConvertToString((*p_n)) + ""
				+ num + "P"
				+ " " + ConvertToString(((SPlane*)CSGHalfSpaceList_[i])->coefficient_[3] * 2.0)
				+ " " + ConvertToString(((SPlane*)CSGHalfSpaceList_[i])->coefficient_[6] * 2.0)
				+ " " + ConvertToString(((SPlane*)CSGHalfSpaceList_[i])->coefficient_[8] * 2.0)
				+ " " + ConvertToString(-((SPlane*)CSGHalfSpaceList_[i])->coefficient_[9])
				+ "\n";
		}

		if (!strcmp(CSGHalfSpaceList_[i]->name_, "spherical_surface"))
		{
			str2 += ConvertToString((*p_n)) + "" + num + "S"
				+ " " + ConvertToString(((SSpherical*)CSGHalfSpaceList_[i])->position_->point.x)
				+ " " + ConvertToString(((SSpherical*)CSGHalfSpaceList_[i])->position_->point.y)
				+ " " + ConvertToString(((SSpherical*)CSGHalfSpaceList_[i])->position_->point.z)
				+ " " + ConvertToString(((SSpherical*)CSGHalfSpaceList_[i])->radius_)
				+ "\n";
		}

		if (!strcmp(CSGHalfSpaceList_[i]->name_, "conical_surface"))
		{
			str2 += ConvertToString((*p_n)) + "" + num + "GQ" + " "
				+ ConvertToString(((SConical*)CSGHalfSpaceList_[i])->coefficient_[0]) + " "
				+ ConvertToString(((SConical*)CSGHalfSpaceList_[i])->coefficient_[4]) + " "
				+ ConvertToString(((SConical*)CSGHalfSpaceList_[i])->coefficient_[7]) + " "
				+ ConvertToString(((SConical*)CSGHalfSpaceList_[i])->coefficient_[1] * 2.0) + " "
				+ ConvertToString(((SConical*)CSGHalfSpaceList_[i])->coefficient_[5] * 2.0) + " "
				+ ConvertToString(((SConical*)CSGHalfSpaceList_[i])->coefficient_[2] * 2.0) + " "
				+ ConvertToString(((SConical*)CSGHalfSpaceList_[i])->coefficient_[3] * 2.0) + " "
				+ ConvertToString(((SConical*)CSGHalfSpaceList_[i])->coefficient_[6] * 2.0) + " "
				+ ConvertToString(((SConical*)CSGHalfSpaceList_[i])->coefficient_[8] * 2.0) + " "
				+ ConvertToString(((SConical*)CSGHalfSpaceList_[i])->coefficient_[9]) + " "
				+ "\n";
			//加一个辅助平面
			double* vertex = ((SConical*)CSGHalfSpaceList_[i])->vertex_;
			str2 += ConvertToString(++(*p_n)) + "" + num + "P" + " "
				+ ConvertToString(((SConical*)CSGHalfSpaceList_[i])->position_->verAxis.dx) + " "
				+ ConvertToString(((SConical*)CSGHalfSpaceList_[i])->position_->verAxis.dy) + " "
				+ ConvertToString(((SConical*)CSGHalfSpaceList_[i])->position_->verAxis.dz) + " "
				+ ConvertToString(((SConical*)CSGHalfSpaceList_[i])->position_->verAxis.dx * vertex[0])
				+ ConvertToString(((SConical*)CSGHalfSpaceList_[i])->position_->verAxis.dy * vertex[1])
				+ ConvertToString(((SConical*)CSGHalfSpaceList_[i])->position_->verAxis.dz * vertex[2])
				+ "\n";
		}

		if (!strcmp(CSGHalfSpaceList_[i]->name_, "cylindrical_surface"))
		{
			str2 += ConvertToString((*p_n)) + "" + num + "GQ" + " "
				+ ConvertToString(CompareNum(((SCylindrical*)CSGHalfSpaceList_[i])->coefficient_[0])) + " "
				+ ConvertToString(CompareNum(((SCylindrical*)CSGHalfSpaceList_[i])->coefficient_[4])) + " "
				+ ConvertToString(CompareNum(((SCylindrical*)CSGHalfSpaceList_[i])->coefficient_[7])) + " "
				+ ConvertToString(CompareNum(((SCylindrical*)CSGHalfSpaceList_[i])->coefficient_[1] * 2.0)) + " "
				+ ConvertToString(CompareNum(((SCylindrical*)CSGHalfSpaceList_[i])->coefficient_[5] * 2.0)) + " "
				+ ConvertToString(CompareNum(((SCylindrical*)CSGHalfSpaceList_[i])->coefficient_[2] * 2.0)) + " "
				+ ConvertToString(CompareNum(((SCylindrical*)CSGHalfSpaceList_[i])->coefficient_[3] * 2.0)) + " "
				+ ConvertToString(CompareNum(((SCylindrical*)CSGHalfSpaceList_[i])->coefficient_[6] * 2.0)) + " "
				+ ConvertToString(CompareNum(((SCylindrical*)CSGHalfSpaceList_[i])->coefficient_[8] * 2.0)) + " "
				+ ConvertToString(CompareNum(((SCylindrical*)CSGHalfSpaceList_[i])->coefficient_[9])) + " "
				+ "\n";
		}
		(*p_n)++;
	}
	
	vecOut.push_back(str2);
}

double StepEntity::CompareNum(double matrix)
{
	if(IS_ZERO(matrix))
		return 0.0;
	else
		return matrix;
}

ConvexConcave StepEntity::MakeBoundFace(vector<Curve*> edgeLoop_)
{
	vector<CPoint3D> boundPtarray;
	vector<CTriChip*> triList;
	vector<EdgeCurveVertex*>().swap(axisVertex_);

	for(auto j = edgeLoop_.begin(); j != edgeLoop_.end(); j++)
	{
		ORIENTATION ori; 
		ori.orientedEdgeOri = (*j)->orientedEdgeOri_;//oriented_dege orientation
		ori.edgeCurveOri = (*j)->edgeCurvesameSense_;//edge_curve orientation
		if(!strcmp("circle", (*j)->curveName_))
			CircleInfo((CIRCLE*)(*j), ori, (*j)->edgeStart_, (*j)->edgeEnd_);
		if (!strcmp("ellipse", (*j)->curveName_))
			EllipseInfo((ELLIPSE*)(*j), ori, (*j)->edgeStart_, (*j)->edgeEnd_);
// 		if (!strcmp("surface_curve"(*j)->curveName_))
// 			SurfaceCurveInfo(pcurve, (*j)->edgeStart_, (*j)->edgeEnd_);
// 			if (!strcmp(("b_spline_curve_with_knots"), pcurve->className()))
// 				 		BSplineCurveWithKnotsInfo(pcurve, eStart, eEnd);
		if(ori.orientedEdgeOri == ori.edgeCurveOri)
		{
			if(0 == boundPtarray.size())
			{
				boundPtarray.push_back((*j)->edgeStart_);
			}
			boundPtarray.push_back((*j)->edgeEnd_);
		}
		else
		{
			if(0 == boundPtarray.size())
			{
				boundPtarray.push_back((*j)->edgeEnd_);
			}
			boundPtarray.push_back((*j)->edgeStart_);
		}
	}
	
	boundPtarray.pop_back();

	if ( boundPtarray.size() >= 3 )
		TriangleSeparate(boundPtarray, triList);

	for(size_t i = 0; i < triList.size(); i++)
	{
		CVector3D pnormal = (triList[i]->vex_[1] - triList[i]->vex_[0]) * (triList[i]->vex_[2] - triList[i]->vex_[0]);
		pnormal.Normalize();
		EdgeCurveVertex* axis = new EdgeCurveVertex;
		CPoint3D pStart(triList[i]->vex_[0].x, triList[i]->vex_[0].y, triList[i]->vex_[0].z);
		axis->cartesianStart = pStart;
		CPoint3D pEnd(triList[i]->vex_[1].x, triList[i]->vex_[1].y, triList[i]->vex_[1].z);
		axis->cartesianEnd = pEnd;
		CVector3D cV(pnormal.dx, pnormal.dy, pnormal.dz);
		axis->axisDirection = cV;
		axisVertex_.push_back(axis);
	}
	boundPtarray.clear();
	vector<CPoint3D>().swap(boundPtarray);
	return UniqueBoundFaceList();
}

int StepEntity::IsInBound(SFace* face, CPoint3D testPoint, InterProcess &mP)
{
	if (!strcmp(face->name_, "plane"))
	{
		int intersetedNum;
		int plusMinus;
		if (face->adFaceSameSense_)
			plusMinus = 1;
		else
			plusMinus = -1;
		CVector3D pVector3D1(face->position_->verAxis.dx * plusMinus,
			face->position_->verAxis.dy * plusMinus,face->position_->verAxis.dz * plusMinus );
		CVector3D pVector3D2 = pVector3D1 * CVector3D(0, 0, 1);
		double angle = acos((pVector3D1 | CVector3D(0, 0, 1)) / pVector3D1.GetLength());
		CMatrix3D rotateMatrix;
		rotateMatrix = CMatrix3D::CreateRotateMatrix(angle, pVector3D2);
		testPoint = testPoint*rotateMatrix;
		CVector3D dir(0, 0, 0);
	reset:dir.dx++;
		LINE* pRay2d = new LINE;
		pRay2d->pnt_ = testPoint;
		pRay2d->dir_ = dir;
		intersetedNum = 0;
		for (auto i = face->faceBounds_.begin(); i != face->faceBounds_.end(); i++)
		{
			for (auto j = (*i)->edgeLoop_.begin(); j < (*i)->edgeLoop_.end(); j++)
			{
				CPoint3D pPoint3D1 = CPoint3D(
					(*j)->edgeStart_.x, (*j)->edgeStart_.y, (*j)->edgeStart_.z)* rotateMatrix;
				CPoint3D pPoint3D2 = CPoint3D(
					(*j)->edgeEnd_.x, (*j)->edgeEnd_.y, (*j)->edgeEnd_.z)* rotateMatrix;
				
				ORIENTATION ori;
				ori.orientedEdgeOri = (*j)->orientedEdgeOri_;
				ori.edgeCurveOri = (*j)->edgeCurvesameSense_;
				if (!strcmp((*j)->curveName_, "line"))
				{
					LINE* pLine1 = ((LINE*)(*j)); 
					mP.SetInterPointList(pLine1->GenerateCoefficient(rotateMatrix), pRay2d);

					for(size_t k = 0; k < mP.interPointList_.size(); k++)
					{
						//射线范围之外的点不计算
						if (IS_NEGATIVE((CVector3D(mP.interPointList_[k].x - testPoint.x, mP.interPointList_[k].y - testPoint.y, 0) | dir)))
							continue;
						//如果TestPoint在边界上，直接返回
						if (IS_ZERO(mP.interPointList_[k].x - testPoint.x) && IS_ZERO(mP.interPointList_[k].y - testPoint.y))
							return 0;
						//如果交到曲线顶点，重新生成射线
						if ((IS_ZERO(mP.interPointList_[k].x - pPoint3D1.x) && IS_ZERO(mP.interPointList_[k].y - pPoint3D1.y)) 
							|| (IS_ZERO(mP.interPointList_[k].x - pPoint3D2.x) && IS_ZERO(mP.interPointList_[k].y - pPoint3D2.y)))
						{
							goto reset;
						}
						//判断是否在有效范围内
						if (IS_NEGATIVE(( CVector3D(mP.interPointList_[k].x - pPoint3D1.x, mP.interPointList_[k].y - pPoint3D1.y, 0)
							| CVector3D(mP.interPointList_[k].x - pPoint3D2.x, mP.interPointList_[k].y - pPoint3D2.y, 0))))
							intersetedNum++;
					}
				}
				else if (!strcmp((*j)->curveName_, "circle"))
				{
					bool isTangency = false;
					CIRCLE* pCircle = ((CIRCLE*)(*j));
					mP.SetInterPointList(pCircle->GenerateCoefficient(rotateMatrix), pRay2d);
					if (mP.interPointList_.size() == 1)
						isTangency = true;
					for(size_t k = 0; k < mP.interPointList_.size(); k++)
					{
						//射线范围之外的点不计算
						if (IS_NEGATIVE((CVector3D(mP.interPointList_[k].x - testPoint.x, mP.interPointList_[k].y - testPoint.y, 0) | dir)))
							continue;
						//如果TestPoint在边界上，直接返回
						if (IS_ZERO(mP.interPointList_[k].x - testPoint.x) && IS_ZERO(mP.interPointList_[k].y - testPoint.y))
							return 0;
						//如果交到曲线顶点，重新生成射线
						if ((IS_ZERO(mP.interPointList_[k].x - pPoint3D1.x) && IS_ZERO(mP.interPointList_[k].y - pPoint3D1.y))
							|| (IS_ZERO(mP.interPointList_[k].x - pPoint3D2.x) && IS_ZERO(mP.interPointList_[k].y - pPoint3D2.y)))
						{
							goto reset;
						}
						//判断是否在有效范围内
						if (ori.orientedEdgeOri == ori.edgeCurveOri)
						{
							if ((IS_POSITIVE(((((CVector3D(mP.interPointList_[k].x - pPoint3D1.x, mP.interPointList_[k].y - pPoint3D1.y, 0)
								*CVector3D(pPoint3D2.x - pPoint3D1.x, pPoint3D2.y - pPoint3D1.y, 0)) | CVector3D(0, 0, 1)))*plusMinus))) || (pPoint3D1 == pPoint3D2) )
							{
								if (isTangency)
									intersetedNum += 2;
								else
									intersetedNum++;
							}
						}
						else
						{
							if ((IS_NEGATIVE(((((CVector3D(mP.interPointList_[k].x - pPoint3D1.x, mP.interPointList_[k].y - pPoint3D1.y, 0)
								*CVector3D(pPoint3D2.x - pPoint3D1.x, pPoint3D2.y - pPoint3D1.y, 0)) | CVector3D(0, 0, 1)))*plusMinus)))
								|| (pPoint3D1 == pPoint3D2))
							{
								if (isTangency)
									intersetedNum += 2;
								else
									intersetedNum++;
							}
						}
					}
				}
				else if ((*j)->curveName_, "ellipse")
				{
					bool isTangency = false;
					ELLIPSE* pEll = ((ELLIPSE*)(*j));
					mP.SetInterPointList(pEll->GenerateCoefficient(rotateMatrix), pRay2d);
					if(mP.interPointList_.size() == 1)
						isTangency = true;
					for(size_t k = 0; k < mP.interPointList_.size(); k++)
					{
						//射线范围之外的点不计算
						if(IS_NEGATIVE((CVector3D(mP.interPointList_[k].x - testPoint.x, mP.interPointList_[k].y - testPoint.y, 0) | dir)))
							continue;
						//如果TestPoint在边界上，直接返回
						if(IS_ZERO(mP.interPointList_[k].x - testPoint.x) && IS_ZERO(mP.interPointList_[k].y - testPoint.y))
							return 0;
						//如果交到曲线顶点，重新生成射线
						if((IS_ZERO(mP.interPointList_[k].x - pPoint3D1.x) && IS_ZERO(mP.interPointList_[k].y - pPoint3D1.y))
							|| (IS_ZERO(mP.interPointList_[k].x - pPoint3D2.x) && IS_ZERO(mP.interPointList_[k].y - pPoint3D2.y)))
						{
							goto reset;
						}
						//判断是否在有效范围内
						if(ori.orientedEdgeOri == ori.edgeCurveOri)
						{
							if((IS_POSITIVE(((((CVector3D(mP.interPointList_[k].x - pPoint3D1.x, mP.interPointList_[k].y - pPoint3D1.y, 0)
								*CVector3D(pPoint3D2.x - pPoint3D1.x, pPoint3D2.y - pPoint3D1.y, 0)) | CVector3D(0, 0, 1)))*plusMinus))) || (pPoint3D1 == pPoint3D2))
							{
								if(isTangency)
									intersetedNum += 2;
								else
									intersetedNum++;
							}
						}
						else
						{
							if((IS_NEGATIVE(((((CVector3D(mP.interPointList_[k].x - pPoint3D1.x, mP.interPointList_[k].y - pPoint3D1.y, 0)
								*CVector3D(pPoint3D2.x - pPoint3D1.x, pPoint3D2.y - pPoint3D1.y, 0)) | CVector3D(0, 0, 1)))*plusMinus)))
								|| (pPoint3D1 == pPoint3D2))
							{
								if(isTangency)
									intersetedNum += 2;
								else
									intersetedNum++;
							}
						}
					}
				}
				else if(!strcmp((*j)->curveName_, "b_spline_curve_with_knots"))
				{
// 					CurveBSpline spline;
// 					stp_b_spline_curve_with_knots* bSpline =
// 						ROSE_CAST(stp_b_spline_curve_with_knots, pcurve);
// 					spline.degree = bSpline->degree();
// 					spline.controlPointsList = bSpline->control_points_list();
// 					spline.curveForm = bSpline->curve_form();
// 					spline.closedCurve = bSpline->closed_curve();
// 					spline.selfIntersect = bSpline->self_intersect();
// 					spline.knotMultiplicities = bSpline->knot_multiplicities();
// 					spline.knots = bSpline->knots();
// 					spline.kontSpec = bSpline->knot_spec();
// 					Curve tempCurve;
// 					tempCurve.curveName = "b_spline_curve_with_knots";
// 					tempCurve.bSpline = GenerateCoefficient(RotateMatrix, spline);
// 					tempCurve.bSpline = spline;
// 					mP.SetInterPointList(tempCurve, pRay2d);
					continue;
				}
			}
		}
		if (intersetedNum % 2 == 1)
			return 1;
		else
			return -1;
	}
	else
	{
		int pIsInBoundTotal = 1;
		int pIsInBound;
		for(size_t i = 0; i < face->faceBounds_.size(); i++)
		{
			ConvexConcave cc = MakeBoundFace(face->faceBounds_[i]->edgeLoop_);
			if (cc == 0) //凸凹实体
			{
				printf("实体凸凹性不一致，暂不能处理！");
				exit(1);
			}
			if (face->adFaceSameSense_)
			{
				pIsInBound = 1;
				if (cc == convex)
				{
					for(size_t j = 0; j < axisVertex_.size(); j++)
					{
						CVector3D test(axisVertex_[j]->cartesianStart.x, 
							axisVertex_[j]->cartesianStart.y, 
							axisVertex_[j]->cartesianStart.z);
						CVector3D pVector3D1(testPoint.x - axisVertex_[j]->cartesianStart.x,
							testPoint.y - axisVertex_[j]->cartesianStart.y,
							testPoint.z - axisVertex_[j]->cartesianStart.z);
						CVector3D pVector3D2 = axisVertex_[j]->axisDirection;
						double test2 = pVector3D1 | pVector3D2;
						if (IS_NEGATIVE((pVector3D1 | pVector3D2)))
							pIsInBound = -1;
						else
						if (IS_ZERO((pVector3D1 | pVector3D2)))
						{
							pIsInBound = 0;
							return pIsInBound;
						}
					}
				}
				else
				{
					pIsInBound = -1;
					for(size_t j = 0; j < axisVertex_.size(); j++)
					{
						CVector3D test(axisVertex_[j]->cartesianStart.x, axisVertex_[j]->cartesianStart.y, axisVertex_[j]->cartesianStart.z);
						CVector3D pVector3D1(testPoint.x - axisVertex_[j]->cartesianStart.x, testPoint.y - axisVertex_[j]->cartesianStart.x, testPoint.z - axisVertex_[j]->cartesianStart.x);
						CVector3D pVector3D2 = axisVertex_[j]->axisDirection;
						if (IS_POSITIVE((pVector3D1 | pVector3D2)))
							pIsInBound = 1;
						else if (IS_ZERO((pVector3D1 | pVector3D2)))
						{
							pIsInBound = 0;
							return pIsInBound;
						}
					}
				}
				if (pIsInBound == -1)
					pIsInBoundTotal = -1;
			}
			else
			{
				if (cc == convex)
				{
					pIsInBound = 1;
					for(size_t j = 0; j < axisVertex_.size(); j++)
					{
						CVector3D test(axisVertex_[j]->cartesianStart.x, 
							axisVertex_[j]->cartesianStart.y,
							axisVertex_[j]->cartesianStart.z);
						CVector3D pVector3D1(testPoint.x - axisVertex_[j]->cartesianStart.x,
							testPoint.y - axisVertex_[j]->cartesianStart.y, 
							testPoint.z - axisVertex_[j]->cartesianStart.z);
						CVector3D pVector3D2 = axisVertex_[j]->axisDirection;
						if (IS_POSITIVE((pVector3D1 | pVector3D2)))
							pIsInBound = -1;
						else
						if (IS_ZERO((pVector3D1 | pVector3D2)))
						{
							pIsInBound = 0;
							return pIsInBound;
						}
					}
				}
				else
				{
					pIsInBound = -1;
					for (size_t j = 0; j < axisVertex_.size(); j++)
					{
						CVector3D pVector3D1(testPoint.x - axisVertex_[j]->cartesianStart.x, testPoint.y - axisVertex_[j]->cartesianStart.x, testPoint.z - axisVertex_[j]->cartesianStart.x);
						CVector3D pVector3D2 = axisVertex_[j]->axisDirection;
						if (IS_NEGATIVE((pVector3D1 | pVector3D2)))
							pIsInBound = 1;
						else if (IS_ZERO((pVector3D1 | pVector3D2)))
						{
							pIsInBound = 0;
							return pIsInBound;
						}
					}
				}
				if (pIsInBound == -1)
					pIsInBoundTotal = -1;
			}
		}
		return pIsInBoundTotal;
	}
}

void StepEntity::TriangleSeparate(vector<CPoint3D>& pArray, vector<CTriChip*>& vTriList)
{
	int number = 0;
	int cyclecount = 0;
	while(pArray.size()>3)
	{
		cyclecount++;
		while(true)
		{
			CPoint3D& pt0 = pArray[((number) % (pArray.size()))];	//delete a point, so maybe beyond the bound
			CPoint3D& pt1 = pArray[((number + 1) % (pArray.size()))];
			CPoint3D& pt2 = pArray[((number + 2) % (pArray.size()))];
			CVector3D normal = CVector3D(pt1.x - pt0.x, pt1.y - pt0.y, pt1.z - pt0.z)
				*CVector3D(pt2.x - pt1.x, pt2.y - pt1.y, pt2.z - pt1.z);
			if ( IS_ZERO(normal.GetLength()) )
			{
				number = (number+1) % (pArray.size());	//push
				continue;
			}
			BOOL noPtin = TRUE;
			BOOL crossed = FALSE;

			for (int j = ((number+3) % (pArray.size())); j != number; j = ((j+1)%(pArray.size())))
			{
				CPoint3D tmp = pArray[j];
				if (( tmp.IsInTheTriangle(pt0, pt1, pt2) ) == 1 )	//in triangle
				{
					noPtin = FALSE;
					break;
				}
				else if ( ( tmp.IsInTheTriangle(pt0, pt1, pt2) ) == 0 )//on the triangle side
				{
					CPoint3D& tmp1 = pArray[((j + 1) % (pArray.size()))];
					CPoint3D& tmp2 = pArray[((j - 1 + pArray.size()) % (pArray.size()))];
					if ( _Intersection(tmp, tmp1, pt0, pt1) || _Intersection(tmp, tmp1, pt1, pt2) || _Intersection(tmp, tmp1, pt2, pt0) )
					{
						vector<CPoint3D> tmpArray;
						int start1 = ((j + 1) % (pArray.size()));
						int start2, end1, end2, index;
						if ( tmp == pt0 )
							start2 = ((number+1) % (pArray.size()));
						else
						{
							if (tmp == pt1)
								start2 = ((number+2) % (pArray.size()));
							else
								start2 = ((number+3) % (pArray.size()));
						}

						index = start1;
						while (pArray[index] != tmp)
						{
							index = ((index + 1) % (pArray.size()));
						}
						end1 = ((index-1+pArray.size()) % (pArray.size()));

						index = start2;
						while (pArray[index] != tmp)
						{
							index = ((index + 1) % (pArray.size()));
						}
						end2 = ((index - 1 + pArray.size()) % (pArray.size()));

						for (index = start1; index != ((end1 + 1) % (pArray.size())); index = ((index + 1) % (pArray.size())))
						{
							tmpArray.push_back(pArray[index]);
						}
						for (index = ((end2+1)%(pArray.size())); index != start1; index = ((index + 1) % (pArray.size())))
						{
							tmpArray.push_back(pArray[index]);
						}
						for (index = start2; index != ((end2 + 1) % (pArray.size())); index = ((index + 1) % (pArray.size())))
						{
							tmpArray.push_back(pArray[index]);
						}
						for (index = ((end1 + 1) % (pArray.size())); index != start2; index = ((index + 1) % (pArray.size())))
						{
							tmpArray.push_back(pArray[index]);
						}
						vector<CPoint3D>().swap(pArray);
						pArray = tmpArray;
						crossed = TRUE;
						break;
					}
					else if ( _Intersection(tmp, tmp2, pt0, pt1) || _Intersection(tmp, tmp2, pt1, pt2) || _Intersection(tmp, tmp2, pt2, pt0) )
					{
						vector<CPoint3D> tmparray;
						int end1 = ((j-1+pArray.size())%(pArray.size()));
						int end2, start1, start2, index;
						if ( tmp == pt0 )
							end2 = ((number-1+pArray.size())%(pArray.size()));
						else
						{
							if ( tmp == pt1 )
								end2 = number;
							else
								end2 = ((number + 1) % (pArray.size()));
						}

						index = end1;
						while (pArray[index] != tmp)
						{
							index = ((index - 1 + pArray.size()) % (pArray.size()));
						}
						start1 = ((index + 1) % (pArray.size()));
						index = end2;
						while (pArray[index] != tmp)
						{
							index = ((index - 1 + pArray.size()) % (pArray.size()));
						}
						start2 = ((index + 1) % (pArray.size()));
						for (index = start1; index != ((end1 + 1) % (pArray.size())); index = ((index + 1) % (pArray.size())))
						{
							tmparray.push_back(pArray[index]);
						}
						for (index = ((end2 + 1) % (pArray.size())); index != start1; index = ((index + 1) % (pArray.size())))
						{
							tmparray.push_back(pArray[index]);
						}
						for (index = start2; index != ((end2 + 1) % (pArray.size())); index = ((index + 1) % (pArray.size())))
						{
							tmparray.push_back(pArray[index]);
						}
						for (index = ((end1 + 1) % (pArray.size())); index != start2; index = ((index + 1) % (pArray.size())))
						{
							tmparray.push_back(pArray[index]);
						}
						vector<CPoint3D>().swap(pArray);
						pArray = tmparray;
						crossed = TRUE;
						break;
					}
				}
			}
			if ( crossed )
			{
				number = ((number + 1) % (pArray.size()));
				continue;
			}
			if ( noPtin )
				break;
			else
				number = ((number + 1) % (pArray.size()));
		}
		CVector3D normal = CVector3D(pArray[((number + 1)%(pArray.size()))].x - pArray[number].x,
			pArray[((number + 1)%(pArray.size()))].y - pArray[number].y,
			pArray[((number + 1)%(pArray.size()))].z - pArray[number].z)
			* CVector3D(pArray[((number + 2)%(pArray.size()))].x - pArray[((number+1)%(pArray.size()))].x,
			pArray[((number + 2)%(pArray.size()))].y - pArray[((number+1)%(pArray.size()))].y,
			pArray[((number + 2)%(pArray.size()))].z - pArray[((number+1)%(pArray.size()))].z);
		CTriChip* tri = new CTriChip(pArray[number],pArray[((number + 1)%(pArray.size()))],pArray[((number + 2)%(pArray.size()))],normal);
		vTriList.push_back(tri);
		pArray.erase(pArray.begin() + ((number + 1) % (pArray.size())));
		if ( cyclecount < 0 )
			break;		//for dead loop
	}
	CVector3D normal = CVector3D(pArray[1].x - pArray[0].x,pArray[1].y - pArray[0].y,pArray[1].z - pArray[0].z)
		* CVector3D(pArray[2].x - pArray[1].x,pArray[2].y - pArray[1].y,pArray[2].z - pArray[1].z);
	CTriChip* tri = new CTriChip(pArray[0], pArray[1], pArray[2], normal);//????
	vTriList.push_back(tri);
}

void StepEntity::UniqueHalfCharacteristicPoint()
{
	vector<CPoint3D>().swap(CITPList_);
	for(int i = 0; i < CITCPList_.size(); i++)
	{
		int index = 0;
		if(i == 46)
			int we = 1;
		for(int j = 0; j < CSGHalfSpaceList_.size(); j++)
		{
			PointPosition PP = CSGHalfSpaceList_[j]->PointIsIn(CITCPList_[i]);
			if(PP == OUT_FACE)
				index += pow(2.0, j);
			else if(PP == IN_FACE)
				continue;
			else if(PP == ON_FACE)
			{
				CITCPList_.erase(CITCPList_.begin() + i);
				i--;
				index = -1;
				break;
			}
		}

		if (index == 1055)
			printf("");
		if((index != -1) && (CITIndex_[index].tListIndex == -1))
		{
			CITIndex_[index].tListIndex = CITPList_.size();
			CITIndex_[index].isNotNull = true;
			CITPList_.push_back(CITCPList_[i]);
			cout << CITIndex_[index].tListIndex + 1 << ':' 
				<< CITPList_[CITIndex_[index].tListIndex].x << ' '
				<< CITPList_[CITIndex_[index].tListIndex].y << ' ' 
				<< CITPList_[CITIndex_[index].tListIndex].z << endl;
		}
	}
}

ConvexConcave StepEntity::UniqueBoundFaceList()
{
	ConvexConcave cc = complex;
	UniqueAxisVertex();
	if ( 1 == axisVertex_.size() )
		cc = convex;
	for (size_t i = 0; i < axisVertex_.size()-1; i++ )
	{
		CVector3D pVector3D1(axisVertex_[i]->axisDirection.dx, axisVertex_[i]->axisDirection.dy, axisVertex_[i]->axisDirection.dz);
		CVector3D pVector3D2(axisVertex_[i + 1]->axisDirection.dx, axisVertex_[i + 1]->axisDirection.dy, axisVertex_[i + 1]->axisDirection.dz);
		CVector3D pVector3D3(axisVertex_[i]->cartesianStart.x - axisVertex_[i + 1]->cartesianStart.x,
			axisVertex_[i]->cartesianStart.y - axisVertex_[i + 1]->cartesianStart.y,
			axisVertex_[i]->cartesianStart.z - axisVertex_[i + 1]->cartesianStart.z);
		CVector3D pVector3D4(axisVertex_[i]->cartesianEnd.x - axisVertex_[i + 1]->cartesianStart.x,
			axisVertex_[i]->cartesianEnd.y - axisVertex_[i + 1]->cartesianStart.y,
			axisVertex_[i]->cartesianEnd.z - axisVertex_[i + 1]->cartesianStart.z);
		CVector3D pVector3D5(axisVertex_[i]->cartesianStart.x - axisVertex_[i + 1]->cartesianEnd.x,
			axisVertex_[i]->cartesianStart.y - axisVertex_[i + 1]->cartesianEnd.y,
			axisVertex_[i]->cartesianStart.z - axisVertex_[i + 1]->cartesianEnd.z);
		CVector3D pVector3D6(axisVertex_[i]->cartesianEnd.x - axisVertex_[i + 1]->cartesianStart.x,
			axisVertex_[i]->cartesianEnd.y - axisVertex_[i + 1]->cartesianStart.y,
			axisVertex_[i]->cartesianEnd.z - axisVertex_[i + 1]->cartesianStart.z);
		if ( IS_NEGATIVE(( pVector3D2 | pVector3D3 )) )
			cc = concave;
		else if ( IS_POSITIVE(( pVector3D2 | pVector3D3 )) )
			cc = convex;
		else
		{
			if (IS_NEGATIVE((pVector3D2 | pVector3D4)))
				cc = concave;
			else if (IS_POSITIVE((pVector3D2 | pVector3D4)))
				cc = convex;
			else
			{
				if (IS_NEGATIVE((pVector3D2 | pVector3D5)))
					cc = concave;
				else if (IS_POSITIVE((pVector3D2 | pVector3D5)))
					cc = convex;
				else
				{
					if (IS_NEGATIVE((pVector3D2 | pVector3D6)))
						cc = concave;
					else if (IS_POSITIVE((pVector3D2 | pVector3D6)))
						cc = convex;
					else
						cc = convex;
				}
			}
		}
	}
	return cc;
}

void StepEntity::UniqueAxisVertex()
{
	for ( size_t i = 0; i < axisVertex_.size(); i++ )
	{
		for ( size_t j = i + 1; j < axisVertex_.size(); j++ )
		{
			CVector3D pVector3D1(axisVertex_[i]->axisDirection.dx, axisVertex_[i]->axisDirection.dy, axisVertex_[i]->axisDirection.dz);
			CVector3D pVector3D2(axisVertex_[j]->axisDirection.dx, axisVertex_[j]->axisDirection.dy, axisVertex_[j]->axisDirection.dz);
			CVector3D pVector3D3(axisVertex_[i]->cartesianStart.x - axisVertex_[j]->cartesianStart.x,
				axisVertex_[i]->cartesianStart.y - axisVertex_[j]->cartesianStart.y,
				axisVertex_[i]->cartesianStart.z - axisVertex_[j]->cartesianStart.z);
			pVector3D1.Normalize();
			pVector3D2.Normalize();
			pVector3D3.Normalize();

			if ( pVector3D1 == pVector3D2 && IS_ZERO(pVector3D1|pVector3D3) )
			{
				axisVertex_.erase(axisVertex_.begin()+j);
				j--;
			}
		}
	}
}

CPoint3D StepEntity::GetPoint(stp_cartesian_point* pt)
{
	CPoint3D cPoint(pt->coordinates()->get(0),pt->coordinates()->get(1),pt->coordinates()->get(2));
	return cPoint;
}

void StepEntity::UniqueHalfSpaceList()
{
	for(size_t i = 0; i < CSGHalfSpaceList_.size(); i++)
	{
		CSGHalfSpaceList_[i]->GenerateCoefficient();
		for(size_t j = i + 1; j < CSGHalfSpaceList_.size(); j++)
		{
			if(CompareSTEPEntity(CSGHalfSpaceList_[i], CSGHalfSpaceList_[j]))
			{
				//移除重复半空间时也要生成系数数组，因为移除的很可能是原有的自然半空间
// 				if (CSGHalfSpaceList_[i]->entityID_ != CSGHalfSpaceList_[j]->entityID_)
// 					continue;
				CSGHalfSpaceList_[j]->GenerateCoefficient();
				CSGHalfSpaceList_.erase(CSGHalfSpaceList_.begin() + j);
				j--;
			}
		}
	}
}

// void StepEntity::StpDefRepresentationInfo(stp_definitional_representation* def )
// {
// 	SetOfstp_representation_item* items1 =  def->items();
// 	for(size_t i  = 0; i<items1->size(); i++ )
// 	{
// 		stp_representation_item* it = items1->get(i);
// 		stp_b_spline_curve_with_knots* pbSpline = ROSE_CAST(stp_b_spline_curve_with_knots, it);
// 		ListOfstp_cartesian_point* pList = pbSpline->control_points_list();
// 	}
// }

// void StepEntity::ListOfPcurveOrSurfaceInfo( ListOfstp_pcurve_or_surface* pList )
// {
// 	for (size_t i = 0; i < pList->size(); i++ )//pcurve
// 	{
// 		stp_pcurve_or_surface* pOrs = pList->get(i);
// 		stp_surface * sss = pOrs->_surface();
// 		stp_pcurve* pCurve = ROSE_CAST(stp_pcurve, sss);
// 		stp_surface* surf = pCurve-> basis_surface();
// 		stp_plane* plane = ROSE_CAST(stp_plane, surf);
// 		StpDefRepresentationInfo(pCurve->reference_to_curve());
// 	}
// }

// void StepEntity::LineInfo(stp_curve* cur)
// {
// 	stp_line* line = ROSE_CAST(stp_line, cur);
// 	stp_cartesian_point* carPoint = line->pnt();
// 	stp_vector* vect1 = line->dir();
// 	stp_direction* direct1 = vect1->orientation();
// 	double vectDouble1 = vect1->magnitude();
// 	ListOfdouble* dou = direct1->direction_ratios();
// }

// void StepEntity::BSplineCurveWithKnotsInfo(stp_curve* cur, stp_cartesian_point* eStart, stp_cartesian_point* eEnd)
// {
// 	stp_b_spline_curve_with_knots* bSpline = ROSE_CAST(stp_b_spline_curve_with_knots, cur);
// 	int degree = bSpline->degree();
// 	ListOfstp_cartesian_point* cars = bSpline->control_points_list();
// 	stp_b_spline_curve_form form = bSpline->curve_form();
// 	RoseLogical closedCurve = bSpline->closed_curve();
// 	RoseLogical selfIntersect = bSpline->self_intersect();
// 	ListOfInteger* knotM = bSpline->knot_multiplicities();
// 	ListOfDouble* knots = bSpline->knots();
// 	stp_knot_type kontSpec = bSpline->knot_spec();
// 	/////////////////////////////////
// }

void StepEntity::SaveAxisVertex(GeometryData axis2, ORIENTATION ori, CVector3D& cVector)
{
	double dir0 = axis2.verAxis.dx;
	double dir1 = axis2.verAxis.dy;
	double dir2 = axis2.verAxis.dz;
	if (ori.edgeCurveOri != ori.orientedEdgeOri)
	{
		dir0 *= -1;
		dir1 *= -1;
		dir2 *= -1;
		CVector3D cV(dir0, dir1, dir2);
		cVector = cV;
	}
	else
	{
		CVector3D cV(dir0, dir1, dir2);
		cVector = cV;
	}
}

void StepEntity::CircleInfo(CIRCLE* cir, ORIENTATION ori, CPoint3D eStart, CPoint3D eEnd)
{
	EdgeCurveVertex* tempAxis = new EdgeCurveVertex;
	tempAxis->cartesianStart = eStart;
	tempAxis->cartesianEnd = eEnd;
	CVector3D cv;
	SaveAxisVertex(cir->position_, ori, cv);
	tempAxis->axisDirection = cv;
	axisVertex_.push_back( tempAxis );
}

void StepEntity::EllipseInfo(ELLIPSE* cur, ORIENTATION ori,
	CPoint3D eStart, CPoint3D eEnd)
{
	EdgeCurveVertex* tempAxis = new EdgeCurveVertex;
	tempAxis->cartesianStart = eStart;
	tempAxis->cartesianEnd = eEnd;
	CVector3D cv;
	SaveAxisVertex(cur->position_, ori, cv);
	tempAxis->axisDirection = cv;
	axisVertex_.push_back(tempAxis);
}

// void StepEntity::SurfaceCurveInfo(stp_curve* cur, stp_cartesian_point*eStart, stp_cartesian_point* eEnd)
// {
// //1.line     2. pcurve
// 	stp_surface_curve* pSur = ROSE_CAST(stp_surface_curve, cur);
// 	stp_curve* curve = pSur->curve_3d();
// 	LineInfo(curve);//line
// 	ListOfPcurveOrSurfaceInfo(pSur->associated_geometry());
// }

void StepEntity::GenerateHalfSpaceList()
{
	bool isGen[1280];
	memset(isGen, true, sizeof(isGen));
	muticp_ = false;
	
	vector<SFace*>().swap(CSGHalfSpaceList_);
	vector<SFace*>().swap(HalfSpaceList_);

	for(size_t i = 0; i < NatlHalfSpaceList_.size(); i++)
	{
		for(size_t j = i + 1; j < NatlHalfSpaceList_.size(); j++)
		{
			if(CompareSTEPEntity(NatlHalfSpaceList_[i], NatlHalfSpaceList_[j]))
			{
				isGen[i] = false;
				isGen[j] = false;
			}
		}
	}

	for(size_t i = 0; i < NatlHalfSpaceList_.size(); i++)
	{
		CSGHalfSpaceList_.push_back(NatlHalfSpaceList_[i]);
		if(isGen[i])
		{
			GenerateSepHalfspace(NatlHalfSpaceList_[i]);
			for(size_t j = 0; j < SepHalfspacePlacementList_.size(); j++)
			{
				CSGHalfSpaceList_.push_back(SepHalfspacePlacementList_[j]);
				muticp_ = true;
			}
		}
	}
	UniqueHalfSpaceList();
	GenerateOffsetHalfSpaceList();
}

void StepEntity::GenerateSepHalfspace(SFace* face)
{
	if(!strcmp("plane", face->name_)) //如果是平面，不生成分割半空间，直接返回
		return;
	RoseBoolean sameSenseFace = face->adFaceSameSense_;
	for(auto iter = face->faceBounds_.begin(); iter != face->faceBounds_.end(); iter++)
	{
		RoseBoolean sameSenseBound = (*iter)->boundsOri_;//face_outer_bound orientation

		MakeBoundFace((*iter)->edgeLoop_);
		
		for(size_t k = 0; k < axisVertex_.size(); k++)
		{
			SFace* ent = face;
			ent->vertex_ = axisVertex_[k];
			SepHalfspacePlacementList_.push_back(ent);
		}
	}
}

void StepEntity::SetSurfaceIntersectPoints(SFace* Surf1, SFace* Surf2, SFace* Surf3)
{

	MP_.SetInterPointList(Surf1, Surf2, Surf3);

	//如果有锥面，因为锥面方程表示两片，而实际只是一片，因此要剔除另一片上的交点
	if (!strcmp(Surf1->name_, "conical_surface"))
	{
		CPoint3D vertex = ((SConical*)Surf1)->vertex_;
		CVector3D pVector3D1 = Surf1->position_->verAxis;
		for (int i = 0; i < MP_.interPointList_.size(); i++)
		{
			CVector3D pVector3D2(MP_.interPointList_[i].x - vertex.x, MP_.interPointList_[i].y - vertex.y, MP_.interPointList_[i].z - vertex.z );
			if (!IS_ZERO(pVector3D1 | pVector3D2) && (pVector3D1 | pVector3D2) < 0)
			{
				MP_.interPointList_.erase(MP_.interPointList_.begin()+i);
				i--;
			}
		}
	}
	if(!strcmp(Surf2->name_, "conical_surface"))
	{
		CPoint3D vertex = ((SConical*)Surf2)->vertex_;
		CVector3D pVector3D1 = Surf2->position_->verAxis;
		for (int i = 0; i < MP_.interPointList_.size(); i++)
		{
			CVector3D pVector3D2(MP_.interPointList_[i].x - vertex.x, 
				MP_.interPointList_[i].y - vertex.y,
				MP_.interPointList_[i].z - vertex.z);
			if (!IS_ZERO(pVector3D1 | pVector3D2) && (pVector3D1 | pVector3D2) < 0)
			{
				MP_.interPointList_.erase(MP_.interPointList_.begin()+i);
				i--;
			}
		}
	}
	if(!strcmp(Surf3->name_, "conical_surface"))
	{
		CPoint3D vertex = ((SConical*)Surf3)->vertex_;
		CVector3D pVector3D1 = Surf3->position_->verAxis;
		for (int i = 0; i < MP_.interPointList_.size(); i++)
		{
			CVector3D pVector3D2(MP_.interPointList_[i].x - vertex.x,
				MP_.interPointList_[i].y - vertex.y,
				MP_.interPointList_[i].z - vertex.z);
			if (IS_NEGATIVE((pVector3D1 | pVector3D2)))
			{
				MP_.interPointList_.erase(MP_.interPointList_.begin()+i);
				i--;
			}
		}
	}
}

bool StepEntity::CompareSTEPEntity(SFace* face1, SFace* face2)
{
	if(strcmp(face1->name_, face2->name_))
		return false;
	else
	{
		if(!strcmp("plane", face1->name_)) //平面
		{
			CVector3D pVector3D1 = ((SPlane*)face1)->position_->verAxis;
			CVector3D pVector3D2 = ((SPlane*)face2)->position_->verAxis;
			CVector3D pVector3D3(
				((SPlane*)face1)->position_->point.x - ((SPlane*)face2)->position_->point.x,
				((SPlane*)face1)->position_->point.y - ((SPlane*)face2)->position_->point.y,
				((SPlane*)face1)->position_->point.z - ((SPlane*)face2)->position_->point.z);
			pVector3D1.Normalize();
			pVector3D2.Normalize();
			pVector3D3.Normalize();
			if(pVector3D1 != pVector3D2 && pVector3D1 != (pVector3D2*(-1.0)))
				return false;
			if(!IS_ZERO(pVector3D1 | pVector3D3))
				return false;
		}
		if(!strcmp("conical_surface", face1->name_))//锥面
		{
			CVector3D pVector3D1 = ((SConical*)face1)->position_->verAxis;
			CVector3D pVector3D2 = ((SConical*)face2)->position_->verAxis;
			CVector3D pVector3D3(
				((SConical*)face1)->position_->point.x - ((SConical*)face2)->position_->point.x,
				((SConical*)face1)->position_->point.y - ((SConical*)face2)->position_->point.y,
				((SConical*)face1)->position_->point.z - ((SConical*)face2)->position_->point.z);
			if(!IS_ZERO(((SConical*)face1)->semi_angle_ - ((SConical*)face2)->semi_angle_))
				return false;
			if(pVector3D1 != pVector3D2)
				return false;

			double sign = pVector3D3.GetNormal() | pVector3D1;
			if(!IS_ZERO(
				fabs((1 / tan(((SConical*)face1)->semi_angle_))*((SConical*)face1)->radius_ 
				- (1 / tan(((SConical*)face2)->semi_angle_))*((SConical*)face2)->radius_)
				- sign * pVector3D3.GetLength()))
				return false;
		}
		if(!strcmp("spherical_surface", face1->name_))//球面
		{
			if(!IS_ZERO(((SSpherical*)face1)->radius_ - ((SSpherical*)face2)->radius_))
				return false;
			if(!IS_ZERO(((SSpherical*)face1)->position_->point.x - ((SSpherical*)face2)->position_->point.x)
				|| !IS_ZERO(((SSpherical*)face1)->position_->point.y - ((SSpherical*)face2)->position_->point.y)
				|| !IS_ZERO(((SSpherical*)face1)->position_->point.z - ((SSpherical*)face2)->position_->point.z))
				return false;
		}
		if(!strcmp("toroidal_surface", face1->name_))//圆环
		{
			return false;
		}
		if(!strcmp("cylindrical_surface", face1->name_))//圆柱
		{
			if(!IS_ZERO(((SCylindrical*)face1)->radius_ - ((SCylindrical*)face2)->radius_))
				return false;
			CVector3D pVector3D1 = ((SCylindrical*)face1)->position_->verAxis; 
			CVector3D pVector3D2 = ((SCylindrical*)face2)->position_->verAxis;
			CVector3D pVector3D3(
				((SCylindrical*)face1)->position_->point.x - ((SCylindrical*)face2)->position_->point.x,
				((SCylindrical*)face1)->position_->point.y - ((SCylindrical*)face2)->position_->point.y,
				((SCylindrical*)face1)->position_->point.z - ((SCylindrical*)face2)->position_->point.z);
			pVector3D1.Normalize();
			pVector3D2.Normalize();
			pVector3D3.Normalize();
			if(pVector3D1 != pVector3D2&&pVector3D1 != (pVector3D2*(-1.0)))
				return false;
			if(!IS_ZERO(pVector3D3.GetLength()) && pVector3D3 != pVector3D2&&pVector3D3 != (pVector3D2*(-1.0)))
				return false;
		}
	}
	return true;
}
