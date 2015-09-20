#include "stdafx.h"
#include "InterProcess.h"


InterProcess::InterProcess(void)
{
	tolerance_ = 1.0e-6;
	//tolerance_ = Precision::Confusion();
}

InterProcess::~InterProcess(void)
{

}

int InterProcess::SetInterPointList(Curve* C1, SFace* F1)
{
	vector<CPoint3D>().swap(interPointList_);
	return 0;
}

int InterProcess::SetInterPointList(SFace* F1, SFace* F2, SFace* F3)
{
	vector<CPoint3D>().swap(interPointList_);
	Handle(Geom_Surface) HS1 = F1->ToOCCT();
	Handle(Geom_Surface) HS2 = F2->ToOCCT();
	Handle(Geom_Surface) HS3 = F3->ToOCCT();
	 
	GeomAPI_IntSS intersector1(HS1, HS2, tolerance_); 
	if(intersector1.IsDone())
	{
		Standard_Integer nb1 = intersector1.NbLines(); 
		for(int i=0;i<nb1;i++)
		{
			Handle(Geom_Curve) HC1 = intersector1.Line(i + 1);
			GeomAPI_IntCS Intersector2(HC1, HS3);
			if(Intersector2.IsDone())
			{
				Standard_Integer nb2 = Intersector2.NbPoints(); 
				for(int j = 0;j < nb2;j++)
				{
					gp_Pnt P = Intersector2.Point(j+1); 
					CPoint3D interpoint(P.X(),P.Y(),P.Z());
					interPointList_.push_back(interpoint);
				}
			}
		}
	}
	if(interPointList_.size()>0)
		return 0;
	else
		return 1;
}

int InterProcess::SetInterPointList(Curve* C1, Curve* C2)
{
	vector<CPoint3D>().swap(interPointList_);
	Handle(Geom2d_Curve) HC1 = C1->ToOCCT();
	Handle(Geom2d_Curve) HC2 = C2->ToOCCT();

	Geom2dAPI_InterCurveCurve Intersector1(HC1,HC2,tolerance_);
	Standard_Integer n = Intersector1.NbPoints(); 
	gp_Pnt2d P;

	for(int i=0;i<n;i++)
	{
		P = Intersector1.Point(i+1); 
		CPoint3D interpoint(P.X(),P.Y(),0);
		interPointList_.push_back(interpoint);
	}
	if(interPointList_.size()>0)
		return 0;
	else
		return 1;
}
