#pragma once
#include "stdafx.h"
#include "SFace.h"
#include "Curve.h"


using namespace std;
class StepEntity;


class InterProcess
{
public:
	InterProcess(void);
	~InterProcess(void);
	int SetInterPointList(SFace* F1, SFace* F2, SFace* F3);//三维空间求交，生成交点，写进交点列表
	int SetInterPointList(Curve* C1,SFace* F1);//三维空间求交，生成交点，写进交点列表
	int SetInterPointList(Curve* C1, Curve* C2);//二维空间求交，生成交点，写进交点列表
public:
	vector<CPoint3D> interPointList_;  //交点链表
};
