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
	int SetInterPointList(SFace* F1, SFace* F2, SFace* F3);//��ά�ռ��󽻣����ɽ��㣬д�������б�
	int SetInterPointList(Curve* C1,SFace* F1);//��ά�ռ��󽻣����ɽ��㣬д�������б�
	int SetInterPointList(Curve* C1, Curve* C2);//��ά�ռ��󽻣����ɽ��㣬д�������б�
public:
	vector<CPoint3D> interPointList_;  //��������
};
