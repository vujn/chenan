
#include "stdafx.h"
#include "InfoMatrix.h"
#include "StepEntity.h"
#include <commdlg.h>
#include <shlwapi.h>
#include <map>
#include <stplib_init.h>


std::string pathName;

void TCharToString(TCHAR* STR, string& pathName);
bool GetStepFileDialog();


void _tmain(int argc, _TCHAR* argv[])
{
	bool isOpen = GetStepFileDialog();
	if(!isOpen)
	{
		printf("���ļ�����!\n");
		exit(1);
	}
	ROSE.quiet(1);	// console show;
	stplib_init();	// initialize merged cad library
	stixmesh_init();
	RoseDesign* design = ROSE.findDesign(pathName.c_str());
	//RoseDesign* design = ROSE.findDesign("11.STEP"); 

	if (!design)
	{
		printf("Could not open STEP file %s\n", pathName.c_str());
		exit(1);
	}
	rose_compute_backptrs(design);
	stix_tag_asms(design);

	StepEntity* step = new StepEntity(design);

}

bool GetStepFileDialog()
{
	TCHAR szBuffer[MAX_PATH] = { 0 };
	OPENFILENAME ofn = { 0 };
	ofn.lStructSize = sizeof(ofn);
	ofn.hwndOwner = nullptr;
	ofn.lpstrFilter = _T("stp�ļ�(*.stp)\0*.stp\0step�ļ�(*.step)\0*.step\0�����ļ�*.*)\0*.*\0");
	ofn.lpstrInitialDir = _T("D:\\stepmod");
	ofn.lpstrFile = szBuffer;
	ofn.nMaxFile = sizeof(szBuffer) / sizeof(*szBuffer);
	ofn.nFilterIndex = 0;
	ofn.Flags = OFN_PATHMUSTEXIST | OFN_FILEMUSTEXIST | OFN_EXPLORER;
	BOOL bSel = GetOpenFileName(&ofn);
	TCharToString(szBuffer, pathName);
	if(0 == _tcslen(szBuffer))
		return false;
	return true;
}

void TCharToString(TCHAR* STR, string& pathName)
{
	int iLen = WideCharToMultiByte(CP_ACP, 0, STR, -1, NULL, 0, NULL, NULL);
	char* chRtn = new char[iLen*sizeof(char)];
	WideCharToMultiByte(CP_ACP, 0, STR, -1, chRtn, iLen, NULL, NULL);
	pathName = chRtn;
}
