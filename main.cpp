
#include "stdafx.h"


std::string pathName;

void TCharToString(TCHAR* STR, string& pathName);
bool GetStepFileDialog();

int main() 
{   
	bool isOpen = GetStepFileDialog();
	if(!isOpen)
	{
		printf("���ļ�����!\n");
		exit(1);
	}

	Standard_Boolean failsonly = Standard_False;
	STEPControl_Reader reader; 
	IFSelect_ReturnStatus status = reader.ReadFile(pathName.c_str());
	reader.PrintCheckLoad(failsonly, IFSelect_ItemsByEntity);
	Standard_Integer NbRoots = reader.NbRootsForTransfer();  
	reader.PrintCheckTransfer(failsonly, IFSelect_ItemsByEntity);
	for (Standard_Integer n = 1; n <= NbRoots; n++)
	{
		Standard_Boolean ok = reader.TransferRoot(n);
	}
	Standard_Integer nbs = reader.NbShapes();
	if (nbs == 0) 
	{
		return IFSelect_RetVoid; 
	}
	for (Standard_Integer i = 1; i <= nbs; i++)
	{
		TopoDS_Shape test = reader.Shape(i);
	}
	Standard_Integer NbTrans = reader.TransferRoots();  
	TopoDS_Shape result = reader.OneShape();  

	Partition part(result);

	return 0;
} 


// void processEdge(const TopoDS_Edge& edge, const TopoDS_Face& face)
// {
// 	Standard_Real dTolerance = BRep_Tool::Tolerance(edge);
// 	
// 	Standard_Boolean bIsGeometric = BRep_Tool::IsGeometric(edge);
// 	Standard_Boolean bIsSameParameter = BRep_Tool::SameParameter(edge);
// 	Standard_Boolean bIsSameRange = BRep_Tool::SameRange(edge);
// 	Standard_Boolean bIsDegenerated = BRep_Tool::Degenerated(edge);
// 	Standard_Boolean bIsClosed = BRep_Tool::IsClosed(edge, face);
// 	
// 	TopAbs_Orientation nOrientation = edge.Orientation();
// 	
// 	// Dump edge info.
// 	std::cout << "====== Edge Info =======" << std::endl;
// 	std::cout << "Tolerance: " << dTolerance << std::endl;
// 	std::cout << "Orientation: " << dumpOrientation(nOrientation) << std::endl;
// 	std::cout << "Geometric: " << (bIsGeometric ? "True" : "False") << std::endl;
// 	std::cout << "Same Parameter: " << (bIsSameParameter ? "True" : "False") << std::endl;
// 	std::cout << "Same Range: " << (bIsSameRange ? "True" : "False") << std::endl;
// 	std::cout << "Degenerated edge: " << (bIsDegenerated ? "True" : "False") << std::endl;
// 	std::cout << "Seam edge: " << (bIsClosed ? "True" : "False") << std::endl;
// 	
// 	// Dump vertex of the edge.
// 	for(TopExp_Explorer vertexItr(edge, TopAbs_VERTEX);
// 	vertexItr.More();
// 	vertexItr.Next())
// 	{
// 		const TopoDS_Vertex& aVertex = TopoDS::Vertex(vertexItr.Current());
// 		gp_Pnt pnt = BRep_Tool::Pnt(aVertex);
// 
// 		std::cout << "Vertex: (" << pnt.X() << ", " << pnt.Y() << ", " << pnt.Z() << ")" << std::endl;
// 	}
// }

//int main(void)
// {
// 	Standard_Integer nSphereFaceCount = 0;
// 	Standard_Integer nSphereEdgeCount = 0;
// 
// 	TopoDS_Shape sphere = BRepPrimAPI_MakeSphere(1.0);
// 
// 	for(TopExp_Explorer faceItr(sphere, TopAbs_FACE);
// 		faceItr.More();
// 		faceItr.Next())
// 	{
// 		const TopoDS_Face& aFace = TopoDS::Face(faceItr.Current());
// 
// 		++nSphereFaceCount;
// 
// 		for(TopExp_Explorer edgeItr(aFace, TopAbs_EDGE);
// 			edgeItr.More();
// 			edgeItr.Next())
// 		{
// 			const TopoDS_Edge& aEdge = TopoDS::Edge(edgeItr.Current());
// 
// 			processEdge(aEdge, aFace);
// 
// 			++nSphereEdgeCount;
// 		}
// 	}
// 
// 	std::cout << "Sphere face count: " << nSphereFaceCount << std::endl;
// 	std::cout << "Sphere edge count: " << nSphereEdgeCount << std::endl;
// 
//    return 0;
// 
// void dumpVertex(const TopoDS_Vertex& vertex)
// {
// 	gp_Pnt pnt = BRep_Tool::Pnt(vertex);
// 
// 	std::cout << "(" << pnt.X() << ", " << pnt.Y() << ", " << pnt.Z() << ")" << std::endl;
// }
// 
// int main(void)
// {
// 	Standard_Integer nCount = 0;
// 	TopoDS_Shape aBox = BRepPrimAPI_MakeBox(100, 150, 200);
// 	
// 	TopTools_IndexedDataMapOfShapeListOfShape shapeMap;
// 	TopTools_ListOfShape edges;
// 	TopTools_ListIteratorOfListOfShape edgeItr;
// 	
// 	// Use TopExp_Explorer to access subshapes.
// 	TopExp_Explorer vertexExp(aBox, TopAbs_VERTEX);
// 	
// 	const TopoDS_Vertex& aVertex = TopoDS::Vertex(vertexExp.Current());
// 
// 	// Use TopExp::MapShapesAndAncestors() to access parent shapes.
// 	TopExp::MapShapesAndAncestors(aBox, TopAbs_VERTEX, TopAbs_EDGE, shapeMap);
// 	
// 	edges = shapeMap.FindFromKey(aVertex);
// 	
// 	dumpVertex(aVertex);
// 	
// 	for(edgeItr.Initialize(edges); edgeItr.More(); edgeItr.Next())
// 	{
// 		const TopoDS_Edge& anEdge = TopoDS::Edge(edgeItr.Value());
// 
// 		std::cout << "Vertex belong to the Edge: " << std::endl;
// 		dumpVertex(TopExp::FirstVertex(anEdge));
// 		dumpVertex(TopExp::LastVertex(anEdge));
// 		std::cout << "---------------------------" << std::endl;
// 	}
// 	
// 	return 0;
// }

// void _tmain(int argc, _TCHAR* argv[])
// {
// 	bool isOpen = GetStepFileDialog();
// 	if(!isOpen)
// 	{
// 		printf("���ļ�����!\n");
// 		exit(1);
// 	}
// 	ROSE.quiet(1);	// console show;
// 	stplib_init();	// initialize merged cad library
// 	stixmesh_init();
// 	RoseDesign* design = ROSE.findDesign(pathName.c_str());
// 	
// 	if (!design)
// 	{
// 		printf("Could not open STEP file %s\n", pathName.c_str());
// 		exit(1);
// 	}
// 	rose_compute_backptrs(design);
// 	stix_tag_asms(design);
// 
// 	BRepToCSG csg(design);
// 	system("pause");
// }

bool GetStepFileDialog()
{
	TCHAR szBuffer[MAX_PATH] = { 0 };
	OPENFILENAME ofn = { 0 };
	ofn.lStructSize = sizeof(ofn);
	ofn.hwndOwner = nullptr;
	ofn.lpstrFilter = _T("�����ļ�*.*)\0*.*\0");
	ofn.lpstrInitialDir = _T("C:\\Users\\hekaifa\\Desktop\\Test Step");
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
