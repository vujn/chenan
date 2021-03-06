// stdafx.h : 标准系统包含文件的包含文件，
// 或是经常使用但不常更改的
// 特定于项目的包含文件
//

#pragma once
#define WNT

#include "targetver.h"
#include <stp_schema.h>
#include <stix.h>
#include <stixmesh.h>
#include <stix_xform.h>
#include <iostream>
#include <vector>
#include <list>
#include <RoseString.h>
#include <map>
#include "GeomCalc.h"
#include <stdio.h>
#include <string>
#include <memory>
#include <tchar.h>
#include <stp_schema.h>
#include <stix.h>
#include <stixmesh.h>
#include <stix_xform.h>
#include <memory>
#include "RoseObject.h"
#include <RoseCursor.h>
#include "RoseDomain.h"
#include <roseHdefs.h>
#include <stix_asm.h>
#include "stp_closed_shell.h"
#include "stixmesh_nurbs.h"
#include <stp_axis1_placement.h>
#include <stp_advanced_brep_shape_representation.h>
#include <stp_advanced_face.h>
#include <stp_manifold_solid_brep.h>
#include <stp_face_bound.h>
#include <stp_path.h>
#include <stp_surface_curve.h>
#include "BRepToCSG.h"
#include "Standard_ErrorHandler.hxx" 
#include "Geom2d_Curve.hxx"
#include "gp_Ax2d.hxx"
#include "Geom_Surface.hxx"
#include "Geom_Plane.hxx"
#include "Geom_CylindricalSurface.hxx"
#include "Geom_sphericalSurface.hxx"
#include "Geom_ConicalSurface.hxx"
#include "Geom_ToroidalSurface.hxx"
#include "Geom_SurfaceOfLinearExtrusion.hxx"
#include "Geom_SurfaceOfRevolution.hxx"
#include "Geom_BezierSurface.hxx"
#include "Geom_BSplineSurface.hxx"
#include "Geom_RectangularTrimmedSurface.hxx"
#include "Geom_OffsetSurface.hxx"
#include "Geom2dAPI_InterCurveCurve.hxx"
#include "GeomAPI_IntCS.hxx"
#include "GeomAPI_IntSS.hxx"
#include "Geom2d_Line.hxx"
#include "Geom2d_Circle.hxx"
#include "Structure.h"
#include "Geom2d_Ellipse.hxx"
#include "Geom_BSplineCurve.hxx"
#include "Geom2d_BoundedCurve.hxx"
#include "Geom2d_BSplineCurve.hxx"
#include "TColgp_Array1OfPnt2d.hxx"
#include "TColStd_Array1OfReal.hxx"
#include "TColStd_Array1OfInteger.hxx"
#include "TColgp_Array2OfPnt.hxx"
#include "TColStd_Array2OfReal.hxx"
#include "TopoDS_Face.hxx"
#include "GeomTools.hxx"
#include "TopoDS_Shape.hxx"
#include "BRepBuilderAPI_MakeEdge.hxx"
#include "gp_Pln.hxx"
#include "TopoDS_Edge.hxx"
#include "TopoDS_Wire.hxx"
#include "BRepBuilderAPI_MakeWire.hxx"
#include "BRepBuilderAPI_MakeFace.hxx"
#include "BRepBuilderAPI_MakeShape.hxx"
#include "gp_Cylinder.hxx"
#include "gp_Sphere.hxx"
#include "gp_Cone.hxx"
#include "gp_Torus.hxx"
#include "BRep_Tool.hxx"
#include "TopExp_Explorer.hxx"
#include "TopoDS.hxx"
#include "BRep_Builder.hxx"
#include "gp_Circ.hxx"
#include "gp_Elips.hxx"
#include "BRepOffsetAPI_Sewing.hxx"
#include "TColStd_ListOfReal.hxx"
#include "BRepExtrema_DistShapeShape.hxx"
#include "TColStd_ListIteratorOfListOfReal.hxx"
#include "TopTools_ListIteratorOfListOfShape.hxx"
#include "ShapeFix_Solid.hxx"
#include "BRepAdaptor_Surface.hxx"
#include "CSLib_DerivativeStatus.hxx"
#include "CSLib_NormalStatus.hxx"
#include "GeomLib_Tool.hxx"
#include "CSLib.hxx"
#include "Bnd_Box.hxx"
#include "BRepBndLib.hxx"
#include "BRepAlgoAPI_Cut.hxx"
#include "BRepPrimAPI_MakeBox.hxx"
#include "Partition.h"
#include "BRepAlgoAPI_Section.hxx"
#include "BRepFeat_SplitShape.hxx"
#include "BRepAlgoAPI_Common.hxx"
#include "BRepPrimAPI_MakeCylinder.hxx"
// TODO: 在此处引用程序需要的其他头文件
