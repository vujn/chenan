
#include "stdafx.h"
#include "SFace.h"
#include <Geom_ToroidalSurface.hxx>


//////////////////////////////////////////////////////////////////////
// FACE
//////////////////////////////////////////////////////////////////////

SFace::SFace()
{
	position_ = NULL;
}

SFace::~SFace()
{

}

void SFace::GenerateCoefficient()
{

}

Geom_Surface * SFace::ToOCCT()
{
	return nullptr;
}

PointPosition SFace::PointIsIn(CPoint3D TestPoint)
{
	PointPosition Position;
	//系数数组，二次曲面的一般方程为Ax^2+2Bxy+2Cxz+2Dx+Ey^2+2Fyz+2Gy+Hz^2+2Iz+J = 0，共10个系数
	if(IS_NEGATIVE(coefficient_[0] * TestPoint.x*TestPoint.x
		+ 2.0*coefficient_[1] * TestPoint.x*TestPoint.y
		+ 2.0*coefficient_[2] * TestPoint.x*TestPoint.z
		+ 2.0*coefficient_[3] * TestPoint.x
		+ coefficient_[4] * TestPoint.y*TestPoint.y
		+ 2.0*coefficient_[5] * TestPoint.y*TestPoint.z
		+ 2.0*coefficient_[6] * TestPoint.y
		+ coefficient_[7] * TestPoint.z*TestPoint.z
		+ 2.0*coefficient_[8] * TestPoint.z
		+ coefficient_[9]))
	{
		Position = IN_FACE;
		if(!strcmp(this->name_, "conical_surface"))
		{
			CPoint3D vertex = ((SConical*)this)->vertex_;
			CVector3D pVector3D1 = ((SConical*)this)->position_->verAxis;
			CVector3D pVector3D2(TestPoint.x - vertex.x, TestPoint.y - vertex.y, TestPoint.z - vertex.z);
			if(IS_NEGATIVE((pVector3D1 | pVector3D2)))
				Position = OUT_FACE;
		}
	}
	else if(IS_ZERO(coefficient_[0] * TestPoint.x*TestPoint.x
		+ 2.0*coefficient_[1] * TestPoint.x*TestPoint.y
		+ 2.0*coefficient_[2] * TestPoint.x*TestPoint.z
		+ 2.0*coefficient_[3] * TestPoint.x
		+ coefficient_[4] * TestPoint.y*TestPoint.y
		+ 2.0*coefficient_[5] * TestPoint.y*TestPoint.z
		+ 2.0*coefficient_[6] * TestPoint.y
		+ coefficient_[7] * TestPoint.z*TestPoint.z
		+ 2.0*coefficient_[8] * TestPoint.z
		+ coefficient_[9]))
	{
		Position = ON_FACE;
		if(!strcmp(this->name_, "conical_surface"))
		{
			CPoint3D vertex = ((SConical*)this)->vertex_;
			CVector3D pVector3D1 = ((SConical*)this)->position_->verAxis;
			CVector3D pVector3D2(TestPoint.x - vertex.x, TestPoint.y - vertex.y, TestPoint.z - vertex.z);
			if(IS_NEGATIVE((pVector3D1 | pVector3D2)))
				Position = OUT_FACE;
		}
	}
	else
		Position = OUT_FACE;
	return Position;
}

//////////////////////////////////////////////////////////////////////
// SPlane
//////////////////////////////////////////////////////////////////////

SPlane::SPlane()
{

}

SPlane::~SPlane()
{

}


void SPlane::GenerateCoefficient()
{
	//二次曲面的一般方程为Ax^2+2Bxy+2Cxz+2Dx+Ey^2+2Fyz+2Gy+Hz^2+2Iz+J = 0，共10个系数
	coefficient_[0] = 0.0;  //A
	coefficient_[1] = 0.0;  //B
	coefficient_[2] = 0.0;  //C
	coefficient_[3] = position_->verAxis.dx / 2.0;  //D
	coefficient_[4] = 0.0;  //E
	coefficient_[5] = 0.0;  //F
	coefficient_[6] = position_->verAxis.dy / 2.0;  //G
	coefficient_[7] = 0.0;  //H
	coefficient_[8] = position_->verAxis.dz / 2.0;  //I
	coefficient_[9] = - position_->verAxis.dx * position_->point.x
		- position_->verAxis.dy * position_->point.y
		- position_->verAxis.dz * position_->point.z;  //J
}

Geom_Surface * SPlane::ToOCCT()
{
	Geom_Plane* plane = new Geom_Plane(
		coefficient_[3] * 2.0, coefficient_[6] * 2.0, coefficient_[8] * 2.0, coefficient_[9]);
	return plane;
}

//////////////////////////////////////////////////////////////////////
// SCylindrical 圆柱
//////////////////////////////////////////////////////////////////////

SCylindrical::SCylindrical()
{

}

SCylindrical::~SCylindrical()
{

}


void SCylindrical::GenerateCoefficient()
{
	//二次曲面的一般方程为Ax^2+2Bxy+2Cxz+2Dx+Ey^2+2Fyz+2Gy+Hz^2+2Iz+J = 0，共10个系数

	CVector3D cV1(position_->verAxis.dx, position_->verAxis.dy, position_->verAxis.dz);
	cV1.Normalize();
	CPoint3D cP1(position_->point.x, position_->verAxis.dy, position_->verAxis.dz);
	coefficient_[0] = cV1.dz * cV1.dz + cV1.dy * cV1.dy;  //A
	coefficient_[1] = -cV1.dx * cV1.dy;  //B
	coefficient_[2] = -cV1.dx * cV1.dz;  //C
	coefficient_[3] = -1.0 * (cV1.dz*cV1.dz + cV1.dy*cV1.dy) * cP1.x + cV1.dx*cV1.dy*cP1.y + cV1.dx*cV1.dz*cP1.z;  //D
	coefficient_[4] = cV1.dz*cV1.dz + cV1.dx*cV1.dx;  //E
	coefficient_[5] = -cV1.dy*cV1.dz;  //F
	coefficient_[6] = -1.0 * (cV1.dz*cV1.dz + cV1.dx*cV1.dx) * cP1.y + cV1.dx*cV1.dy*cP1.x + cV1.dy*cV1.dz*cP1.z;  //G
	coefficient_[7] = cV1.dy*cV1.dy + cV1.dx*cV1.dx;  //H
	coefficient_[8] = -1.0 * (cV1.dx*cV1.dx + cV1.dy*cV1.dy) * cP1.z + cV1.dz*cV1.dy*cP1.y + cV1.dx*cV1.dz*cP1.x;  //I
	coefficient_[9] = -2.0 * cV1.dy*cV1.dz * cP1.y*cP1.z - 2.0 * cV1.dy*cV1.dx * cP1.y*cP1.x - 2.0 * cV1.dx*cV1.dz * cP1.x*cP1.z
		+ (cV1.dz*cV1.dz + cV1.dy*cV1.dy) * cP1.x*cP1.x
		+ (cV1.dz*cV1.dz + cV1.dx*cV1.dx) * cP1.y*cP1.y
		+ (cV1.dx*cV1.dx + cV1.dy*cV1.dy) * cP1.z*cP1.z
		- radius_ * radius_;   //J
}

Geom_Surface * SCylindrical::ToOCCT()
{
	gp_Pnt P1(position_->point.x, position_->point.y, position_->point.z);
	gp_Vec Vec1(position_->verAxis.dx, position_->verAxis.dy, position_->verAxis.dz);
	gp_Dir Dir1(Vec1);
	gp_Ax3 Ax31(P1, Dir1);
	Geom_CylindricalSurface * Cyl1 = new Geom_CylindricalSurface(Ax31, radius_);
	return Cyl1;
}

//////////////////////////////////////////////////////////////////////
// SSpherical 球面
//////////////////////////////////////////////////////////////////////

SSpherical::SSpherical()
{

}

SSpherical::~SSpherical()
{

}

void SSpherical::GenerateCoefficient()
{
	//二次曲面的一般方程为Ax^2+2Bxy+2Cxz+2Dx+Ey^2+2Fyz+2Gy+Hz^2+2Iz+J = 0，共10个系数
	coefficient_[0] = 1.0;  //A
	coefficient_[1] = 0.0;  //B
	coefficient_[2] = 0.0;  //C
	coefficient_[3] = -position_->point.x;  //D
	coefficient_[4] = 1.0;  //E
	coefficient_[5] = 0.0;  //F
	coefficient_[6] = -position_->point.y;  //G
	coefficient_[7] = 1.0;  //H
	coefficient_[8] = -position_->point.z;  //I
	coefficient_[9] = pow(position_->point.x, 2.0)
		+ pow(position_->point.y, 2.0)
		+ pow(position_->point.z, 2.0)
		- radius_ * radius_;  //J
}

Geom_Surface * SSpherical::ToOCCT()
{
	gp_Pnt P1(position_->point.x, position_->point.y, position_->point.z);
	gp_Vec Vec1(position_->verAxis.dx, position_->verAxis.dy, position_->verAxis.dz);
	gp_Dir Dir1(Vec1);
	gp_Ax3 Ax31(P1, Dir1);
	Geom_SphericalSurface* spherical = new Geom_SphericalSurface(Ax31, radius_);
	return spherical;
}

//////////////////////////////////////////////////////////////////////
// SConical 圆锥
//////////////////////////////////////////////////////////////////////

SConical::SConical()
{

}

SConical::~SConical()
{

}

void SConical::GenerateCoefficient()
{
	//二次曲面的一般方程为Ax^2+2Bxy+2Cxz+2Dx+Ey^2+2Fyz+2Gy+Hz^2+2Iz+J = 0，共10个系数

	CVector3D cV1(position_->verAxis.dx, position_->verAxis.dy, position_->verAxis.dz);
	cV1.Normalize();
	CPoint3D cP1(position_->point.x, position_->point.y, position_->point.z);

	vertex_[0] = cP1.x - cV1.dx * radius_*(1/tan(semi_angle_));
	vertex_[1] = cP1.y - cV1.dy * radius_*(1/tan(semi_angle_));
	vertex_[2] = cP1.z - cV1.dz * radius_*(1/tan(semi_angle_));

	double M = radius_*(1/tan(semi_angle_)) - cV1.dx*cP1.x - cV1.dy*cP1.y - cV1.dz*cP1.z;

	coefficient_[0] = cV1.dz*cV1.dz + cV1.dy*cV1.dy - cV1.dx*cV1.dx*tan(semi_angle_);  //A
	coefficient_[1] = -cV1.dx*cV1.dy - cV1.dx*cV1.dy*tan(semi_angle_);  //B
	coefficient_[2] = -cV1.dx*cV1.dz - cV1.dx*cV1.dz*tan(semi_angle_);  //C
	coefficient_[3] = -1.0 * (cV1.dz*cV1.dz + cV1.dy*cV1.dy) * cP1.x 
		+ cV1.dx*cV1.dy*cP1.y 
		+ cV1.dx*cV1.dz*cP1.z 
		- M*tan(semi_angle_)*cV1.dx;  //D
	coefficient_[4] = cV1.dz*cV1.dz + cV1.dx*cV1.dx - cV1.dy*cV1.dy*tan(semi_angle_);  //E
	coefficient_[5] = -cV1.dy*cV1.dz - cV1.dy*cV1.dz*tan(semi_angle_);  //F
	coefficient_[6] = -1.0 * (cV1.dz*cV1.dz + cV1.dx*cV1.dx) * cP1.y 
		+ cV1.dx*cV1.dy*cP1.x
		+ cV1.dy*cV1.dz*cP1.z 
		- M*tan(semi_angle_)*cV1.dy;  //G
	coefficient_[7] = cV1.dy*cV1.dy + cV1.dx*cV1.dx - cV1.dz*cV1.dz*tan(semi_angle_);  //H
	coefficient_[8] = -1.0 * (cV1.dx*cV1.dx + cV1.dy*cV1.dy) * cP1.z 
		+ cV1.dz*cV1.dy*cP1.y 
		+ cV1.dx*cV1.dz*cP1.x
		- M*tan(semi_angle_)*cV1.dz;  //I
	coefficient_[9] = -2.0 * cV1.dy*cV1.dz * cP1.y*cP1.z 
		- 2.0 * cV1.dy*cV1.dx * cP1.y*cP1.x 
		- 2.0 * cV1.dx*cV1.dz * cP1.x*cP1.z
		+ (cV1.dz*cV1.dz + cV1.dy*cV1.dy) * cP1.x*cP1.x
		+ (cV1.dz*cV1.dz + cV1.dx*cV1.dx) * cP1.y*cP1.y
		+ (cV1.dx*cV1.dx + cV1.dy*cV1.dy) * cP1.z*cP1.z
		- M*M*tan(semi_angle_);   //J
}

Geom_Surface * SConical::ToOCCT()
{
	gp_Pnt P1(position_->point.x, position_->point.y, position_->point.z);
	gp_Vec Vec1(position_->verAxis.dx, position_->verAxis.dy, position_->verAxis.dz);
	gp_Dir Dir1(Vec1);
	gp_Ax3 Ax31(P1, Dir1);
	Geom_ConicalSurface* conical = new Geom_ConicalSurface(Ax31, semi_angle_, radius_);
	return conical;
}

//////////////////////////////////////////////////////////////////////
// SToroidal 圆环
//////////////////////////////////////////////////////////////////////

SToroidal::SToroidal()
{

}

SToroidal::~SToroidal()
{

}

void SToroidal::GenerateCoefficient()
{

}

Geom_Surface * SToroidal::ToOCCT()
{
	gp_Pnt P1(position_->point.x, position_->point.y, position_->point.z);
	gp_Vec Vec1(position_->verAxis.dx, position_->verAxis.dy, position_->verAxis.dz);
	gp_Dir Dir1(Vec1);
	gp_Ax3 Ax31(P1, Dir1);
	Geom_ToroidalSurface* toroidal = new Geom_ToroidalSurface(Ax31, major_radius_, minor_radius_);
	return toroidal;
}

