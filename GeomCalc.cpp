#include "stdafx.h"
#include "GeomCalc.h"
#include "math.h"

double _AngleBetween(VECTOR3D v1, VECTOR3D v2)					//retrun 0 to 180 degree
{
	CVector3D cv1(v1), cv2(v2);
	cv1.Normalize();
	cv2.Normalize();
	return acos(max(-1.0, min(1.0, cv1 | cv2)));
}

double _AngleBetween(VECTOR3D v1, VECTOR3D v2, VECTOR3D normal)	//if 0, return 360
{
	CVector3D cv1(v1), cv2(v2), norm(normal);
	// 	if ( _IsParallel(cv1,cv2) )
	// 	{
	// 		if ((cv1|cv2) <= 0)
	// 		{
	// 			return 0;
	// 		}
	// 	}
	cv1.Normalize();
	cv2.Normalize();
	if ( ( ( norm*cv1 ) | cv2 ) > 0 )
		return acos(max(-1.0, min(1.0, cv1 | cv2)));
	else
		return 2 * PI - acos(max(-1.0, min(1.0, cv1 | cv2)));

}

double _DistOf(POINT3D pt0, POINT3D pt1)
{
	CVector3D v(pt1.x - pt0.x, pt1.y - pt0.y, pt1.z - pt0.z);
	return v.GetLength();
}

BOOL _IsParallel(VECTOR3D v1, VECTOR3D v2)
{
	CVector3D cv1(v1), cv2(v2);
	return IS_ZERO(( cv1*cv2 ) | ( cv1*cv2 ));
}

BOOL _IsOrthogonal(VECTOR3D v1, VECTOR3D v2)
{
	CVector3D cv1(v1), cv2(v2);
	return IS_ZERO(cv1 | cv2);
}

BOOL _IsInTheSameFace(CPoint3D pt1, CPoint3D pt2, CPoint3D pt3, CPoint3D pt4)
{
	CVector3D v1 = pt1 - pt2;
	CVector3D v2 = pt2 - pt3;
	CVector3D v3 = pt3 - pt4;
	if ( IS_ZERO(( ( v1*v2 ) * ( v2*v3 ) ) | ( ( v1*v2 ) * ( v2*v3 ) )) )
		return TRUE;
	else
		return FALSE;
}

BOOL _Intersection(CPoint3D pt1, CPoint3D pt2, CPoint3D pt3, CPoint3D pt4, CPoint3D* intersection/* = NULL*/)
{
	CPoint3D tmp;
	CVector3D v1 = pt1 - pt2;
	CVector3D v2 = pt3 - pt4;

	double dx1 = pt2.x - pt1.x;
	double dy1 = pt2.y - pt1.y;
	double dz1 = pt2.z - pt1.z;

	double dx3 = pt4.x - pt3.x;
	double dy3 = pt4.y - pt3.y;
	double dz3 = pt4.z - pt3.z;

	if ( _IsParallel(v1, v2) )
		return FALSE;
	if ( !_IsInTheSameFace(pt1, pt2, pt3, pt4) )
		return FALSE;
	//in a plane and not parallel
	//calculation x
	if ( IS_ZERO(dx1) )
		tmp.x = pt1.x;
	else
	{
		if ( IS_ZERO(dx3) )
		{
			tmp.x = pt3.x;
		}
		else
		{
			if ( !IS_ZERO(dx3*dy1 - dx1*dy3) )
			{
				tmp.x = ( dx1*dx3*( pt3.y - pt1.y ) + dx3*dy1*pt1.x - dx1*dy3*pt3.x ) / ( dx3*dy1 - dx1*dy3 );
			}
			else
			{
				tmp.x = ( dx1*dx3*( pt3.z - pt1.z ) + dx3*dz1*pt1.x - dx1*dz3*pt3.x ) / ( dx3*dz1 - dx1*dz3 );
			}
		}
	}
	//cal y
	if ( IS_ZERO(dy1) )
	{
		tmp.y = pt1.y;
	}
	else
	{
		if ( IS_ZERO(dy3) )
		{
			tmp.y = pt3.y;
		}
		else
		{
			if ( !IS_ZERO(dy3*dz1 - dy1*dz3) )
			{
				tmp.y = ( dy1*dy3*( pt3.z - pt1.z ) + dy3*dz1*pt1.y - dy1*dz3*pt3.y ) / ( dy3*dz1 - dy1*dz3 );
			}
			else
			{
				tmp.y = ( dy1*dy3*( pt3.x - pt1.x ) + dy3*dx1*pt1.y - dy1*dx3*pt3.y ) / ( dy3*dx1 - dy1*dx3 );
			}
		}
	}
	//cal z
	if ( IS_ZERO(dz1) )
	{
		tmp.z = pt1.z;
	}
	else
	{
		if ( IS_ZERO(dz3) )
		{
			tmp.z = pt3.z;
		}
		else
		{
			if ( !IS_ZERO(dz3*dx1 - dz1*dx3) )
			{
				tmp.z = ( dz1*dz3*( pt3.x - pt1.x ) + dz3*dx1*pt1.z - dz1*dx3*pt3.z ) / ( dz3*dx1 - dz1*dx3 );
			}
			else
			{
				tmp.z = ( dz1*dz3*( pt3.y - pt1.y ) + dz3*dy1*pt1.z - dz1*dy3*pt3.z ) / ( dz3*dy1 - dz1*dy3 );
			}
		}
	}

	if ( intersection != NULL )
		*intersection = tmp;

	if ( tmp == pt1 || tmp == pt2 || tmp == pt3 || tmp == pt4 )
		return FALSE;

	if ( ( ( tmp.x <= max(pt1.x, pt2.x) ) && ( tmp.x >= min(pt1.x, pt2.x) ) &&
		( tmp.y <= max(pt1.y, pt2.y) ) && ( tmp.y >= min(pt1.y, pt2.y) ) &&
		( tmp.z <= max(pt1.z, pt2.z) ) && ( tmp.z >= min(pt1.z, pt2.z) ) ) &&
		( ( tmp.x <= max(pt3.x, pt4.x) ) && ( tmp.x >= min(pt3.x, pt4.x) ) &&
		( tmp.y <= max(pt3.y, pt4.y) ) && ( tmp.y >= min(pt3.y, pt4.y) ) &&
		( tmp.z <= max(pt3.z, pt4.z) ) && ( tmp.z >= min(pt3.z, pt4.z) ) ) )
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

double GetMax(double a, double b)
{
	return a > b ? a : b;
}

double GetMin(double a, double b)
{
	return a < b ? a : b;
}

/************************************************************************/
/* CPoint3D                                                                     */
/************************************************************************/
//constructor & destructor
CPoint3D::CPoint3D()
{
	x = y = z = 0.0;
}

CPoint3D::CPoint3D( double ix, double iy, double iz/* =0.0 */)
{
	x = ix;
	y = iy;
	z = iz;
}

CPoint3D::CPoint3D( const double* p )
{
	x = p[0];
	y = p[1];
	z = p[2];
}

CPoint3D::CPoint3D( POINT3D p )
{
	x = p.x;
	y = p.y;
	z = p.z;
}

CPoint3D::~CPoint3D()
{
}

//operators
//modified by wangxin 
CPoint3D CPoint3D::operator*( const MATRIX3D& matrix ) const
{
	double rx, ry, rz, sc;
	rx = x*matrix.a[0] + y*matrix.a[4] + z*matrix.a[8] + matrix.a[12];
	ry = x*matrix.a[1] + y*matrix.a[5] + z*matrix.a[9] + matrix.a[13];
	rz = x*matrix.a[2] + y*matrix.a[6] + z*matrix.a[10] + matrix.a[14];
	sc = x*matrix.a[3] + y*matrix.a[7] + z*matrix.a[11] + matrix.a[15];
	if(sc==0)
		sc=1;
	rx /= sc;
	ry /= sc;
	rz /= sc;
	return CPoint3D( rx, ry, rz );
}

void CPoint3D::operator*=( const MATRIX3D& matrix )
{
	(*this) = (*this) * matrix;
}

CPoint3D CPoint3D::operator+( VECTOR3D v ) const
{
	return CPoint3D( x+v.dx, y+v.dy, z+v.dz );
}

void CPoint3D::operator+=( VECTOR3D v )

{
	x += v.dx;
	y += v.dy;
	z += v.dz;
}

CPoint3D CPoint3D::operator-( VECTOR3D v ) const
{
	return CPoint3D( x-v.dx, y-v.dy, z-v.dz );
}

void CPoint3D::operator-=( VECTOR3D v )
{
	x -= v.dx;
	y -= v.dy;
	z -= v.dz;
}

//derived vector = this point -sp
CVector3D CPoint3D::operator-( POINT3D sp ) const
{
	return CVector3D( x-sp.x, y-sp.y, z-sp.z );
}

BOOL CPoint3D::operator==( POINT3D pos ) const
{
	CVector3D vect( x-pos.x, y-pos.y, z-pos.z );

	if ( IS_ZERO( vect.GetLength() ) )
		return TRUE;
	else
		return FALSE;
}

BOOL CPoint3D::operator!=( POINT3D pos ) const
{
	CVector3D vect( x-pos.x, y-pos.y, z-pos.z );

	if ( IS_ZERO( vect.GetLength() ) )
		return FALSE;
	else
		return TRUE;
}

CPoint3D CPoint3D::operator/(double d) const
{
	return CPoint3D(x / d, y / d, z / d);
}

void CPoint3D::operator/=(double d)
{
	x /= d;
	y /= d;
	z /= d;
}


BOOL CPoint3D::IsAtTheLeftOf(CPoint3D p1, CPoint3D p2, CVector3D norm) const
{
	CVector3D v, v1;
	v = p2 - p1;
	v1 = *this - p1;
	double a = ((norm*v) | v1);
	if (fabs(a) < 0.00001)		//on the line
	{
		return 0;
	}
	else
	{
		if (a > 0)				//line left
			return 1;
		else					//line right
			return -1;
		return 0;
	}
}
 // if the point is on the left of the triangle, then it is in the triangle
BOOL CPoint3D::IsInTheTriangle(CPoint3D p1, CPoint3D p2, CPoint3D p3) const
{
	CVector3D normal = (p2-p1)*(p3-p2);
	normal.Normalize();
	if (	this->IsAtTheLeftOf(p1, p2, normal)==0 ||
			this->IsAtTheLeftOf(p2, p3, normal)==0 ||
			this->IsAtTheLeftOf(p3, p1, normal)==0)
	{
		return 0;				//one the triangle's side
	}
	else
	{
		if (	this->IsAtTheLeftOf(p1, p2, normal)==1 &&
				this->IsAtTheLeftOf(p2, p3, normal)==1 &&
				this->IsAtTheLeftOf(p3, p1, normal)==1	)
		{
			return 1;			//in the triangle
		}
		else
			return -1;			//out the triangle
	}
}

/************************************************************************/
/* CVector3D                                                                     */
/************************************************************************/
CVector3D::CVector3D()
{
	dx = dy = dz = 0;
}

CVector3D::CVector3D( double ix, double iy, double iz/* =0.0 */ )
{
	dx = ix;
	dy = iy;
	dz = iz;
}

CVector3D::CVector3D( const double* pv)
{
	dx = pv[0];
	dy = pv[1];
	dz = pv[2];
}

CVector3D::CVector3D( VECTOR3D v )
{
	dx = v.dx;
	dy = v.dy;
	dz = v.dz;
}

CVector3D::~CVector3D()
{
}

//operator
CVector3D CVector3D::operator+( VECTOR3D v ) const
{
	return CVector3D( dx+v.dx, dy+v.dy, dz+v.dz );
}

void CVector3D::operator+=( VECTOR3D v )
{
	dx += v.dx;
	dy += v.dy;
	dz += v.dz;
}

CVector3D CVector3D::operator-( VECTOR3D v ) const
{
	return CVector3D( dx-v.dx, dy-v.dy, dz-v.dz );
}

void CVector3D::operator-=( VECTOR3D v )
{
	dx -= v.dx;
	dy -= v.dy;
	dz -= v.dz;
}

CVector3D CVector3D::operator*( double d ) const
{
	return CVector3D( dx*d, dy*d, dz*d );
}

void CVector3D::operator*=( double d )
{
	dx *= d;
	dy *= d;
	dz *= d;
}

CVector3D CVector3D::operator/( double d ) const
{
	return CVector3D( dx/d, dy/d, dz/d );
}

void CVector3D::operator/=( double d )
{
	dx /= d;
	dy /= d;
	dz /= d;
}

//vector cross product
//|vector c|=|vector a×vector b|=|a||b|sin<a,b> 

//if vector a=(a1,b1,c1)，vector b=(a2,b2,c2)， 
//vector a×vector b= 
//| i j k | 
//|a1 b1 c1| 
//|a2 b2 c2| 
//=(b1c2-b2c1,c1a2-a1c2,a1b2-a2b1) 
CVector3D CVector3D::operator*( VECTOR3D v ) const
{
	return CVector3D( dy*v.dz-v.dy*dz, dz*v.dx-dx*v.dz, dx*v.dy-dy*v.dx );
}

//dot product
//vector a·vector b=|a||b|cos<a,b> 
//if vector a=(a1,b1,c1)，vector b=(a2,b2,c2)， 
//vector a·vector b=a1a2+b1b2+c1c2 
double CVector3D::operator|( VECTOR3D v ) const
{
	return dx*v.dx+dy*v.dy+dz*v.dz;
}

//modified by wangxin 
CVector3D CVector3D::operator*( const MATRIX3D& matrix ) const
{
	double rx, ry, rz, sc;
	rx = dx*matrix.a[0] + dy*matrix.a[4] + dz*matrix.a[8];
	ry = dx*matrix.a[1] + dy*matrix.a[5] + dz*matrix.a[9];
	rz = dx*matrix.a[2] + dy*matrix.a[6] + dz*matrix.a[10];
	sc = dx*matrix.a[3] + dy*matrix.a[7] + dz*matrix.a[11];
	if(sc==0)
		sc=1;
	rx /= sc;
	ry /= sc;
	rz /= sc;
	return CVector3D( rx, ry, rz );
}

void CVector3D::operator*=( const MATRIX3D& matrix )
{
	(*this) = (*this) * matrix;
}

//相等判断
BOOL CVector3D::operator==( VECTOR3D v ) const
{
	CVector3D vect( dx-v.dx, dy-v.dy, dz-v.dz );

	if ( IS_ZERO( vect.GetLength() ) )
	{
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

BOOL CVector3D::operator!=( VECTOR3D v ) const
{
	CVector3D vect( dx-v.dx, dy-v.dy, dz-v.dz );

	if ( IS_ZERO( vect.GetLength() ) )
		return FALSE;
	else
		return TRUE;
}

//length
double CVector3D::GetLength() const
{
	return sqrt( dx*dx+dy*dy+dz*dz );
}

double CVector3D::GetLengthXY() const	//projection on plane XY
{
	return sqrt( dx*dx+dy*dy );
}

double CVector3D::GetLengthYZ() const
{
	return sqrt( dy*dy+dz*dz );
}

double CVector3D::GetLengthZX() const
{
	return sqrt( dx*dx+dz*dz );
}

CVector3D CVector3D::GetNormal() const
{
	double len = GetLength();
	return CVector3D( dx/len, dy/len, dz/len );
}


CVector3D CVector3D::GetVerticalVector(int m) const
{

	if(!IS_ZERO(dx))
		return CVector3D(-(m*dy+dz)/dx,m,1);
	else
		if(!IS_ZERO(dy))
			return CVector3D(1,-(dx+m*dz)/dy,m);
		else
			if(!IS_ZERO(dz))
				return CVector3D(m,1,-(m*dx+dy)/dz);
			else
				return CVector3D(0,0,0);

}

void CVector3D::GenerateTFace(double * coefficient,CPoint3D p,int m) const
{

	CVector3D pVector3D1 = GetVerticalVector(m);
	if(pVector3D1.dx==0&&pVector3D1.dy==0&&pVector3D1.dz==0)
	{
		printf("出现零向量，错误！");
		exit(1);
	}
	coefficient[0]=0.0;  //A
	coefficient[1]=0.0;  //B
	coefficient[2]=0.0;  //C
	coefficient[3]=pVector3D1.dx/2.0;  //D
	coefficient[4]=0.0;  //E
	coefficient[5]=0.0;  //F
	coefficient[6]=pVector3D1.dy/2.0;  //G
	coefficient[7]=0.0;  //H
	coefficient[8]=pVector3D1.dz/2.0;  //I
	coefficient[9]= - pVector3D1.dx*p.x - pVector3D1.dy*p.y - pVector3D1.dz*p.z;  //J
}

void CVector3D::GenerateVFace(double * coefficient,CPoint3D p) const
{
	coefficient[0] = 0.0;  //A
	coefficient[1]=0.0;  //B
	coefficient[2]=0.0;  //C
	coefficient[3]=dx/2.0;  //D
	coefficient[4]=0.0;  //E
	coefficient[5]=0.0;  //F
	coefficient[6]=dy/2.0;  //G
	coefficient[7]=0.0;  //H
	coefficient[8]=dz/2.0;  //I
	coefficient[9]= - dx*p.x - dy*p.y - dz*p.z;  //J
}

void CVector3D::Normalize()
{
	double len = GetLength();
	if(!IS_ZERO(len))
	{
		dx /= len;
		dy /= len;
		dz /= len;
	}
}

BOOL CVector3D::IsZeroLength() const
{
	return IS_ZERO(GetLength());
}
/************************************************************************/
/* CPlane3D                                                                     */
/************************************************************************/
CPlane3D::CPlane3D( CPoint3D F_P1,CPoint3D F_P2,CPoint3D F_P3,CPoint3D F_P4)
{
	CVector3D a=F_P1-F_P2;
	CVector3D b=F_P1-F_P3;
	Normal=a*b;
	Normal.Normalize();
	Distance = -Normal.dx*F_P4.x - Normal.dy*F_P4.y - Normal.dz*F_P4.z;
}

double CPlane3D::SideTest( CPoint3D F_TestPoint )
{
	double SideValue = Normal.dx*F_TestPoint.x + Normal.dy*F_TestPoint.y + Normal.dz*F_TestPoint.z + Distance;
	return SideValue;
}


/************************************************************************/
/* CMatrix3D                                                                     */
/************************************************************************/
CMatrix3D::CMatrix3D()
{
	for (int i =0; i<16; i++)
	{
		a[i] = 0 ;
	}
	a[0] = a[5] = a[10] = 1;
}

CMatrix3D::CMatrix3D( const double* matrix)
{
	for (int i =0; i<16; i++)
	{
		a[i] = matrix[i];
	}
}

CMatrix3D::CMatrix3D( MATRIX3D& v )
{
	for (int i =0; i<16; i++)
	{
		a[i] = v.a[i];
	}
}

CMatrix3D:: ~CMatrix3D()
{
}

//operator
CMatrix3D CMatrix3D::operator*( const MATRIX3D& matrix ) const
{
	CMatrix3D matrix1;
	for (int i = 0; i<4; i++)
	{
		for (int j = 0; j<4; j++)
		{
			matrix1.a[i*4+j] = a[i*4+0]*matrix.a[j+0*4] + a[i*4+1]*matrix.a[j+1*4] 
				+ a[i*4+2]*matrix.a[j+2*4] + a[i*4+3]*matrix.a[j+3*4];
		} 
	}
	return matrix1;
}
void CMatrix3D::operator*=( const MATRIX3D& matrix )
{
	(*this) = (*this)*matrix;
}

//methods
void CMatrix3D::IdenticalMatrix()
{
	for (int i =0; i<16; i++)
	{
		a[i] = 0 ;
	}
	a[0] = a[5] = a[10] = 1;
}

double CMatrix3D::GetValue() const
{
	return	a[0]*a[5]*a[10] + a[1]*a[6]*a[8] + a[2]*a[4]*a[9] 
		- a[0]*a[6]*a[9] - a[1]*a[4]*a[10] - a[2]*a[5]*a[8] ;			
}

	//static member functions
double CMatrix3D::GetValue( double a00, double a01, double a02,
							double a10, double a11, double a12,
							double a20, double a21, double a22 )
{
	return	a00*a11*a22 + a01*a12*a20 + a02*a10*a21 -
			a00*a12*a21 - a01*a01*a22 - a02*a11*a20;
}

CMatrix3D CMatrix3D::CreateMirrorMatrix( VECTOR3D v )
{
	double len = ((CVector3D)v).GetLength();
	CMatrix3D matrix;
	matrix.a[0] = (v.dx*v.dx-1)*2/len/len;
	matrix.a[5] = (v.dy*v.dy-1)*2/len/len;
	matrix.a[10] = (v.dz*v.dz-1)*2/len/len;
	matrix.a[1] = matrix.a[4] = v.dx*v.dy*2/len/len;
	matrix.a[2] = matrix.a[8] = v.dx*v.dz*2/len/len;
	matrix.a[6] = matrix.a[9] = v.dz*v.dy*2/len/len;
	return matrix;
}
//modified by wangxin 
CMatrix3D CMatrix3D::CreateRotateMatrix( double da, CVector3D bv )
{
	bv.Normalize();
	CMatrix3D matrix;
	matrix.a[0] = bv.dx*bv.dx*(1-cos(da))+cos(da);
	matrix.a[5] = bv.dy*bv.dy*(1-cos(da))+cos(da);
	matrix.a[10] = bv.dz*bv.dz*(1-cos(da))+cos(da);
	matrix.a[1] = bv.dx*bv.dy*(1-cos(da))+bv.dz*sin(da);
	matrix.a[4] = bv.dx*bv.dy*(1-cos(da))-bv.dz*sin(da);
	matrix.a[2] = bv.dx*bv.dz*(1-cos(da))-bv.dy*sin(da);
	matrix.a[8] = bv.dx*bv.dz*(1-cos(da))+bv.dy*sin(da);
	matrix.a[6] = bv.dy*bv.dz*(1-cos(da))+bv.dx*sin(da);
	matrix.a[9] = bv.dy*bv.dz*(1-cos(da))-bv.dx*sin(da); 
	matrix.a[3] = matrix.a[7] = matrix.a[11] = matrix.a[12] = matrix.a[13] = matrix.a[14] = 0;
	matrix.a[15] = 1;
	return matrix;
}

CMatrix3D CMatrix3D::CreateScaleMatrix( double d )
{
	CMatrix3D m;
	m.a[0] = m.a[5] = m.a[10] = d;
	return m;
}

CMatrix3D CMatrix3D::CreateTransMatrix( VECTOR3D vec )
{
	CMatrix3D m;
	m.a[12] = vec.dx;
	m.a[13] = vec.dy;
	m.a[14] = vec.dz;
	return m;
}

//////////////////////////////////////////////////////////////////////////
//	CBox3D
//////////////////////////////////////////////////////////////////////////
CBox3D::CBox3D()
{
	x0 = y0 = z0 = x1 = y1 = z1 = 0;
}

CBox3D::CBox3D(double ix0, double iy0, double iz0, double ix1, double iy1, double iz1)
{
	x0 = ix0;
	y0 = iy0;
	z0 = iz0;
	x1 = ix1;
	y1 = iy1;
	z1 = iz1;
	normalize();
}

CBox3D::CBox3D(POINT3D pt0, POINT3D pt1)
{
	x0 = pt0.x;
	y0 = pt0.y;
	z0 = pt0.z;
	x1 = pt1.x;
	y1 = pt1.y;
	z1 = pt1.z;
	normalize();
}

CBox3D::CBox3D(BOX3D b)
{
	x0 = b.x0;
	y0 = b.y0;
	z0 = b.z0;
	x1 = b.x1;
	y1 = b.y1;
	z1 = b.z1;
	normalize();
}

CBox3D::CBox3D(POINT3D p, VECTOR3D v)
{
	x0 = p.x;
	y0 = p.y;
	z0 = p.z;
	x1 = p.x + v.dx;
	y1 = p.y + v.dy;
	z1 = p.z + v.dz;
	normalize();
}

CBox3D::~CBox3D()
{

}

//operator

CBox3D CBox3D::operator +(BOX3D b)const
{
	CBox3D box = *this;

	if (((CBox3D)b).IsEmpty())
	{
		return box;
	}
	if (IsEmpty())
	{
		return b;
	}
	
	box.x0 = (x0 < b.x0) ? x0 : b.x0;
	box.y0 = (y0 < b.y0) ? y0 : b.y0;
	box.z0 = (x0 < b.z0) ? y0 : b.z0;
	box.x1 = (x1 > b.x1) ? x1 : b.x1;
	box.y1 = (y1 > b.y1) ? y1 : b.y1;
	box.z1 = (x1 > b.z1) ? y1 : b.z1;

	return box;
}

void CBox3D::operator *=(double sc)
{
	x0 *= sc;
	y0 *= sc;
	z0 *= sc;
	x1 *= sc;
	y1 *= sc;
	z1 *= sc;
}

CBox3D CBox3D::operator *(const MATRIX3D & matrix)const
{
	CPoint3D p0(x0, y0, z0);
	CPoint3D p1(x1, y1, z1);

	p0 *= matrix;
	p1 *= matrix;

	return CBox3D(p0, p1);
}

void CBox3D::operator *=(const MATRIX3D & matrix)
{
	(*this) = (*this)*matrix;
}

CBox3D CBox3D::operator *(double sc)const
{
	CBox3D box;
	box.x0 = x0*sc;
	box.y0 = y0*sc;
	box.z0 = z0*sc;
	box.x1 = x1*sc;
	box.y1 = y1*sc;
	box.z1 = z1*sc;
	return box;
}

void CBox3D::operator +=(BOX3D b)
{
	(*this) = (*this) + b;
}

CBox3D CBox3D::operator &(BOX3D b)const //Get the big box include this two box
{
	CBox3D box;

	if(((CBox3D)b).IsEmpty() || ((CBox3D*)this)->IsEmpty())
		return box;

	double xx0, yy0, zz0, xx1, yy1, zz1;
	xx0 = (x0>b.x0) ? x0 : b.x0;
	yy0 = (y0>b.y0) ? y0 : b.y0;
	zz0 = (z0>b.z0) ? z0 : b.z0;
	xx1 = (x1<b.x1) ? x1 : b.x1;
	yy1 = (y1<b.y1) ? y1 : b.y1;
	zz1 = (z1<b.z1) ? z1 : b.z1;
	if ((xx1>xx0) && (yy1>yy0) && (zz1>zz0))
		box = CBox3D(xx0, yy0, zz0, xx1, yy1, zz1);
	return box;
}

void CBox3D::operator &=(BOX3D b)
{
	(*this) = (*this) & b;
}

BOOL CBox3D::operator <<(BOX3D b)const
{
	if (IsEmpty())
		return TRUE;

	if ((x0>=b.x0) && (y0>=b.y0) && (z0>=b.z0) && (x1<=b.x1) && (y1<=b.y1) && (z1<=b.z1))
		return FALSE;

	else
		return TRUE;
}

BOOL CBox3D::operator >>(BOX3D b)const
{
	if (IsEmpty())
		return TRUE;
	if ((x0>=b.x0) && (y0>=b.y0) && (z0>=b.z0) && (x1<=b.x1) && (y1<=b.y1) && (z1<=b.z1))
		return TRUE;
	else
		return FALSE;
}

CBox3D CBox3D::operator |(BOX3D b)const//get the little in the two box
{
	CBox3D box;

	if(((CBox3D*)this)->IsEmpty())
	{
		box = b;
		return box;
	} 
	if (((CBox3D)b).IsEmpty())
	{
		box = *this;
		return box;
	}
	double xx0, yy0, zz0, xx1, yy1, zz1;
	xx0 = (x0<b.x0) ? x0 : b.x0;
	yy0 = (y0<b.y0) ? y0 : b.y0;
	zz0 = (z0<b.z0) ? z0 : b.z0;
	xx1 = (x1>b.x1) ? x1 : b.x1;
	yy1 = (y1>b.y1) ? y1 : b.y1;
	zz1 = (z1>b.z1) ? z1 : b.z1;
	if ((xx1>xx0) && (yy1>yy0) && (zz1>zz0))
	{
		box = CBox3D(xx0, yy0, zz0, xx1, yy1, zz1);
	}
	return box;
}

void CBox3D::operator |=(BOX3D b)
{
	(*this) = (*this) | b;
}

BOOL CBox3D::IsEmpty()const
{
	return	IS_ZERO(Width()) && IS_ZERO(Height()) && IS_ZERO(Length());
}

double CBox3D::Width()const
{
	return x1 - x0;
}

double CBox3D::Height()const
{
	return z1 - z0;
}

double CBox3D::Length()const
{
	return y1 - y0;
}

//relationship calculation
//there may be four cases:
//< enumSeperated, enumIntersected >
UINT CBox3D::GetRelationWidth(BOX3D b)const
{
	CBox3D box;
	box = (*this) & b;
	if (box.IsEmpty())
		return enumSeperated;

	else
		return enumIntersected;
}

void CBox3D::normalize()
{
	double xx0, yy0, zz0, xx1, yy1, zz1;
	xx0 = (x0<x1) ? x0 : x1;
	yy0 = (y0<y1) ? y0 : y1;
	zz0 = (z0<z1) ? z0 : z1;
	xx1 = (x0>x1) ? x0 : x1;
	yy1 = (y0>y1) ? y0 : y1;
	zz1 = (z0>z1) ? z0 : z1;
	x0 = xx0;
	y0 = yy0;
	z0 = zz0;
	x1 = xx1;
	y1 = yy1;
	z1 = zz1;
}

CBox3D CBox3D::operator +(VECTOR3D vec)const
{
	CBox3D box;
	box.x0 = x0 + vec.dx;
	box.y0 = y0 + vec.dy;
	box.z0 = z0 + vec.dz;
	box.x1 = x1 + vec.dx;
	box.y1 = y1 + vec.dy;
	box.z1 = z1 + vec.dz;
	return box;
}

void CBox3D::operator +=(VECTOR3D vec)
{
	(*this) = (*this) + vec;
}

CBox3D CBox3D::operator -(VECTOR3D vec)const
{
	CBox3D box;
	box.x0 = x0 + vec.dx;
	box.y0 = y0 + vec.dy;
	box.z0 = z0 + vec.dz;
	box.x1 = x1 + vec.dx;
	box.y1 = y1 + vec.dy;
	box.z1 = z1 + vec.dz;
	return box;
}

void CBox3D::operator -=(VECTOR3D vec)
{
	(*this) = (*this) - vec;
}