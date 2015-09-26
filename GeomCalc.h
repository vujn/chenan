#pragma  once
#include "stdafx.h"
#include <math.h>
#include <wtypes.h>


#define CAD_ZERO				1.0E-6
#define NC_ZERO					1.0E-3
#define IS_ZERO(x)				( fabs(x) <= CAD_ZERO )
#define IS_POSITIVE(x)			( fabs(x) > CAD_ZERO && x > 0.0)
#define IS_NEGATIVE(x)			( fabs(x) > CAD_ZERO && x < 0.0)
#define IS_NCZERO(x)			( fabs(x) <= NC_ZERO )
#define IS_BETWEEN(x, min, max)	( x <= max && x >= min )
#define PI						3.1415926535
#define OFFSET					1.0E-1

typedef struct tagPoint3D
{
	double x;
	double y;
	double z;
} POINT3D, *PPOINT3D;

typedef struct tagVector3D
{
	double dx;
	double dy;
	double dz;
} VECTOR3D, *PVECTOR3D;

typedef struct tagMatrix3D 
{
	double a[16];
} MATRIX3D, *PMATRIX3D;

class CPoint3D;
class CVector3D;
class CMatrix3D;

class CPoint3D :public POINT3D
{
public:
	CPoint3D();
	CPoint3D( double ix, double iy, double iz=0.0);
	CPoint3D( const double* );
	CPoint3D( POINT3D p );
	virtual ~CPoint3D();

public:
	//operators
	CPoint3D operator*( const MATRIX3D& matrix ) const;
	void operator*=( const MATRIX3D& matrix );

	//offsetting with vector
	CPoint3D operator+( VECTOR3D v ) const;
	void operator+=( VECTOR3D v );
	CPoint3D operator-( VECTOR3D v ) const;
	void operator-=( VECTOR3D v );

	CPoint3D operator/(double d) const;
	void operator/=(double d);

	BOOL operator==( POINT3D pos ) const;
	BOOL operator!=( POINT3D pos ) const;

	//derived vector = this point -sp
	CVector3D operator-( POINT3D sp ) const;

	BOOL IsAtTheLeftOf(CPoint3D p1, CPoint3D p2, CVector3D norm) const;
	BOOL IsInTheTriangle(CPoint3D p1, CPoint3D p2, CPoint3D p3) const;
};

class CVector3D :public VECTOR3D
{
public:
	CVector3D();
	CVector3D( double ix, double iy, double iz=0.0);
	CVector3D( const double* );
	CVector3D( VECTOR3D v );
	virtual ~CVector3D();

	//operator
	CVector3D operator+( VECTOR3D v ) const;
	void operator+=( VECTOR3D v );
	CVector3D operator-( VECTOR3D v ) const;
	void operator-=( VECTOR3D v );

	CVector3D operator*( double d ) const;
	void operator*=( double d );
	CVector3D operator/( double d ) const;
	void operator/=( double d );

	//cross product
	CVector3D operator*( VECTOR3D v ) const;

	//dot product
	double operator|( VECTOR3D v ) const;

	CVector3D operator*( const MATRIX3D& matrix ) const;
	void operator*=( const MATRIX3D& matrix );

	//相等判断
	BOOL operator==( VECTOR3D v ) const;
	BOOL operator!=( VECTOR3D v ) const;

	//length
	double GetLength() const;
	double GetLengthXY() const;
	double GetLengthYZ() const;
	double GetLengthZX() const;

	CVector3D GetNormal() const;
	void Normalize();
	BOOL IsZeroLength() const;
	//生成一个与给定向量垂直的向量
	CVector3D GetVerticalVector(int m) const;
	//生成一个过给定向量的平面
	void GenerateTFace(double * coefficient,CPoint3D p,int m) const;
	//生成一个垂直于给定向量的平面
	void GenerateVFace(double * coefficient,CPoint3D p) const;
};

class CPlane3D
{
public:
	CPlane3D(){};
	CPlane3D( CPoint3D F_P1,CPoint3D F_P2,CPoint3D F_P3,CPoint3D F_P4);
	CVector3D GetNormal( CPoint3D F_Point ) { return Normal; };
	double GetDistance() { return Distance; };
	double SideTest(CPoint3D F_TestPoint);
private:
	CVector3D Normal;
	double Distance;
};

class CMatrix3D :public MATRIX3D
{
public:
	CMatrix3D();
	CMatrix3D( const double* );
	CMatrix3D( MATRIX3D& v );
	virtual ~CMatrix3D();

	//operator
	CMatrix3D operator*( const MATRIX3D& matrix ) const;
	void operator*=( const MATRIX3D& matrix );

	//methods
	void IdenticalMatrix();
	double GetValue() const;

public:
	//static member functions
	static double GetValue( double a00, double a01, double a02,
							double a10, double a11, double a12,
							double a20, double a21, double a22 );
	static CMatrix3D CreateMirrorMatrix( VECTOR3D v );
	static CMatrix3D CreateRotateMatrix( double da, CVector3D bv );
	static CMatrix3D CreateScaleMatrix( double );
	static CMatrix3D CreateTransMatrix( VECTOR3D vec );
};

double  _AngleBetween(VECTOR3D v1, VECTOR3D v2);
double  _DistOf(POINT3D pt0, POINT3D pt1);
BOOL  _IsParallel(VECTOR3D v1, VECTOR3D v2);
BOOL  _IsOrthogonal(VECTOR3D v1, VECTOR3D v2);
double  _AngleBetween(VECTOR3D v1, VECTOR3D v2, VECTOR3D normal);
BOOL  _IsInTheSameFace(CPoint3D pt1, CPoint3D pt2, CPoint3D pt3, CPoint3D pt4);
BOOL  _Intersection(CPoint3D pt1, CPoint3D pt2, CPoint3D pt3, CPoint3D pt4, CPoint3D* intersection = NULL);
double  GetMax(double a, double b);
double  GetMin(double a, double b);


typedef struct tagBox3D 
{
	double x0;
	double y0;
	double z0;
	double x1;
	double y1;
	double z1;
}	BOX3D, *PBOX3D;

class  CBox3D : public BOX3D
{
public:
	CBox3D();
	CBox3D(double ix0, double iy0, double iz0, double ix1, double iy1, double iz1);
	CBox3D(POINT3D pt0, POINT3D pt1);
	CBox3D(BOX3D b);
	CBox3D(POINT3D p, VECTOR3D v);
	virtual ~CBox3D();

	//operator
public:
	//get the union box of this and box b
	CBox3D operator+(BOX3D b)const;
	void operator+=(BOX3D b);

	//get the intersect box of this and box b
	CBox3D operator&(BOX3D b)const;
	void operator&=(BOX3D b);
	CBox3D operator*(double sc)const;
	void operator*=(double sc);
	CBox3D operator*(const MATRIX3D& matrix)const;
	void operator*=(const MATRIX3D& matrix);

	BOOL operator<<(BOX3D b)const;
	BOOL operator>>(BOX3D b)const;
	BOOL operator>>(POINT3D p)const;
	
	CBox3D operator|(BOX3D b)const;
	void operator|=(BOX3D b);

	CBox3D operator+(VECTOR3D vec)const;
	void operator+=(VECTOR3D vec);
	CBox3D operator-(VECTOR3D vec)const;
	void operator-=(VECTOR3D vec);

	//get attributes
	BOOL IsEmpty()const;
	double Width()const;
	double Length()const;
	double Height()const;

	enum {enumSeperated, enumIntersected};
	UINT GetRelationWidth(BOX3D b)const;

protected:
	void normalize();
};
