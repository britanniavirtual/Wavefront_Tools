#include <iostream>
#include <sstream>

#include "JointHeader.h"

using namespace std;

//Euclidean distance
float Vector3DUtils::dist(float x1, float y1, float z1, float x2, float y2, float z2)
{
	return(sqrt(pow((float)(x2 - x1), (float)2) + pow((float)(y2 - y1), (float)2) + pow((float)(z2 - z1), (float)2)));
}

//Cross product of two vector array.
Vector3D Vector3DUtils::cross(const Vector3D& A, const Vector3D& B)
{
	Vector3D crossP(0, 0, 0);
	crossP.x = A.y * B.z - A.z * B.y;

	crossP.y = A.z * B.x - A.x * B.z;
	crossP.z = A.x * B.y - A.y * B.x;

	return crossP;
}

float Vector3DUtils::angularDifference(Vector3D a, Vector3D b)
{
	float angle = std::acos(dot(a, b) / (length(a)*length(b)));
	return angle;
}



double Vector3DUtils::dot(Vector3D input1, const Vector3D& input2)
{
	return input1.x * input2.x + input1.y * input2.y + input1.z * input2.z;
}

Vector3D Vector3DUtils::intersectPoint(Vector3D rayVector, Vector3D rayPoint, Vector3D planeNormal, Vector3D planePoint)
{
	Vector3D diff = rayPoint - planePoint;
	double prod1 = dot(diff, planeNormal);
	double prod2 = dot(rayVector, planeNormal);
	double prod3 = prod1 / prod2;
	return rayPoint - rayVector * prod3;
}

//Get arbitrary 3d vector that is perpendicular to the parameter vector
//There are infinite such vectors, get one such
Vector3D Vector3DUtils::arbitraryOrthogonal(Vector3D vec)
{
	bool b0 = (vec.x < vec.y) && (vec.x < vec.z);
	bool b1 = (vec.y <= vec.x) && (vec.y < vec.z);
	bool b2 = (vec.z <= vec.x) && (vec.z <= vec.y);

	Vector3D op(0, 0, 0);
	op = cross(vec, Vector3D(int(b0), int(b1), int(b2)));

	return op;
}

//Use spherical coordinates to get a position
Vector3D Vector3DUtils::OrbitalPosition(float angle1, float angle2, Vector3D centroid)
{
	float sx = centroid.x;// -0.013;
	float sy = centroid.y;// 1.06;
	float sz = centroid.z;// 1.06;

	float Theta = angle1;
	float Phi = angle2;
	float radius = 1.0;
	float Y = radius * sin(Theta);
	float X = radius * cos(Theta) * cos(Phi);
	float Z = radius * cos(Theta) * sin(Phi);

	return Vector3D(X + sx, Y + sy, Z + sz);
}

//Set the length (magnitude) of a given vector
Vector3D Vector3DUtils::setVectorMagnitude(Vector3D input, float newMag)
{
	float mag = sqrt(input.x * input.x + input.y * input.y + input.z * input.z);

	float new_x = input.x * newMag / mag;
	float new_y = input.y * newMag / mag;
	float new_z = input.z * newMag / mag;

	Vector3D op(new_x, new_y, new_z);
	return op;
}


Vector3D Vector3DUtils::lerp(Vector3D a, Vector3D b, float scale)
{
	Vector3D op0(0, 0, 0);

	//[End-Start]
	op0.x = b.x - a.x;
	op0.y = b.y - a.y;
	op0.z = b.z - a.z;
	//[Multiply by scale]
	op0 *= scale;

	Vector3D op1(0, 0, 0);
	op1.x = a.x + op0.x;
	op1.y = a.y + op0.y;
	op1.z = a.z + op0.z;

	return(op1);
}


Vector3D Vector3DUtils::displaceVectorTowards(Vector3D a, Vector3D b, float amount)
{
	if (utils.dist(a.x, a.y, a.z, b.x, b.y, b.z) <= 0.0)
	{
		//Vector3D op4(256, 256, 256);
		return(a);
	}

	Vector3D op0(0, 0, 0);

	//[End-Start]
	op0.x = b.x - a.x;
	op0.y = b.y - a.y;
	op0.z = b.z - a.z;
	
	Vector3D op1(op0.x, op0.y, op0.z);
	Vector3D vi(op1.x, op1.y, op1.z);
	float vLen0 = length(vi);
	float vLen1 = 1 / vLen0;//Amount to scale to increase by 1

	Vector3D op3(op1.x, op1.y, op1.z);
	op3 *= vLen1 * amount;

	Vector3D op2(0, 0, 0);
	op2.x = a.x + op3.x;
	op2.y = a.y + op3.y;
	op2.z = a.z + op3.z;

	return(op2);
}

double Vector3DUtils::length(Vector3D vec)
{
	return sqrt(pow(vec.x, 2) + pow(vec.y, 2) + pow(vec.z, 2));
}

Vector3D Vector3DUtils::normalize(Vector3D vec)
{
	float op1 = pow(vec.x, 2) + pow(vec.y, 2) + pow(vec.z, 2);
	op1 = sqrt(op1);

	Vector3D op;
	op.x = vec.x / op1;
	op.y = vec.y / op1;
	op.z = vec.z / op1;

	return op;
}


// Algorithm to test if the line actually reaches rather than extending to infinity
// Get position of intersect, if it lies between start and end of the line then its a hit
// Get Top XYZ and Bottom XYZ

Vector3D vertex0;
Vector3D vertex1;
Vector3D vertex2;
Vector3D edge1, edge2, h, s, q;
const float EPSILON = 0.0000001;
Vector3D rayVector;
bool Vector3DUtils::LineTriangleIntersect(Vector3D lineStart, Vector3D lineEnd, Vector3D v1, Vector3D v2, Vector3D v3, Vector3D* outIntersectionPoint)
{
	rayVector = vector3DUtils.normalize(lineEnd - lineStart);

	//Vector3D rayVector(0, 0, -1);

	vertex0 = v1;
	vertex1 = v2;
	vertex2 = v3;

	float a, f, u, v;
	edge1 = vertex1 - vertex0;
	edge2 = vertex2 - vertex0;

	
	//h = rayVector.crossProduct(edge2);
	h = cross(rayVector, edge2);


	//a = edge1.dotProduct(h);
	a = dot(edge1, h);

	if (a > -EPSILON && a < EPSILON)
		return false;//This ray is parallel to this triangle.


	f = 1.0 / a;
	s = lineStart - vertex0;
	u = f * dot(s, h);//s.dotProduct(h);
	if (u < 0.0 || u > 1.0)
		return false;


	q = cross(s, edge1);//s.crossProduct(edge1);
	v = f * dot(rayVector, q);//rayVector.dotProduct(q);
	if (v < 0.0 || u + v > 1.0)
		return false;


	//At this stage we can compute t to find out where the intersection point is on the line.
	float t = f * dot(edge2, q);//edge2.dotProduct(q);
	if (t > EPSILON)//ray intersection
	{
		*outIntersectionPoint = lineStart + rayVector * t;
		//return true;
	}
	else//No ray intersection
	{
		return false;
	}

	float d1 = dist(lineStart.x, lineStart.y, lineStart.z, outIntersectionPoint->x, outIntersectionPoint->y, outIntersectionPoint->z);
	float d2 = dist(lineEnd.x, lineEnd.y, lineEnd.z, outIntersectionPoint->x, outIntersectionPoint->y, outIntersectionPoint->z);
	
	bool furthest = 0;
	if (d1 < d2) { furthest = 1; }

	float lineLength = dist(lineStart.x, lineStart.y, lineStart.z, lineEnd.x, lineEnd.y, lineEnd.z);

	if (furthest == 0)//D2
	{	
		if (lineLength > d1)
		{
			return true;
		}
		else
		{
			return false;
		}
	}
	else
	{
		if (lineLength > d2)
		{
			return true;
		}
		else
		{
			return false;
		}
	}
}

bool Vector3DUtils::RayTriangleIntersect(Vector3D rayOrigin, Vector3D rayVector, Vector3D* v1, Vector3D* v2, Vector3D* v3, Vector3D& outIntersectionPoint)
{
	const float EPSILON = 0.0000001;
	Vector3D vertex0 = *v1;
	Vector3D vertex1 = *v2;
	Vector3D vertex2 = *v3;
	Vector3D edge1, edge2, h, s, q;
	float a, f, u, v;
	edge1 = vertex1 - vertex0;
	edge2 = vertex2 - vertex0;

	//h = rayVector.crossProduct(edge2);
	h = cross(rayVector, edge2);

	//a = edge1.dotProduct(h);
	a = dot(edge1, h);
	if (a > -EPSILON && a < EPSILON)
		return false;    // This ray is parallel to this triangle.

	f = 1.0 / a;
	s = rayOrigin - vertex0;
	u = f * dot(s, h);//s.dotProduct(h);
	if (u < 0.0 || u > 1.0)
		return false;

	q = cross(s, edge1);// s.crossProduct(edge1);
	v = f * dot(rayVector, q);// rayVector.dotProduct(q);
	if (v < 0.0 || u + v > 1.0)
		return false;

	// At this stage we can compute t to find out where the intersection point is on the line.
	float t = f * dot(edge2, q);// edge2.dotProduct(q);
	if (t > EPSILON) // ray intersection
	{
		outIntersectionPoint = rayOrigin + rayVector * t;
		return true;
	}
	else // This means that there is a line intersection but not a ray intersection.
		return false;
}



Vector3D Vector3DUtils::getTriangleNormal(Vector3D a, Vector3D b, Vector3D c)
{
	Vector3D N(0, 0, 0);

	N.x = a.y * b.z - a.z * b.y;
	N.y = a.z * b.x - a.x * b.z;
	N.z = a.x * b.y - a.y * b.x;


	//N.x = A.y * B.z - A.z * B.y;
	//N.y = A.z * B.x - A.x * B.z;
	//N.z = A.x * B.y - A.y * B.x;

	return N;
}

float Vector3DUtils::pointDistanceToTriangle(Vector3D pointPosition, Vector3D A, Vector3D B, Vector3D C)
{
	Vector3D planeNormal2 = vector3DUtils.normalize(vector3DUtils.getTriangleNormal(A, B, C));
	Vector3D centroid((A.x + B.x + C.x) / 3, (A.y + B.y + C.y) / 3, (A.z + B.z + C.z) / 3);
	Vector3D planePoint = vector3DUtils.closestPlanePoint(pointPosition, centroid, planeNormal2);

	return dist(planePoint.x, planePoint.y, planePoint.z, pointPosition.x, pointPosition.y, pointPosition.z);
}

Vector3D Vector3DUtils::closestPlanePoint(Vector3D pointPosition, Vector3D planePosition, Vector3D planeNormal)
{
	float sb, sn, sd;

	Vector3D d1 = pointPosition - planePosition;
	sn = -vector3DUtils.dot(planeNormal, d1);
	sd = vector3DUtils.dot(planeNormal, planeNormal);

	sb = sn / sd;

	//sn = -Vector3D.Dot(planeNormal, (pointPosition - planePosition));
	//sd = Vector3D.Dot(planeNormal, planeNormal);
	//sb = sn / sd;

	Vector3D result = pointPosition + (planeNormal * sb);

	return result;
}


//--------------------------------------------------------------------------------------------------------------------


void Vector3DUtils::VmV(float Vr[3], const float V1[3], const float V2[3])
{
	Vr[0] = V1[0] - V2[0];
	Vr[1] = V1[1] - V2[1];
	Vr[2] = V1[2] - V2[2];
}

float Vector3DUtils::VdotV(const float V1[3], const float V2[3])
{
	return (V1[0] * V2[0] + V1[1] * V2[1] + V1[2] * V2[2]);
}

void Vector3DUtils::VcV(float Vr[3], const float V[3])
{
	Vr[0] = V[0];  Vr[1] = V[1];  Vr[2] = V[2];
}

void Vector3DUtils::VpV(float Vr[3], const float V1[3], const float V2[3])
{
	Vr[0] = V1[0] + V2[0];
	Vr[1] = V1[1] + V2[1];
	Vr[2] = V1[2] + V2[2];
}

void Vector3DUtils::VpVxS(float Vr[3], const float V1[3], const float V2[3], float s)
{
	Vr[0] = V1[0] + V2[0] * s;
	Vr[1] = V1[1] + V2[1] * s;
	Vr[2] = V1[2] + V2[2] * s;
}

void Vector3DUtils::VcrossV(float Vr[3], const float V1[3], const float V2[3])
{
	Vr[0] = V1[1] * V2[2] - V1[2] * V2[1];
	Vr[1] = V1[2] * V2[0] - V1[0] * V2[2];
	Vr[2] = V1[0] * V2[1] - V1[1] * V2[0];
}

void Vector3DUtils::VxS(float Vr[3], const float V[3], float s)
{
	Vr[0] = V[0] * s;
	Vr[1] = V[1] * s;
	Vr[2] = V[2] * s;
}

float Vector3DUtils::VdistV2(const float V1[3], const float V2[3])
{
	return ((V1[0] - V2[0]) * (V1[0] - V2[0]) +
		(V1[1] - V2[1]) * (V1[1] - V2[1]) +
		(V1[2] - V2[2]) * (V1[2] - V2[2]));
}

void Vector3DUtils::MxVpV(float Vr[3], const float M1[3][3], const float V1[3], const float V2[3])
{
	Vr[0] = (M1[0][0] * V1[0] +
		M1[0][1] * V1[1] +
		M1[0][2] * V1[2] +
		V2[0]);
	Vr[1] = (M1[1][0] * V1[0] +
		M1[1][1] * V1[1] +
		M1[1][2] * V1[2] +
		V2[1]);
	Vr[2] = (M1[2][0] * V1[0] +
		M1[2][1] * V1[1] +
		M1[2][2] * V1[2] +
		V2[2]);
}

//-------------------------------------
//
//-------------------------------------
void Vector3DUtils::SegPoints(float VEC[3], float X[3], float Y[3], // closest points
	const float P[3], const float A[3], // seg 1 origin, vector
	const float Q[3], const float B[3]) // seg 2 origin, vector
{
	float T[3], A_dot_A, B_dot_B, A_dot_B, A_dot_T, B_dot_T;
	float TMP[3];

	VmV(T, Q, P);
	A_dot_A = VdotV(A, A);
	B_dot_B = VdotV(B, B);
	A_dot_B = VdotV(A, B);
	A_dot_T = VdotV(A, T);
	B_dot_T = VdotV(B, T);

	// t parameterizes ray P,A 
	// u parameterizes ray Q,B 

	float t, u;

	// compute t for the closest point on ray P,A to
	// ray Q,B

	float denom = A_dot_A * B_dot_B - A_dot_B * A_dot_B;

	t = (A_dot_T*B_dot_B - B_dot_T * A_dot_B) / denom;

	// clamp result so t is on the segment P,A

	if ((t < 0) || isnan(t)) t = 0; else if (t > 1) t = 1;

	// find u for point on ray Q,B closest to point at t

	u = (t*A_dot_B - B_dot_T) / B_dot_B;

	// if u is on segment Q,B, t and u correspond to 
	// closest points, otherwise, clamp u, recompute and
	// clamp t 

	if ((u <= 0) || isnan(u)) {

		VcV(Y, Q);

		t = A_dot_T / A_dot_A;

		if ((t <= 0) || isnan(t)) {
			VcV(X, P);
			VmV(VEC, Q, P);
		}
		else if (t >= 1) {
			VpV(X, P, A);
			VmV(VEC, Q, X);
		}
		else {
			VpVxS(X, P, A, t);
			VcrossV(TMP, T, A);
			VcrossV(VEC, A, TMP);
		}
	}
	else if (u >= 1) {

		VpV(Y, Q, B);

		t = (A_dot_B + A_dot_T) / A_dot_A;

		if ((t <= 0) || isnan(t)) {
			VcV(X, P);
			VmV(VEC, Y, P);
		}
		else if (t >= 1) {
			VpV(X, P, A);
			VmV(VEC, Y, X);
		}
		else {
			VpVxS(X, P, A, t);
			VmV(T, Y, P);
			VcrossV(TMP, T, A);
			VcrossV(VEC, A, TMP);
		}
	}
	else {

		VpVxS(Y, Q, B, u);

		if ((t <= 0) || isnan(t)) {
			VcV(X, P);
			VcrossV(TMP, T, B);
			VcrossV(VEC, B, TMP);
		}
		else if (t >= 1) {
			VpV(X, P, A);
			VmV(T, Q, X);
			VcrossV(TMP, T, B);
			VcrossV(VEC, B, TMP);
		}
		else {
			VpVxS(X, P, A, t);
			VcrossV(VEC, A, B);
			if (VdotV(VEC, T) < 0) {
				VxS(VEC, VEC, -1);
			}
		}
	}
}
//-------------------------------------
//
//-------------------------------------
bool Vector3DUtils::vectorPointsTowards(Vector3D centroid, Vector3D a, Vector3D b)
{
	Vector3D an = normalize(a - centroid);
	Vector3D bn = normalize(b - centroid);

	if (dot(an, bn) < 0)
	{
		return 0;
	}

	return 1;
}


///////////////////////////////////////////
//Point triangle distance
///////////////////////////////////////////
template <typename T> int sign(T val)
{
	return (T(0) < val) - (val < T(0));
}

//-------------------------------------
//
//-------------------------------------
float dot2(Vector3D v) { return vector3DUtils.dot(v, v); }

//-------------------------------------
//
//-------------------------------------
double clamp(double d, double min, double max)
{
	const double t = d < min ? min : d;
	return t > max ? max : t;
}

//-------------------------------------
//
//-------------------------------------
float Vector3DUtils::trianglePointDist(Vector3D v1, Vector3D v2, Vector3D v3, Vector3D point)
{
	Vector3D v21 = v2 - v1; Vector3D p1 = point - v1;
	Vector3D v32 = v3 - v2; Vector3D p2 = point - v2;
	Vector3D v13 = v1 - v3; Vector3D p3 = point - v3;
	Vector3D nor = vector3DUtils.cross(v21, v13);

	return sqrt( 
		(sign(vector3DUtils.dot(vector3DUtils.cross(v21, nor), p1)) +
			sign(vector3DUtils.dot(vector3DUtils.cross(v32, nor), p2)) +
			sign(vector3DUtils.dot(vector3DUtils.cross(v13, nor), p3)) < 2.0)
		?

		min(min(
			dot2(v21 * clamp(vector3DUtils.dot(v21, p1) / dot2(v21), 0.0, 1.0) - p1),
			dot2(v32 * clamp(vector3DUtils.dot(v32, p2) / dot2(v32), 0.0, 1.0) - p2)),
			dot2(v13 * clamp(vector3DUtils.dot(v13, p3) / dot2(v13), 0.0, 1.0) - p3))
		:
		vector3DUtils.dot(nor, p1) * vector3DUtils.dot(nor, p1) / dot2(nor));
}


//-------------------------------------
//
//-------------------------------------
Vector3D Vector3DUtils::closestTrianglePoint(const Vector3D *triangle, const Vector3D &position)
{
	Vector3D edge0 = triangle[1] - triangle[0];
	Vector3D edge1 = triangle[2] - triangle[0];
	Vector3D v0 = triangle[0] - position;

	float a = vector3DUtils.dot(edge0, edge0);
	float b = vector3DUtils.dot(edge0, edge1);
	float c = vector3DUtils.dot(edge1, edge1);
	float d = vector3DUtils.dot(edge0, v0);
	float e = vector3DUtils.dot(edge1, v0);

	float det = a * c - b * b;
	float s = b * e - c * d;
	float t = b * d - a * e;

	if (s + t < det)
	{
		if (s < 0.f)
		{
			if (t < 0.f)
			{
				if (d < 0.f)
				{
					s = clamp(-d / a, 0.f, 1.f);
					t = 0.f;
				}
				else
				{
					s = 0.f;
					t = clamp(-e / c, 0.f, 1.f);
				}
			}
			else
			{
				s = 0.f;
				t = clamp(-e / c, 0.f, 1.f);
			}
		}
		else if (t < 0.f)
		{
			s = clamp(-d / a, 0.f, 1.f);
			t = 0.f;
		}
		else
		{
			float invDet = 1.f / det;
			s *= invDet;
			t *= invDet;
		}
	}
	else
	{
		if (s < 0.f)
		{
			float tmp0 = b + d;
			float tmp1 = c + e;
			if (tmp1 > tmp0)
			{
				float numer = tmp1 - tmp0;
				float denom = a - 2 * b + c;
				s = clamp(numer / denom, 0.f, 1.f);
				t = 1 - s;
			}
			else
			{
				t = clamp(-e / c, 0.f, 1.f);
				s = 0.f;
			}
		}
		else if (t < 0.f)
		{
			if (a + d > b + e)
			{
				float numer = c + e - b - d;
				float denom = a - 2 * b + c;
				s = clamp(numer / denom, 0.f, 1.f);
				t = 1 - s;
			}
			else
			{
				s = clamp(-e / c, 0.f, 1.f);
				t = 0.f;
			}
		}
		else
		{
			float numer = c + e - b - d;
			float denom = a - 2 * b + c;
			s = clamp(numer / denom, 0.f, 1.f);
			t = 1.f - s;
		}
	}

	return triangle[0] + (edge0 * s) + (edge1 * t);
}




//-------------------------------------
// Minimum distance between two triangles
//-------------------------------------
float Vector3DUtils::triangleTriangleDistance(float IntersectPointA[3], float IntersectPointB[3], Vector3D vA1, Vector3D vA2, Vector3D vA3, Vector3D vB1, Vector3D vB2, Vector3D vB3)
{
	float S[3][3];
	float T[3][3];

	S[0][0] = vA1.x;
	S[0][1] = vA1.y;
	S[0][2] = vA1.z;

	S[1][0] = vA2.x;
	S[1][1] = vA2.y;
	S[1][2] = vA2.z;

	S[2][0] = vA3.x;
	S[2][1] = vA3.y;
	S[2][2] = vA3.z;


	T[0][0] = vB1.x;
	T[0][1] = vB1.y;
	T[0][2] = vB1.z;

	T[1][0] = vB2.x;
	T[1][1] = vB2.y;
	T[1][2] = vB2.z;

	T[2][0] = vB3.x;
	T[2][1] = vB3.y;
	T[2][2] = vB3.z;

	//Compute vectors along the 6 sides

	float Sv[3][3], Tv[3][3];
	float VEC[3];

	VmV(Sv[0], S[1], S[0]);
	VmV(Sv[1], S[2], S[1]);
	VmV(Sv[2], S[0], S[2]);

	VmV(Tv[0], T[1], T[0]);
	VmV(Tv[1], T[2], T[1]);
	VmV(Tv[2], T[0], T[2]);

	float V[3];
	float Z[3];
	float minP[3], minQ[3], mindd;
	int shown_disjoint = 0;

	mindd = VdistV2(S[0], T[0]) + 1;

	for (unsigned short i = 0; i < 3; i++)
	{
		for (unsigned short j = 0; j < 3; j++)
		{
			SegPoints(VEC, IntersectPointA, IntersectPointB, S[i], Sv[i], T[j], Tv[j]);

			VmV(V, IntersectPointB, IntersectPointA);
			float dd = VdotV(V, V);

			if (dd <= mindd)
			{
				VcV(minP, IntersectPointA);
				VcV(minQ, IntersectPointB);
				mindd = dd;

				VmV(Z, S[(i + 2) % 3], IntersectPointA);
				float a = VdotV(Z, VEC);
				VmV(Z, T[(j + 2) % 3], IntersectPointB);
				float b = VdotV(Z, VEC);

				if ((a <= 0) && (b >= 0)) return sqrt(dd);

				float p = VdotV(V, VEC);

				if (a < 0) a = 0;
				if (b > 0) b = 0;
				if ((p - a + b) > 0) shown_disjoint = 1;
			}
		}
	}

	float Sn[3], Snl;
	VcrossV(Sn, Sv[0], Sv[1]);
	Snl = VdotV(Sn, Sn);

	if (Snl > 1e-15)
	{
		float Tp[3];

		VmV(V, S[0], T[0]);
		Tp[0] = VdotV(V, Sn);

		VmV(V, S[0], T[1]);
		Tp[1] = VdotV(V, Sn);

		VmV(V, S[0], T[2]);
		Tp[2] = VdotV(V, Sn);
		int point = -1;
		if ((Tp[0] > 0) && (Tp[1] > 0) && (Tp[2] > 0))
		{
			if (Tp[0] < Tp[1]) point = 0; else point = 1;
			if (Tp[2] < Tp[point]) point = 2;
		}
		else if ((Tp[0] < 0) && (Tp[1] < 0) && (Tp[2] < 0))
		{
			if (Tp[0] > Tp[1]) point = 0; else point = 1;
			if (Tp[2] > Tp[point]) point = 2;
		}

		if (point >= 0)
		{
			shown_disjoint = 1;

			VmV(V, T[point], S[0]);
			VcrossV(Z, Sn, Sv[0]);
			if (VdotV(V, Z) > 0)
			{
				VmV(V, T[point], S[1]);
				VcrossV(Z, Sn, Sv[1]);
				if (VdotV(V, Z) > 0)
				{
					VmV(V, T[point], S[2]);
					VcrossV(Z, Sn, Sv[2]);
					if (VdotV(V, Z) > 0)
					{
						VpVxS(IntersectPointA, T[point], Sn, Tp[point] / Snl);
						VcV(IntersectPointB, T[point]);
						return sqrt(VdistV2(IntersectPointA, IntersectPointB));
					}
				}
			}
		}
	}

	float Tn[3], Tnl;
	VcrossV(Tn, Tv[0], Tv[1]);
	Tnl = VdotV(Tn, Tn);

	if (Tnl > 1e-15)
	{
		float Sp[3];

		VmV(V, T[0], S[0]);
		Sp[0] = VdotV(V, Tn);

		VmV(V, T[0], S[1]);
		Sp[1] = VdotV(V, Tn);

		VmV(V, T[0], S[2]);
		Sp[2] = VdotV(V, Tn);

		int point = -1;
		if ((Sp[0] > 0) && (Sp[1] > 0) && (Sp[2] > 0))
		{
			if (Sp[0] < Sp[1]) point = 0; else point = 1;
			if (Sp[2] < Sp[point]) point = 2;
		}
		else if ((Sp[0] < 0) && (Sp[1] < 0) && (Sp[2] < 0))
		{
			if (Sp[0] > Sp[1]) point = 0; else point = 1;
			if (Sp[2] > Sp[point]) point = 2;
		}

		if (point >= 0)
		{
			shown_disjoint = 1;

			VmV(V, S[point], T[0]);
			VcrossV(Z, Tn, Tv[0]);
			if (VdotV(V, Z) > 0)
			{
				VmV(V, S[point], T[1]);
				VcrossV(Z, Tn, Tv[1]);
				if (VdotV(V, Z) > 0)
				{
					VmV(V, S[point], T[2]);
					VcrossV(Z, Tn, Tv[2]);
					if (VdotV(V, Z) > 0)
					{
						VcV(IntersectPointA, S[point]);
						VpVxS(IntersectPointB, S[point], Tn, Sp[point] / Tnl);


						return sqrt(VdistV2(IntersectPointA, IntersectPointB));
					}
				}
			}
		}
	}

	if (shown_disjoint)
	{
		VcV(IntersectPointA, minP);
		VcV(IntersectPointB, minQ);
		return sqrt(mindd);
	}
	else return 0;
}