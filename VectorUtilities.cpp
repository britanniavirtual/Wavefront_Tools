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




double Vector3DUtils::dot(Vector3D input1, const Vector3D& input2)
{
	return input1.x * input2.x + input1.y * input2.y + input1.z * input2.z;
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
