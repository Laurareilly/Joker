#include "QuaternionTest.h"
#include "Quaternion.h"
#include "vector3.h"
#include "utils.h"

void runQuaternionTests()
{
	Vector3 startPoint = UP_VECTOR3;

	Quaternion test = Quaternion::makeRotationZYX(DegToRad(90.0f), DegToRad(90.0f), 0.0f);
	Vector3 newPoint = test * startPoint;
	newPoint.print();
	//printVector(&newPoint, '\n');

	//Quaternion xyz = Quaternion::makeRotationXYZ(DegToRad(40.0f), DegToRad(23.512f), DegToRad(180.0f)); //correctly produces different results.
	//Quaternion zyx = Quaternion::makeRotationZYX(DegToRad(40.0f), DegToRad(23.512f), DegToRad(180.0f));

	Quaternion xyz = Quaternion::makeRotationXYZ(DegToRad(90.0f), 0.0f, 0.0f);	//correctly produces the same result.
	Quaternion zyx = Quaternion::makeRotationZYX(0.0f, 0.0f, DegToRad(90.0f));

	Vector3 startXYZ = UP_VECTOR3, startZYX = UP_VECTOR3;
	startXYZ = xyz * startXYZ;
	startZYX = zyx * startZYX;

	startXYZ.print();
	startZYX.print();
}
