#include "TStack.h"
#include <iostream>
#include "vector3.h"
#include "transformMatrix.h"
#include "Quaternion.h"
#include "utils.h"

TransformationMatrix Stack<TransformationMatrix>::product() const
{
	TransformationMatrix result;

	auto walker = mHead;
	while (walker != nullptr)
	{
		result *= walker->mData;
		walker = walker->mNext;
	}

	return result;
}


void testStack()
{
	Stack<int> aStack({ 5, 4, 3, 2, 1 });

	while (!aStack.empty())
		std::cout << aStack.pop() << " ";

	std::cout << std::endl;

	aStack.push(5);
	aStack.push(75);

	std::cout << aStack.peek() << " ";
	aStack.pop();
	std::cout << aStack.peek() << " ";
	aStack.pop();

	std::cout << " empty now? " << (aStack.empty() ? "true" : "false") << std::endl;

	Stack<TransformationMatrix> transformStack;
	
	transformStack.push(TransformationMatrix::makeRotationZYX(0.0f, 0.0f, DegToRad(-90.0f)));
	transformStack.push(TransformationMatrix::makeUniformScale(2.0f));
	transformStack.push(TransformationMatrix::makeTranslation(0.0f, 0.0f, 5.0f));
	transformStack.push(TransformationMatrix::makeTranslation(0.0f, 2.0f, 0.0f));

	Vector3 newPoint = transformStack.product() * UP_VECTOR3;
	newPoint.print();

	transformStack.push(TransformationMatrix::makeTranslation(0.0f, -2.0f, -5.0f));
	transformStack.push(TransformationMatrix::makeUniformScale(0.5f));
	transformStack.push(TransformationMatrix::makeRotationZYX(0.0f, 0.0f, DegToRad(90.0f)));

	newPoint = transformStack.product() * UP_VECTOR3;
	newPoint.print();

	std::cout << std::endl;
	system("pause");
}
