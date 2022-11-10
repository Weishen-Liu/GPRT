


#include "generator/CylinderMesh.hpp"

using namespace generator;

CylinderMesh::CylinderMesh(
	double radius,
	double size,
	int slices,
	int segments,
	double start,
	double sweep
) :
	axisSwapMesh_{
		{
			{{size, radius}, {-size, radius}, segments},
			{1.0, 0.0}, slices, start, sweep
		},
		Axis::Y, Axis::Z, Axis::X
	}
{ }


