

#ifndef GENERATOR_UVFLIPMESH_HPP
#define GENERATOR_UVFLIPMESH_HPP

#include "TransformMesh.hpp"


namespace generator {


/// Flips texture coordinate axis directions.
template <typename Mesh>
class UvFlipMesh
{
private:

	using Impl = TransformMesh<Mesh>;
	Impl transformMesh_;

public:

	/// @param mesh Source data mesh.
	/// @param u Flip u
	/// @param v Flip v
	UvFlipMesh(Mesh mesh, bool u, bool v) :
		transformMesh_{
			std::move(mesh),
			[u, v] (MeshVertex& vertex) {
				if (u) vertex.texCoord[0] = 1.0 - vertex.texCoord[0];
				if (v) vertex.texCoord[1] = 1.0 - vertex.texCoord[1];
			}
		}
	{ }

	using Triangles = typename Impl::Triangles;

	Triangles triangles() const noexcept { return transformMesh_.triangles(); }

	using Vertices = typename Impl::Vertices;

	Vertices vertices() const noexcept { return transformMesh_.vertices(); }

};


template <typename Mesh>
UvFlipMesh<Mesh> uvFlipMesh(Mesh mesh) {
	return UvFlipMesh<Mesh>{std::move(mesh)};
}


}


#endif

