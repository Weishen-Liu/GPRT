

#ifndef GENERATOR_RECTANGLESHAPE_HPP
#define GENERATOR_RECTANGLESHAPE_HPP


#include "LineShape.hpp"
#include "MergeShape.hpp"


namespace generator {



/// Rectangle centered at origin aligned along the x and y axis.
/// @image html RectangleShape.svg
class RectangleShape
{
private:

	using Impl = MergeShape<LineShape, LineShape, LineShape, LineShape>;
	Impl mergeShape_;

public:

	/// @param size Half of the length of an edge.
	/// @param segments Number of subdivisions per edge.
	RectangleShape(
		const gml::dvec2& size = gml::dvec2{1.0, 1.0},
		const gml::ivec2& segments = gml::ivec2{8, 8}
	);

	using Edges = typename Impl::Edges;

	Edges edges() const noexcept { return mergeShape_.edges(); }

	using Vertices = typename Impl::Vertices;

	Vertices vertices() const noexcept { return mergeShape_.vertices(); }

};


}

#endif
