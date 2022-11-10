

#include "generator/AnyPath.hpp"


using namespace generator;


AnyPath::Base::~Base() { }


AnyPath::AnyPath(const AnyPath& that) :
	base_{that.base_->clone()}
{ }


AnyPath& AnyPath::operator=(const AnyPath& that) {
	base_ = that.base_->clone();
	return *this;
}


AnyGenerator<Edge> AnyPath::edges() const noexcept {
	return base_->edges();
}


AnyGenerator<PathVertex> AnyPath::vertices() const noexcept {
	return base_->vertices();
}
