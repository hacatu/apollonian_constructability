#include <cmath>
#include <vector>
#include <algorithm>
#include <memory>
#include "primitives.hxx"


Circle::Circle(double x, double y, double r) : c(x, y), r(r) {}
Circle::Circle(const Point &c, double r) : c(c), r(r) {}

Circle Circle::construct(const Point &a, const Point &b){
	return Circle(a, std::hypot(b.x - a.x, b.y - a.y));
}

bool Circle::contains(const Point &p) const {
	return std::fabs(std::hypot(p.x - c.x, p.y - c.y) - r) < EPSILON;
}

Point Circle::getArbitraryPoint(const Point &a, const Point &b) const;

bool Circle::operator==(const Circle &other) const {
	return this->c == other.c && std::fabs(other.r - this->r) < EPSILON;
}

Line::Line(double x, double y, double dx, double dy) : p(x, y), dx(dx), dy(dy) {}
Line::Line(const Point &p, double dx, double dy) : p(p), dx(dx), dy(dy) {}

Line Line::construct(const Point &a, const Point &b){
	double dx = b.x - a.x;
	double dy = b.y - a.y;
	double h = std::hypot(dx, dy);
	return Line(a, dx/h, dy/h);
}

bool Line::contains(const Point &p) const {
	return std::fabs((p.x - this->p.x)*-dy + (p.y - this->p.y)*dx) < EPSILON;
}

Line::isInfinite(const Point &a, const Point &b) const {
	return isInfiniteTail(a, b) || isInfiniteHead(a, b);
}

Line::isInfiniteTail(const Point &a, const Point &b) const {
	return std::isinf(a * this->d);
}

Line::Segment::isInfiniteHead(const Point &a, const Point &b) const {
	return std::isinf(b * this->d);
}

Line::Segment::isInfiniteBoth(const Point &a, const Point &b) const {
	return isInfiniteTail(a, b) && isInfiniteHead(a, b);
}

Point Line::getArbitraryPoint(const Point &a, const Point &b) const;

bool Line::operator==(const Line &other) const {
	if(std::fabs(this->dx*other.dx + this->dy*other.dy) < 1 - EPSILON){
		return false;
	}
	double dx = other.p.x - this->p.x;
	double dy = other.p.y - this->p.y;
	return std::fabs(dx*-this->dy + dy*other.dx) < EPSILON;
}

