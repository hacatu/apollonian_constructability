#define _USE_MATH_DEFINES
#include <cmath>
#include <vector>
#include <algorithm>
#include <functional>
#include <stdexcept>
#include "geometry.hxx"
#include "init_geometry.hxx"

EquilateralFactory::EquilateralFactory(double side, Point &&c, double angle) : side(side), c(c), angle(angle) {}

ApproxCons EquilateralFactory::operator()() const {
	ApproxCons ret(10, 2, 3, 2, 2);
	double r = this->side/std::sqrt(3);
	const Point p = this->c + Point(r, 0);
	for(int i = 0; i < 3; ++i){
		ret.points.push_back(this->c.rotate(this->angle + 2*M_PI*i/3, p));
	}
	for(size_t i = 0; i < 3; ++i){
		ret.add_circle_unchecked(Circle(ret.points[i], this->side/2));
		ret.add_line_unchecked(Line::construct(ret.points[i], ret.points[(i +1)%3]));
	}
	ret.goal = Circle(this->c, this->side*(1/std::sqrt(3) - .5));
	return ret;
}

IsoscelesFactory::IsoscelesFactory(double side, double base, Point &&c, double angle) : side(side), base(base), c(c), angle(angle) {
	if(2*side <= base){
		throw std::domain_error("Invalid triangle sides");
	}
}
	
ApproxCons IsoscelesFactory::operator()() const {
	ApproxCons ret(10, 2, 3, 4, 2);
	double rb = this->base/2;
	double rs = this->side - rb;
	double ra = 1/(2/rb + 1/rs + 2*std::sqrt(1/(rb*rb) + 2/(rb*rs)));
	double ia = std::atan2(rs, rs + ra - std::sqrt(this->side*this->side - rb*rb));
	ret.points.push_back(this->c.rotate(angle, c + Point(rs + ra, 0)));
	const Point a = this->c + Point(rb + ra, 0);
	ret.points.push_back(this->c.rotate(angle + ia, a));
	ret.points.push_back(this->c.rotate(angle - ia, a));
	ret.add_circle_unchecked(Circle(ret.points[0], rs));
	ret.add_circle_unchecked(Circle(ret.points[1], rb));
	ret.add_circle_unchecked(Circle(ret.points[2], rb));
	for(size_t i = 0; i < 3; ++i){
		ret.add_line_unchecked(Line::construct(ret.points[i], ret.points[(i +1)%3]));
	}
	ret.goal = Circle(this->c, ra);
	return ret;
}

