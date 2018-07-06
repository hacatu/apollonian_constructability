#define _USE_MATH_DEFINES
#include <cmath>
#include <complex>
#include <vector>
#include <algorithm>
#include <functional>
#include <stdexcept>
#include "geometry.hxx"
#include "init_geometry.hxx"

EquilateralFactory::EquilateralFactory(double side, Point &&c, double angle) : side(side), c(c), angle(angle) {}

ApproxCons EquilateralFactory::operator()() const {
	ApproxCons ret(10, 2, 3, 2, 2);
	const double r = this->side/std::sqrt(3);
	const Point p = this->c + Point(r, 0);
	for(int i = 0; i < 3; ++i){
		ret.points.push_back(this->c.rotate(this->angle + 2*M_PI*i/3, p));
	}
	ret.init_topology();
	for(size_t i = 0; i < 3; ++i){
		ret.add_circle_unchecked(Circle(ret.points[i], this->side/2));
	}
	ret.add_line_unchecked(Line::construct(ret.points[0], ret.points[1]));
	ret.add_line_unchecked(Line::construct(ret.points[0], ret.points[2]));
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
	const double rb = this->base/2;
	const double rs = this->side - rb;
	const double ra = 1/(2/rb + 1/rs + 2*std::sqrt(1/(rb*rb) + 2/(rb*rs)));
	const double ia = std::atan2(rs, rs + ra - std::sqrt(this->side*this->side - rb*rb));
	ret.points.push_back(this->c.rotate(this->angle, this->c + Point(rs + ra, 0)));
	const Point a = this->c + Point(rb + ra, 0);
	ret.points.push_back(this->c.rotate(this->angle + ia, a));
	ret.points.push_back(this->c.rotate(this->angle - ia, a));
	ret.init_topology();
	ret.add_circle_unchecked(Circle(ret.points[0], rs));
	ret.add_circle_unchecked(Circle(ret.points[1], rb));
	ret.add_circle_unchecked(Circle(ret.points[2], rb));
	ret.add_line_unchecked(Line::construct(ret.points[0], ret.points[1]));
	ret.add_line_unchecked(Line::construct(ret.points[0], ret.points[2]));
	ret.goal = Circle(this->c, ra);
	return ret;
}

RightFactory::RightFactory(double a, double b, Point &&c, double angle) : a(a), b(b), c(c), angle(angle) {}

ApproxCons RightFactory::operator()() const {
	ApproxCons ret(10, 2, 3, 6, 2);
	const double c = std::hypot(this->a, this->b);
	const double rx = (c + this->a - this->b)/2;
	const double ry = (this->b + c - this->a)/2;
	const double r0 = (this->a + this->b - c)/2;
	const double r = 1/(1/r0 + 1/rx + 1/ry + 2*std::sqrt(1/(r0*rx) + 1/(rx*ry) + 1/(ry*r0)));
	const double x = r*(a/rx + std::sqrt(2*a*b/(rx*ry)));
	const double y = r*(b/ry + std::sqrt(2*a*b/(rx*ry)));
	ret.points.push_back(this->c.rotate(this->angle, this->c + Point(-x, -y)));
	ret.points.push_back(this->c.rotate(this->angle, this->c + Point(this->a - x, -y)));
	ret.points.push_back(this->c.rotate(this->angle, this->c + Point(-x, this->b - y)));
	ret.init_topology();
	ret.add_circle_unchecked(Circle(ret.points[0], r0));
	ret.add_circle_unchecked(Circle(ret.points[1], rx));
	ret.add_circle_unchecked(Circle(ret.points[2], ry));
	ret.add_line_unchecked(Line::construct(ret.points[0], ret.points[1]));
	ret.add_line_unchecked(Line::construct(ret.points[0], ret.points[2]));
	ret.goal = Circle(this->c, r);
	return ret;
}

ScaleneFactory::ScaleneFactory(double a, double b, double c, Point &&p, double angle) : a(a), b(b), c(c), p(p), angle(angle) {}

ApproxCons ScaleneFactory::operator()() const {
	ApproxCons ret(12, 2, 3, 7, 2);
	const double rx = (this->c + this->a - this->b)/2;
	const double ry = (this->b + this->c - this->a)/2;
	const double r0 = (this->a + this->b - this->c)/2;
	const double C = std::acos((this->a*this->a + this->b*this->b - this->c*this->c)/(2*this->a*this->b));
	const double r = 1/(1/r0 + 1/rx + 1/ry + 2*std::sqrt(1/(r0*rx) + 1/(rx*ry) + 1/(ry*r0)));
	const auto [x, y] = reinterpret_cast<double(&&)[2]>(std::move(std::complex(
		r*(a/rx + b*std::exp(static_cast<std::complex<double>>(1i*C))/ry + 2.*std::sqrt(a*b*std::exp(static_cast<std::complex<double>>(1i*C))/(rx*ry)))
	)));
	const Point pc = this->p + Point(-x, -y);
	const Point pb = this->p + Point(this->a - x, -y);
	const Point pa = pc.rotate(C, this->p + Point(this->b - x, -y));
	ret.points.push_back(this->p.rotate(this->angle, pc));
	ret.points.push_back(this->p.rotate(this->angle, pb));
	ret.points.push_back(this->p.rotate(this->angle, pa));
	ret.init_topology();
	ret.add_circle_unchecked(Circle(ret.points[0], r0));
	ret.add_circle_unchecked(Circle(ret.points[0], rx));
	ret.add_circle_unchecked(Circle(ret.points[0], ry));
	ret.add_line_unchecked(Line::construct(ret.points[0], ret.points[1]));
	ret.add_line_unchecked(Line::construct(ret.points[0], ret.points[2]));
	ret.goal = Circle(this->p, r);
	return ret;
}

