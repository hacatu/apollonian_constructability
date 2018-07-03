#ifndef _INIT_GEOMETRY_HXX__
#define _INIT_GEOMETRY_HXX__

#include "geometry.hxx"

class ApproxConsFactory {
	public:
	virtual ApproxCons operator()() const = 0;
};

class EquilateralFactory : ApproxConsFactory {
	public:
	
	const double side;
	const Point c;
	const double angle;
	
	EquilateralFactory(double side, Point &&c, double angle);
	
	ApproxCons operator()() const;
};

class IsoscelesFactory : ApproxConsFactory {
	public:
	
	const double side, base;
	const Point c;
	const double angle;
	
	IsoscelesFactory(double side, double base, Point &&c, double angle);
	
	ApproxCons operator()() const;
};

#endif

