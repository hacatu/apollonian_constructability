#include <string.h>
#include <math.h>
#include <mpfr.h>
#include <mpfi.h>
#include "geometry.h"

#define EMPLACE_MPFI(d, p) ({mpfi_t v; mpfi_init2(&v, p); mpfi_set_d(&v, d); v;})
#define EMPLACE_POINT(x, y) {EMPLACE_MPFI(x, PRECISION), EMPLACE_MPFI(y, PRECISION)}
#define EMPLACE_LINE(x, y, dx, dy) {EMPLACE_MPFI(x, PRECISION), EMPLACE_MPFI(y, PRECISION), EMPLACE_MPFI(dx, PRECISION), EMPLACE_MPFI(dy, PRECISION)}
#define EMPLACE_CIRCLE(x, y, r) {EMPLACE_MPFI(x, PRECISION), EMPLACE_MPFI(y, PRECISION), EMPLACE_MPFI(r, PRECISION)}

void approx_cons_init(ApproxCons *self){
	self->points_len = 0;
	self->steps_len = 0;
	for(int i = 0; i < 6; ++i){
		mpfi_init2(self->scratch[i], PRECISION);
	}
}

void init_geometry_222(ApproxCons *self){
	approx_cons_init(self);
	static const Point init_points[] = {
		EMPLACE_POINT(-.5, 3.5/sqrt(3)), EMPLACE_POINT(.5, 3.5/sqrt(3)),
		EMPLACE_POINT(0, 2/sqrt(3)), EMPLACE_POINT(0, -1/sqrt(3)),
		EMPLACE_POINT(-.5, .5/sqrt(3)), EMPLACE_POINT(.5, .5/sqrt(3)),
		EMPLACE_POINT(-1, -1/sqrt(3)), EMPLACE_POINT(1, -1/sqrt(3)),
		EMPLACE_POINT(-1.5, -2.5/sqrt(3)), EMPLACE_POINT(1.5, -2.5/sqrt(3))};
	memcpy(self->points, init_points, sizeof(init_points));
	self->points_len = LENGTHOF(init_points);
	static const Line init_lines[] = {
		EMPLACE_LINE(0, 1, .5, sqrt(3)/2), EMPLACE_LINE(0, 1, -.5, sqrt(3)/2)};
	memcpy(self->lines, init_lines, sizeof(init_lines));
	self->lines_len = LENGTHOF(init_lines);
	static const Circle init_circles[] = {
		EMPLACE_CIRCLE(0, 2/sqrt(3), 1), EMPLACE_CIRCLE(-1, -1/sqrt(3), 1), EMPLACE_CIRCLE(1, -1/sqrt(3), 1)};
	memcpy(self->circles, init_circles, sizeof(init_circles));
	self->circles_len = LENGTHOF(init_circles);
	static const Circle goal[1] = {EMPLACE_CIRCLE(0, 0, 2/sqrt(3) - 1)};
	self->goal = goal[0];
}

void init_geometry_556(ApproxCons *self){
	approx_cons_init(self);
	static const Point init_points[] = {
		EMPLACE_POINT(-1.2, 5.6), EMPLACE_POINT(1.2, 5.6),
		EMPLACE_POINT(0, 4), EMPLACE_POINT(0, 0),
		EMPLACE_POINT(-1.2, 2.4), EMPLACE_POINT(1.2, 2.4),
		EMPLACE_POINT(-3, 0), EMPLACE_POINT(3, 0),
		EMPLACE_POINT(-4.8, -2.4), EMPLACE_POINT(4.8, -2.4)};
	memcpy(self->points, init_points, sizeof(init_points));
	self->points_len = LENGTHOF(init_points);
	static const Line init_lines[] = {
		EMPLACE_LINE(0, 4, .6, .8), EMPLACE_LINE(0, 4, -.6, .8)};
	memcpy(self->lines, init_lines, sizeof(init_lines));
	self->lines_len = LENGTHOF(init_lines);
	static const Circle init_circles[] = {
		EMPLACE_CIRCLE(0, 4, 2), EMPLACE_CIRCLE(-3, 0, 3), EMPLACE(3, 0, 3)};
	memcpy(self->circles, init_circles, sizeof(init_circles));
	self->circles_len = LENGTHOF(init_circles);
	static const Circle goal[1] = {EMPLACE_CIRCLE(0, 1.6, .4)};
	self->goal = goal[0];
}

void init_geometry_345(ApproxCons *self){
	approx_cons_init(self);
	static const Point init_points[] = {
		EMPLACE_POINT(0, 7), EMPLACE_POINT(0, 4),
		EMPLACE_POINT(0, 1), EMPLACE_POINT(0, 0),
		EMPLACE_POINT(0, -1), EMPLACE_POINT(-1, 0),
		EMPLACE_POINT(1, 0), EMPLACE_POINT(3, 0),
		EMPLACE_POINT(5, 0), EMPLACE_POINT(1.8, 1.6)};
	memcpy(self->points, init_points, sizeof(init_points));
	self->points_len = LENGTHOF(init_points);
	static const Line init_lines[] = {
		EMPLACE_LINE(0, 0, 0, 1), EMPLACE_LINE(0, 0, 1, 0)};
	memcpy(self->lines, init_lines, sizeof(init_lines));
	self->lines_len = LENGTHOF(init_lines);
	static const Circle init_circles[] = {
		EMPLACE_CIRCLE(0, 0, 1), EMPLACE_CIRCLE(3, 0, 2), EMPLACE_CIRCLE(0, 4, 3)};
	memcpy(self->circles, init_circles, sizeof(init_circles));
	self->circles_len = LENGTHOF(init_circles);
	static const Circle goal[1] = {EMPLACE_CIRCLE(21/23., 20/23., 6/23.)}
	self->goal = goal[0];
}

void init_geometry_right(ApproxCons *self, double a, double b){
	double c = hypot(a, b);
	double r0 = (+a + b - c)/2;
	double rx = (+a - b + c)/2;
	double ry = (-a + b + c)/2;
	const Point init_points[] = {{0, b + ry}, {0, b}, {0, r0}, {0, 0}, {0, -r0}, {-r0, 0}, {r0, 0}, {a, 0}, {a + rx, 0}, {(1 - rx/c)*a, rx/c*b}};
	memcpy(self->points, init_points, sizeof(init_points));
	self->points_len = LENGTHOF(init_points);
	static const Line init_lines[] = {{0, 0, 0, 1}, {0, 0, 1, 0}};
	memcpy(self->lines, init_lines, sizeof(init_lines));
	self->lines_len = LENGTHOF(init_lines);
	const Circle init_circles[] = {{0, 0, r0}, {a, 0, rx}, {0, b, ry}};
	memcpy(self->circles, init_circles, sizeof(init_circles));
	self->circles_len = LENGTHOF(init_circles);
	self->steps_len = 0;
	double r = 1/(1/r0 + 1/rx + 1/ry + 2*sqrt(1/(r0*rx) + 1/(rx*ry) + 1/(ry*r0)));
	double x = r*(a/rx + sqrt(2*a*b/(rx*ry)));
	double y = r*(b/ry + sqrt(2*a*b/(rx*ry)));
	self->goal = (Circle){x, y, r};
}

int init_geometry_scalene_lt120(ApproxCons *self, double a, double b, double c){
	double C = acos((a*a + b*b - c*c)/(2*a*b));
	double r0 = (+a + b - c)/2;
	double rx = (+a - b + c)/2;
	double ry = (-a + b + c)/2;
	if(a*sin(C) <= rx || b*sin(C) <= ry){
		return 0;
	}
	const Point init_points[] = {
		{cos(C)*(b + ry), sin(C)*(b + ry)},
		{cos(C)*b, sin(C)*b},
		{cos(C)*r0, sin(C)*r0}, {0, 0}, {-cos(C)*r0, -sin(C)*r0},
		{-r0, 0}, {r0, 0}, {a, 0}, {a + rx, 0},
		{a - (a + r0)*rx/c, sin(C)*b*rx/c}};
	memcpy(self->points, init_points, sizeof(init_points));
	self->points_len = LENGTHOF(init_points);
	const Line init_lines[] = {{0, 0, cos(C), sin(C)}, {0, 0, 1, 0}};
	memcpy(self->lines, init_lines, sizeof(init_lines));
	self->lines_len = LENGTHOF(init_lines);
	const Circle init_circles[] = {{0, 0, r0}, {a, 0, rx}, {cos(C)*b, sin(C)*b, ry}};
	memcpy(self->circles, init_circles, sizeof(init_circles));
	self->circles_len = LENGTHOF(init_circles);
	self->steps_len = 0;
	double r = 1/(1/r0 + 1/rx + 1/ry + 2*sqrt(1/(r0*rx) + 1/(rx*ry) + 1/(ry*r0)));
	double complex z = r*(a/rx + b*cexp(I*C)/ry + 2*csqrt(a*b*cexp(I*C)/(rx*ry)));
	self->goal = (Circle){creal(z), cimag(z), r};
	return 1;
}

