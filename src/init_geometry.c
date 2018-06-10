#include <string.h>
#include <math.h>
#include "geometry.h"

void init_geometry_222(ApproxCons *self){
	static const Point init_points[] = {
		{-.5, 7/sqrt(12)}, {-.5, 7/sqrt(12)},
		{0, 2/sqrt(3)}, {0, -1/sqrt(3)},
		{-.5, 1/sqrt(12)}, {.5, 1/sqrt(12)},
		{-1, -1/sqrt(3)}, {1, -1/sqrt(3)},
		{-1.5, -5/sqrt(12)}, {1.5, -5/sqrt(12)}};
	memcpy(self->points, init_points, sizeof(init_points));
	self->points_len = LENGTHOF(init_points);
	static const Line init_lines[] = {
		{0, 1, 1./2, sqrt(3)/2}, {0, 1, -1./2, sqrt(3)/2}};
	memcpy(self->lines, init_lines, sizeof(init_lines));
	self->lines_len = LENGTHOF(init_lines);
	static const Circle init_circles[] = {
		{0, 2/sqrt(3), 1}, {-1, -1/sqrt(3), 1}, {1, -1/sqrt(3), 1}};
	memcpy(self->circles, init_circles, sizeof(init_circles));
	self->circles_len = LENGTHOF(init_circles);
	self->steps_len = 0;
	self->goal = (Circle){0, 0, 2/sqrt(3) - 1};
}

void init_geometry_556(ApproxCons *self){
	static const Point init_points[] = {
		{-1.2, 5.6}, {1.2, 5.6},
		{0, 4}, {0, 0},
		{-1.2, 2.4}, {1.2, 2.4},
		{-3, 0}, {3, 0},
		{-4.8, -2.4}, {4.8, -2.4}};
	memcpy(self->points, init_points, sizeof(init_points));
	self->points_len = LENGTHOF(init_points);
	static const Line init_lines[] = {
		{0, 4, .6, .8}, {0, 4, -.6, .8}};
	memcpy(self->lines, init_lines, sizeof(init_lines));
	self->lines_len = LENGTHOF(init_lines);
	static const Circle init_circles[] = {
		{0, 4, 2}, {-3, 0, 3}, {3, 0, 3}};
	memcpy(self->circles, init_circles, sizeof(init_circles));
	self->circles_len = LENGTHOF(init_circles);
	self->steps_len = 0;
	self->goal = (Circle){0, 1.6, .4};
}

void init_geometry_345(ApproxCons *self){
	static const Point init_points[] = {{0, 7}, {0, 4}, {-1, 0}, {0, 0}, {1, 0}, {5, 0}, {0, -1}, {1.8, 1.6}};
	memcpy(self->points, init_points, sizeof(init_points));
	self->points_len = LENGTHOF(init_points);
	static const Line init_lines[] = {{0, 0, 0, 1}, {0, 0, 1, 0}};
	memcpy(self->lines, init_lines, sizeof(init_lines));
	self->lines_len = LENGTHOF(init_lines);
	static const Circle init_circles[] = {{0, 0, 1}, {3, 0, 2}, {0, 4, 3}};
	memcpy(self->circles, init_circles, sizeof(init_circles));
	self->circles_len = LENGTHOF(init_circles);
	self->steps_len = 0;
	self->goal = (Circle){21/23., 20/23., 6/23.};
}

