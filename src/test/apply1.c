#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "geometry.h"
#include "dfs.h"

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
	//initialize self->goal;
	self->goal = (Circle){0, 1.6, .4};
}

int main(){
	ApproxCons base;
	init_geometry_556(&base);
	apply_construction(&base, 4, &(ConsStep[]){
		{0, 1, GEOM_LINE},
		{2, 3, GEOM_LINE},
		{8, 4, GEOM_CIRCLE},
		{0, 11, GEOM_LINE},
	});
}

