#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "geometry.h"

void init_geometry_222(ApproxCons *self){
	static const Point init_points[] = {
		{-1.5, 7.5}, {1.5, 7.5},
		{0, 5}, {0, 0},
		{-1.5, 2.5}, {1.5, 2.5},
		{-3, 0}, {3, 0},
		{-4.5, -2.5}, {4.5, -2.5}};
	memcpy(self->points, init_points, sizeof(init_points));
	self->points_len = LENGTHOF(init_points);
	static const Line init_lines[] = {
		{0, 5, .6, .8}, {0, 5, -.6, .8}};
	memcpy(self->lines, init_lines, sizeof(init_lines));
	self->lines_len = LENGTHOF(init_lines);
	static const Circle init_circles[] = {
		{0, 5, 2}, {-3, 0, 3}, {3, 0, 3}};
	memcpy(self->circles, init_circles, sizeof(init_circles));
	self->circles_len = LENGTHOF(init_circles);
	self->steps_len = 0;
	//initialize self->goal;
	self->goal = (Circle){0, 2.6, .4};
}

/* I want a more general dfs algorithm so I don't have to rewrite the
 * same steps every time.  If we're doing an n step search, step n is
 * drawing the final shape so we can just replace it with a check for
 * the preconditions of the shape.  These checks happen after step n - 1
 * but since they always happen we can put them in step n.  After step
 * n - 2 we need an object through the center, and before step n - 2
 * there are no hard constraints.
 */

int main(){
	
}

