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
	ConsVec candidates = {malloc(4096*4*sizeof(ConsStep)), 0, 4096, 4};
	if(!candidates.constructions){
		fprintf(stderr, "\e[1;31mERROR: Could not allocate candidate solution buffer.\e[0m\n");
		exit(1);
	}
	ApproxCons base;
	init_geometry_556(&base);
	dfs_cons(&base, &candidates, 5);
	printf("Found %d 5-step constructions for the interior Appolonian circle of an isosceles set:\n", candidates.len);
	for(int i = 0; i < candidates.len; ++i){
		const ConsStep *cons = candidates.constructions + i*candidates.depth;
		printf(" %5d: %2d %2d %s\n", i, cons[0].i, cons[0].j,
			cons[0].type == GEOM_LINE ? "line" : "circle");
		for(int j = 1; j < 4; ++j){
			printf("      : %2d %2d %s\n", cons[j].i, cons[j].j,
				cons[j].type == GEOM_LINE ? "line" : "circle");
		}
	}
	free(candidates.constructions);
}

