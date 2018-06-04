#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "geometry.h"
#include "dfs.h"

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
	//initialize self->goal;
	self->goal = (Circle){0, 0, 2/sqrt(3) - 1};
}

int main(){
	ConsVec candidates = {malloc(4096*2*sizeof(ConsStep)), 0, 4096, 2};
	if(!candidates.constructions){
		fprintf(stderr, "\e[1;31mERROR: Could not allocate candidate solution buffer.\e[0m\n");
		exit(1);
	}
	ApproxCons base;
	init_geometry_222(&base);
	dfs_cons(&base, &candidates, 3);
	printf("Found %d 3-step constructions for the interior Appolonian circle of an equilateral set:\n", candidates.len);
	for(int i = 0; i < candidates.len; ++i){
		const ConsStep *cons = candidates.constructions + i*candidates.depth;
		printf(" %5d: %2d %2d %s\n", i, cons[0].i, cons[0].j,
			cons[0].type == GEOM_LINE ? "line" : "circle");
		printf("      : %2d %2d %s\n", cons[1].i, cons[1].j,
			cons[1].type == GEOM_LINE ? "line" : "circle");
	}
	free(candidates.constructions);
}

