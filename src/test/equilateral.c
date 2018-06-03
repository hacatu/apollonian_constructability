#include <stdio.h>
#include <stdlib.h>
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
	//initialize self->goal;
	self->goal = (Circle){0, 0, 2/sqrt(3) - 1};
}

int dfs_check_state(const ApproxCons *base, ConsVec *candidates){
	int center = 0, radial = 0;
	Point o = {base->goal.x, base->goal.y};
	int i = 0;
	for(; i < base->points_len; ++i){
		if(eq_points(&o, base->points + i)){
			center = 1;
			break;
		}
		if(is_on_circle(&base->goal, base->points + i)){
			radial = 1;
			break;
		}
	}
	if(radial){
		for(; i < base->points_len; ++i){
			if(eq_points(&o, base->points + i)){
				record_construction(candidates, base);
				return 1;
			}
		}
	}else if(center){
		for(; i < base->points_len; ++i){
			if(is_on_circle(&base->goal, base->points + i)){
				record_construction(candidates, base);
				return 1;
			}
		}
	}
	return 0;
}

void dfs_cons2(const ApproxCons *base, ConsVec *candidates){
	for(int i = 0; i < base->points_len - 1; ++i){
		const Point *restrict a = base->points + i;
		for(int j = i + 1; j < base->points_len; ++j){
			const Point *restrict b = base->points + j;
			Line l;
			construct_line(&l, a, b);
			ApproxCons next = *base;
			if(add_line_single(&next, &l)){
				record_step(&next, i, j, GEOM_LINE);
				dfs_check_state(&next, candidates);
			}
			Circle c;
			construct_circle(&c, a, b);
			next = *base;
			if(add_circle_single(&next, &c)){
				record_step(&next, i, j, GEOM_CIRCLE);
				dfs_check_state(&next, candidates);
			}
			construct_circle(&c, b, a);
			next = *base;
			if(add_circle_single(&next, &c)){
				record_step(&next, j, i, GEOM_CIRCLE);
				dfs_check_state(&next, candidates);
			}
		}
	}
}

void dfs_cons1(const ApproxCons *base, ConsVec *candidates){
	Point center = {base->goal.x, base->goal.y};
	for(int i = 0; i < base->points_len - 1; ++i){
		const Point *restrict a = base->points + i;
		for(int j = i + 1; j < base->points_len; ++j){
			const Point *restrict b = base->points + j;
			Line l;
			construct_line(&l, a, b);
			ApproxCons next;
			if(is_on_line(&l, &center)){
				next = *base;
				add_line_unchecked(&next, &l);
				remove_duplicate_points(&next, base->points_len);
				record_step(&next, i, j, GEOM_LINE);
				dfs_cons2(&next, candidates);
			}
			Circle c;
			construct_circle(&c, a, b);
			if(is_on_circle(&c, &center)){
				next = *base;
				add_circle_unchecked(&next, &c);
				remove_duplicate_points(&next, base->points_len);
				record_step(&next, i, j, GEOM_CIRCLE);
				dfs_cons2(&next, candidates);
			}
			construct_circle(&c, b, a);
			if(is_on_circle(&c, &center)){
				next = *base;
				add_circle_unchecked(&next, &c);
				remove_duplicate_points(&next, base->points_len);
				record_step(&next, j, i, GEOM_CIRCLE);
				dfs_cons2(&next, candidates);
			}
		}
	}
}

int main(){
	ConsVec candidates = {malloc(4096*2*sizeof(*candidates.constructions)), 0, 4096, 2};
	if(!candidates.constructions){
		fprintf(stderr, "\e[1;31mERROR: Could not allocate candidate solution buffer.\e[0m\n");
		exit(1);
	}
	ApproxCons base;
	init_geometry_222(&base);
	dfs_cons1(&base, &candidates);
	printf("Found %d 3-step constructions for the interior Appolonian circle of and equilateral set:\n", candidates.len);
	for(int i = 0; i < candidates.len; ++i){
		const ConsStep *cons = candidates.constructions + i*candidates.depth;
		printf(" %5d: %2d %2d %s\n", i, cons[0].i, cons[0].j,
			cons[0].type == GEOM_LINE ? "line" : "circle");
		printf("      : %2d %2d %s\n", cons[1].i, cons[1].j,
			cons[1].type == GEOM_LINE ? "line" : "circle");
	}
	free(candidates.constructions);
}

