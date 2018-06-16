#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "geometry.h"
#include "dfs.h"

int dfs_check_state(ApproxCons *base, int (*candidate_cb)(const ApproxCons *base, void *data), void *cb_data, int last_len){
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
				remove_duplicate_points(base, last_len);
				candidate_cb(base, cb_data);
				return 1;
			}
		}
	}else if(center){
		for(; i < base->points_len; ++i){
			if(is_on_circle(&base->goal, base->points + i)){
				remove_duplicate_points(base, last_len);
				candidate_cb(base, cb_data);
				return 1;
			}
		}
	}
	return 0;
}

void dfs_cons__1(const ApproxCons *base, int(*candidate_cb)(const ApproxCons *base, void *data), void *cb_data){
	for(int i = 0; i < base->points_len - 1; ++i){
		const Point *restrict a = base->points + i;
		for(int j = i + 1; j < base->points_len; ++j){
			const Point *restrict b = base->points + j;
			Line l;
			construct_line(&l, a, b);
			ApproxCons next = *base;
			if(add_line_single(&next, &l)){
				record_step(&next, i, j, GEOM_LINE);
				dfs_check_state(&next, candidate_cb, cb_data, base->points_len);
			}
			Circle c;
			construct_circle(&c, a, b);
			next = *base;
			if(add_circle_single(&next, &c)){
				record_step(&next, i, j, GEOM_CIRCLE);
				dfs_check_state(&next, candidate_cb, cb_data, base->points_len);
			}
			construct_circle(&c, b, a);
			next = *base;
			if(add_circle_single(&next, &c)){
				record_step(&next, j, i, GEOM_CIRCLE);
				dfs_check_state(&next, candidate_cb, cb_data, base->points_len);
			}
		}
	}
}

void dfs_cons__2_unchecked(const ApproxCons *base, int (*candidate_cb)(const ApproxCons *base, void *data), void *cb_data){
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
				dfs_cons__1(&next, candidate_cb, cb_data);
			}
			Circle c;
			construct_circle(&c, a, b);
			if(is_on_circle(&c, &center)){
				next = *base;
				add_circle_unchecked(&next, &c);
				remove_duplicate_points(&next, base->points_len);
				record_step(&next, i, j, GEOM_CIRCLE);
				dfs_cons__1(&next, candidate_cb, cb_data);
			}
			construct_circle(&c, b, a);
			if(is_on_circle(&c, &center)){
				next = *base;
				add_circle_unchecked(&next, &c);
				remove_duplicate_points(&next, base->points_len);
				record_step(&next, j, i, GEOM_CIRCLE);
				dfs_cons__1(&next, candidate_cb, cb_data);
			}
		}
	}
}

void dfs_cons__2(const ApproxCons *base, int (*candidate_cb)(const ApproxCons *base, void *data), void *cb_data){
	int center_geom = 0;
	Point center = {base->goal.x, base->goal.y};
	for(int i = 0; i < base->lines_len; ++i){
		if(is_on_line(base->lines + i, &center)){
			center_geom = 1;
			break;
		}
	}
	if(!center_geom){
		for(int i = 0; i < base->circles_len; ++i){
			if(is_on_circle(base->circles + i, &center)){
				center_geom = 1;
				break;
			}
		}
	}
	if(!center_geom){
		dfs_cons__2_unchecked(base, candidate_cb, cb_data);
	}else for(int i = 0; i < base->points_len - 1; ++i){
		const Point *restrict a = base->points + i;
		for(int j = i + 1; j < base->points_len; ++j){
			const Point *restrict b = base->points + j;
			Line l;
			construct_line(&l, a, b);
			if(is_on_line(&l, &center)){
				
			}
			ApproxCons next;
			next = *base;
			if(add_line_single(&next, &l)){
				remove_duplicate_points(&next, base->points_len);
				record_step(&next, i, j, GEOM_LINE);
				dfs_cons__1(&next, candidate_cb, cb_data);
			}
			Circle c;
			construct_circle(&c, a, b);
			next = *base;
			if(add_circle_single(&next, &c)){
				remove_duplicate_points(&next, base->points_len);
				record_step(&next, i, j, GEOM_CIRCLE);
				dfs_cons__1(&next, candidate_cb, cb_data);
			}
			construct_circle(&c, b, a);
			next = *base;
			if(add_circle_single(&next, &c)){
				remove_duplicate_points(&next, base->points_len);
				record_step(&next, j, i, GEOM_CIRCLE);
				dfs_cons__1(&next, candidate_cb, cb_data);
			}
		}
	}
}

void dfs_cons(const ApproxCons *base, int (*candidate_cb)(const ApproxCons *base, void *data), void *cb_data, int depth){
	if(depth == 3){
		dfs_cons__2(base, candidate_cb, cb_data);
		return;
	}
	for(int i = 0; i < base->points_len - 1; ++i){
		const Point *restrict a = base->points + i;
		for(int j = i + 1; j < base->points_len; ++j){
			const Point *restrict b = base->points + j;
			Line l;
			construct_line(&l, a, b);
			ApproxCons next = *base;
			if(add_line_single(&next, &l)){
				remove_duplicate_points(&next, base->points_len);
				record_step(&next, i, j, GEOM_LINE);
				dfs_cons(&next, candidate_cb, cb_data, depth - 1);
			}
			Circle c;
			construct_circle(&c, a, b);
			next = *base;
			if(add_circle_single(&next, &c)){
				remove_duplicate_points(&next, base->points_len);
				record_step(&next, i, j, GEOM_CIRCLE);
				dfs_cons(&next, candidate_cb, cb_data, depth - 1);
			}
			construct_circle(&c, b, a);
			next = *base;
			if(add_circle_single(&next, &c)){
				remove_duplicate_points(&next, base->points_len);
				record_step(&next, j, i, GEOM_CIRCLE);
				dfs_cons(&next, candidate_cb, cb_data, depth - 1);
			}
		}
	}
}

void dfs_cons_parallel(const ApproxCons *base, int (*candidate_cb)(const ApproxCons *base, void *data), void *cb_data, int depth, int tid, int num_threads){
	if(depth == 3){
		dfs_cons__2(base, candidate_cb, cb_data);
		return;
	}
	for(int i = 0; i < base->points_len - 1; ++i){
		const Point *restrict a = base->points + i;
		for(int j = i + 1 + ((base->points_len - 1)*i - (i - 1)*i/2 + tid)%num_threads; j < base->points_len; j += num_threads){
			const Point *restrict b = base->points + j;
			Line l;
			construct_line(&l, a, b);
			ApproxCons next = *base;
			if(add_line_single(&next, &l)){
				remove_duplicate_points(&next, base->points_len);
				record_step(&next, i, j, GEOM_LINE);
				dfs_cons(&next, candidate_cb, cb_data, depth - 1);
			}
			Circle c;
			construct_circle(&c, a, b);
			next = *base;
			if(add_circle_single(&next, &c)){
				remove_duplicate_points(&next, base->points_len);
				record_step(&next, i, j, GEOM_CIRCLE);
				dfs_cons(&next, candidate_cb, cb_data, depth - 1);
			}
			construct_circle(&c, b, a);
			next = *base;
			if(add_circle_single(&next, &c)){
				remove_duplicate_points(&next, base->points_len);
				record_step(&next, j, i, GEOM_CIRCLE);
				dfs_cons(&next, candidate_cb, cb_data, depth - 1);
			}
		}
	}
}

