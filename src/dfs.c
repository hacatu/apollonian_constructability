#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "geometry.h"
#include "dfs.h"

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

void dfs_cons__1(const ApproxCons *base, ConsVec *candidates){
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

void dfs_cons__2_unchecked(const ApproxCons *base, ConsVec *candidates){
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
				dfs_cons__1(&next, candidates);
			}
			Circle c;
			construct_circle(&c, a, b);
			if(is_on_circle(&c, &center)){
				next = *base;
				add_circle_unchecked(&next, &c);
				remove_duplicate_points(&next, base->points_len);
				record_step(&next, i, j, GEOM_CIRCLE);
				dfs_cons__1(&next, candidates);
			}
			construct_circle(&c, b, a);
			if(is_on_circle(&c, &center)){
				next = *base;
				add_circle_unchecked(&next, &c);
				remove_duplicate_points(&next, base->points_len);
				record_step(&next, j, i, GEOM_CIRCLE);
				dfs_cons__1(&next, candidates);
			}
		}
	}
}

void dfs_cons__2(const ApproxCons *base, ConsVec *candidates){
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
		dfs_cons__2_unchecked(base, candidates);
	}else for(int i = 0; i < base->points_len - 1; ++i){
		const Point *restrict a = base->points + i;
		for(int j = i + 1; j < base->points_len; ++j){
			const Point *restrict b = base->points + j;
			Line l;
			construct_line(&l, a, b);
			ApproxCons next;
			next = *base;
			if(add_line_single(&next, &l)){
				remove_duplicate_points(&next, base->points_len);
				record_step(&next, i, j, GEOM_LINE);
				dfs_cons__1(&next, candidates);
			}
			Circle c;
			construct_circle(&c, a, b);
			next = *base;
			if(add_circle_single(&next, &c)){
				remove_duplicate_points(&next, base->points_len);
				record_step(&next, i, j, GEOM_CIRCLE);
				dfs_cons__1(&next, candidates);
			}
			construct_circle(&c, b, a);
			next = *base;
			if(add_circle_single(&next, &c)){
				remove_duplicate_points(&next, base->points_len);
				record_step(&next, j, i, GEOM_CIRCLE);
				dfs_cons__1(&next, candidates);
			}
		}
	}
}

void dfs_cons(const ApproxCons *base, ConsVec *candidates, int depth){
	if(depth == 3){
		dfs_cons__2(base, candidates);
		return;
	}
	for(int i = 0; i < base->points_len - 1; ++i){
		if(depth == 5){
			printf("%d\n", i);
		}
		const Point *restrict a = base->points + i;
		for(int j = i + 1; j < base->points_len; ++j){
			if(depth == 5){
				printf(" %d\n", j);
			}
			const Point *restrict b = base->points + j;
			Line l;
			construct_line(&l, a, b);
			ApproxCons next = *base;
			if(add_line_single(&next, &l)){
				remove_duplicate_points(&next, base->points_len);
				record_step(&next, i, j, GEOM_LINE);
				dfs_cons(&next, candidates, depth - 1);
			}
			Circle c;
			construct_circle(&c, a, b);
			next = *base;
			if(add_circle_single(&next, &c)){
				remove_duplicate_points(&next, base->points_len);
				record_step(&next, i, j, GEOM_CIRCLE);
				dfs_cons(&next, candidates, depth - 1);
			}
			construct_circle(&c, b, a);
			next = *base;
			if(add_circle_single(&next, &c)){
				remove_duplicate_points(&next, base->points_len);
				record_step(&next, j, i, GEOM_CIRCLE);
				dfs_cons(&next, candidates, depth - 1);
			}
		}
	}
}

int apply_construction(ApproxCons *base, int steps_len, const ConsStep steps[static steps_len]){
	for(int i = 0; i < steps_len; ++i){
		int last_len = base->points_len;
		Line l;
		Circle c;
		if(steps[i].i >= last_len || steps[i].j >= last_len){
			return 0;
		}
		switch(steps[i].type){
		case GEOM_LINE:
			construct_line(&l, base->points + steps[i].i, base->points + steps[i].j);
			if(!add_line_single(base, &l)){
				return 0;
			}
			break;
		case GEOM_CIRCLE:
			construct_circle(&c, base->points + steps[i].i, base->points + steps[i].j);
			if(!add_circle_single(base, &c)){
				return 0;
			}
			break;
		case GEOM_COUNT:
			i = steps_len;
			break;
		default:
			return 0;
		}
		remove_duplicate_points(base, last_len);
	}
	return 1;
}

int record_xstep(XCons *self, int i1, int i2, int o1, int o2, GeomDepType type){
	if(self->len == self->cap){
		int new_cap = self->cap << 1;
		XConsStep *tmp = realloc(self->construction, new_cap*sizeof(XConsStep));
		if(!tmp){
			return 0;
		}
		self->construction = tmp;
		self->cap = new_cap;
	}
	self->construction[self->len++] = (XConsStep){i1, i2, o1, o2, type};
	return 1;
}

int export_construction(XCons *out, ApproxCons *base, int steps_len, const ConsStep steps[static steps_len]){
	for(int i = 0; i < steps_len; ++i){
		int last_len = base->points_len;
		Line l;
		Circle c;
		if(steps[i].i >= last_len || steps[i].j >= last_len){
			return 0;
		}
		switch(steps[i].type){
		case GEOM_LINE:
			construct_line(&l, base->points + steps[i].i, base->points + steps[i].j);
			if(!add_line_single(base, &l)){
				return 0;
			}
			if(!record_xstep(out, steps[i].i, steps[i].j, base->lines_len - 1, -1, GEOM_DEP_PPL)){
				return 0;
			}
			remove_duplicate_points(base, last_len);
			if(!record_new_line_points(out, base, last_len)){
				return 0;
			}
			break;
		case GEOM_CIRCLE:
			construct_circle(&c, base->points + steps[i].i, base->points + steps[i].j);
			if(!add_circle_single(base, &c)){
				return 0;
			}
			if(!record_xstep(out, steps[i].i, steps[i].j, base->circles_len - 1, -1, GEOM_DEP_PPC)){
				return 0;
			}
			remove_duplicate_points(base, last_len);
			if(!record_new_circle_points(out, base, last_len)){
				return 0;
			}
			break;
		case GEOM_COUNT:
			i = steps_len;
			break;
		default:
			return 0;
		}
	}
	identify_goal(out, base);
	return 1;
}

void identify_goal(XCons *self, const ApproxCons *base){
	double min_err_c = INFINITY, min_err_r = INFINITY;
	int min_err_c_i = -1, min_err_r_i = -1;
	for(int i = 0; i < base->points_len; ++i){
		double err = hypot(base->goal.x - base->points[i].x, base->goal.y - base->points[i].y);
		if(err < min_err_c){
			min_err_c = err;
			min_err_c_i = i;
		}
		err = fabs(err - base->goal.r);
		if(err < min_err_r){
			min_err_r = err;
			min_err_r_i = i;
		}
	}
	self->i_c = min_err_c_i;
	self->i_r = min_err_r_i;
}

int record_new_line_points(XCons *self, const ApproxCons *base, int last_len){
	int i = last_len;
	for(int l = 0; i < base->points_len && l < base->lines_len - 1; ++l){
		if(is_on_line(base->lines + l, base->points + i)){
			if(!record_xstep(self, l, base->lines_len - 1, i, -1, GEOM_DEP_LLP)){
				return 0;
			}
			++i;
		}
	}
	for(int c = 0; i < base->points_len && c < base->circles_len;){
		if(!is_on_circle(base->circles + c, base->points + i)){
			++c;
		}else{
			if(!record_xstep(self, c, base->lines_len - 1, i, -1, GEOM_DEP_CLP)){
				return 0;
			}
			++i;
		}
	}
	return 1;
}

int record_new_circle_points(XCons *self, const ApproxCons *base, int last_len){
	int i = last_len;
	for(int l = 0; i < base->points_len && l < base->lines_len;){
		if(!is_on_line(base->lines + l, base->points + i)){
			++l;
		}else{
			if(!record_xstep(self, base->circles_len - 1, l, i, -1, GEOM_DEP_CLP)){
				return 0;
			}
			++i;
		}
	}
	for(int c = 0; i < base->points_len && c < base->circles_len;){
		if(!is_on_circle(base->circles + c, base->points + i)){
			++c;
		}else{
			if(!record_xstep(self, base->circles_len - 1, c, i, -1, GEOM_DEP_CCP)){
				return 0;
			}
			++i;
		}
	}
	return 1;
}

