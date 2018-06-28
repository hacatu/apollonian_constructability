#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <inttypes.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <mpfr.h>
#include <mpfi.h>
#include "geometry.h"
#include "construction.h"

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
			construct_line(base, &l, base->points + steps[i].i, base->points + steps[i].j);
			if(!add_line_single(base, &l)){
				return 0;
			}
			break;
		case GEOM_CIRCLE:
			construct_circle(base, &c, base->points + steps[i].i, base->points + steps[i].j);
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

int apply_xcons(ApproxCons *base, const XCons *steps){
	ApproxCons pbuf;
	approx_cons_init(&pbuf);
	for(int i = 0; i < steps->len; ++i){
		int last_len = base->points_len, n = 0;
		Line l;
		Circle c;
		const XConsStep *step = steps->construction + i;
		int i1 = step->i1, i2 = step->i2, o1 = step->o1, o2 = step->o2, type = step->type;
		if(i1 >= last_len || i2 >= last_len){
			return 0;
		}
		switch(type){
		case GEOM_DEP_PPL:
			construct_line(&pbuf, &l, base->points + i1, base->points + i2);
			base->lines[o1] = l;
			if(o1 + 1 > base->lines_len){
				base->lines_len = o1 + 1;
			}
			break;
		case GEOM_DEP_PPC:
			construct_circle(&pbuf, &c, base->points + i1, base->points + i2);
			base->circles[o1] = c;
			if(o1 + 1 > base->circles_len){
				base->circles_len = o1 + 1;
			}
			break;
		case GEOM_DEP_LLP:
			pbuf.points_len = 0;
			if(!intersect_LL(&pbuf, base->lines + i1, base->lines + i2)){
				return 0;
			}
			base->points[o1] = pbuf.points[0];
			if(o1 + 1 > base->points_len){
				base->points_len = o1 + 1;
			}
			break;
		case GEOM_DEP_CLP:
			pbuf.points_len = 0;
			n = intersect_CL(&pbuf, base->circles + i1, base->lines + i2);
			if((o2 != -1 && n != 2) || !n){
				return 0;
			}
			if(o1 != -1){
				base->points[o1] = pbuf.points[0];
				if(o1 + 1 > base->points_len){
					base->points_len = o1 + 1;
				}
			}
			if(o2 != -1){
				base->points[o2] = pbuf.points[1];
				if(o2 + 1 > base->points_len){
					base->points_len = o2 + 1;
				}
			}
			break;
		case GEOM_DEP_CCP:
			pbuf.points_len = 0;
			n = intersect_CC(&pbuf, base->circles + i1, base->circles + i2);
			if((o2 != -1 && n != 2) || !n){
				return 0;
			}
			if(o1 != -1){
				base->points[o1] = pbuf.points[0];
				if(o1 + 1 > base->points_len){
					base->points_len = o1 + 1;
				}
			}
			if(o2 != -1){
				base->points[o2] = pbuf.points[1];
				if(o2 + 1 > base->points_len){
					base->points_len = o2 + 1;
				}
			}
			break;
		case GEOM_DEP_COUNT:
			return 1;
		default:
			return 0;
		}
	}
	return 1;
}

int record_xstep(XCons *self, int i1, int i2, int o1, int o2, GeomDepType type){
	self->construction[self->len++] = (XConsStep){i1, i2, o1, o2, type};
	return 1;
}

void identify_goal(XCons *self, ApproxCons *base){
	//min_err_c = INFINITY, min_err_r = INFINITY
	mpfi_interv_d(base->scratch + 0, INFINITY, INFINITY);
	mpfi_interv_d(base->scratch + 1, INFINITY, INFINITY);
	int min_err_c_i = -1, min_err_r_i = -1;
	for(int i = 0; i < base->points_len; ++i){
		//err = hypot(base->goal.x - base->points[i].x, base->goal.y - base->points[i].y)
		mpfi_sub(base->scratch + 2, base->goal.x - base->points[i].x);
		mpfi_sub(base->scratch + 3, base->goal.y - base->points[i].y);
		mpfi_hypot(base->scratch + 2, base->scratch + 2, base->scratch + 3);
		//if(err < min_err_c) min_err_c = err, min_err_c_i = i
		if(mpfr_cmp(base->scratch[2]->left, base->scratch[0]->left) < -1){
			mpfi_set(base->scratch + 0, base->scratch + 2);
			min_err_c_i = i;
		}
		//err = fabs(err - base->goal.r)
		mpfi_sub(base->scratch + 2, base->scratch + 2, base->goal.r);
		mpfi_abs(base->scratch + 2, base->scratch + 2);
		//if(err < min_err_r) min_err_r = err, min_err_r_i = i
		if(mpfr_cmp(base->scratch[2]->left, base->scratch[1]->left) < 0){
			mpfi_set(base->scratch + 1, base->scratch + 2);
			min_err_r_i = i;
		}
	}
	self->i_c = min_err_c_i;
	self->i_r = min_err_r_i;
}

int record_new_line_points(XCons *self, const ApproxCons *base, int last_len){
	for(int l = 0; l < base->lines_len - 1; ++l){
		ApproxCons out;
		approx_cons_init(&out);
		if(intersect_LL(&out, base->lines + l, base->lines + base->lines_len - 1)){
			for(int i = last_len; i < base->points_len; ++i){
				if(eq_points(&out, out.points, base->points + i)){
					if(!record_xstep(self, l, base->lines_len - 1, i, -1, GEOM_DEP_LLP)){
						return 0;
					}
					break;
				}
			}
		}
	}
	for(int c = 0; c < base->circles_len; ++c){
		ApproxCons out;
		approx_cons_init(&out);
		if(intersect_CL(&out, base->circles + c, base->lines + base->lines_len - 1)){
			int o1 = -1, o2 = -1, i;
			for(i = last_len; i < base->points_len; ++i){
				if(o1 == -1 && eq_points(&out, out.points, base->points + i)){
					o1 = i;
				}else if(out.points_len == 2 && o2 == -1 && eq_points(&out, out.points + 1, base->points + i)){
					o2 = i;
				}
			}
			if(o1 != -1 || o2 != -1){
				if(!record_xstep(self, c, base->lines_len - 1, o1, o2, GEOM_DEP_CLP)){
					return 0;
				}
			}
		}
	}
	return 1;
}

int record_new_circle_points(XCons *self, const ApproxCons *base, int last_len){
	for(int l = 0; l < base->lines_len; ++l){
		ApproxCons out;
		approx_cons_init(&out);
		if(intersect_CL(&out, base->circles + base->circles_len - 1, base->lines + l)){
			int o1 = -1, o2 = -1, i;
			for(i = last_len; i < base->points_len; ++i){
				if(o1 == -1 && eq_points(&out, out.points, base->points + i)){
					o1 = i;
				}else if(out.points_len == 2 && o2 == -1 && eq_points(&out, out.points + 1, base->points + i)){
					o2 = i;
				}
			}
			if(o1 != -1 || o2 != -1){
				if(!record_xstep(self, base->circles_len - 1, l, o1, o2, GEOM_DEP_CLP)){
					return 0;
				}
			}
		}
	}
	for(int c = 0; c < base->circles_len - 1; ++c){
		ApproxCons out;
		approx_cons_init(&out);
		if(intersect_CC(&out, base->circles + base->circles_len - 1, base->circles + c)){
			int o1 = -1, o2 = -1, i;
			for(i = last_len; i < base->points_len; ++i){
				if(o1 == -1 && eq_points(&out, out.points, base->points + i)){
					o1 = i;
				}else if(out.points_len == 2 && o2 == -1 && eq_points(&out, out.points + 1, base->points + i)){
					o2 = i;
				}
			}
			if(o1 != -1 || o2 != -1){
				if(!record_xstep(self, base->circles_len - 1, c, o1, o2, GEOM_DEP_CCP)){
					return 0;
				}
			}
		}
	}
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
			construct_line(base, &l, base->points + steps[i].i, base->points + steps[i].j);
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
			construct_circle(base, &c, base->points + steps[i].i, base->points + steps[i].j);
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

int record_construction_cb(const ApproxCons *base, void *data){
	return record_construction(data, base);
}

int init_write_cb_data(Write_cb_data *self, const char *name){
	/*
	int fds[2];
	if(pipe(fds) == -1){
		return -1;
	}
	self->fd = fds[1];
	init_geometry_556(&self->base);
	return fds[0];
	*/
	self->fd = open(name, O_WRONLY | O_CREAT | O_TRUNC, 0777);
	return self->fd != -1;
}

int write_xcons(int fd, const XCons *self){
	const void *buf = self;
	size_t nbyte = offsetof(XCons, construction) + self->len*sizeof(XConsStep);
	while(nbyte){
		ssize_t len = write(fd, buf, nbyte);
		while(len == -1){
			switch(errno){
			case EINTR:
			case EAGAIN:
				len = write(fd, buf, nbyte);
				continue;
			default:
				return 0;
			}
		}
		nbyte -= len;
		buf += len;
	}
	return 1;
}

int write_cb(const ApproxCons *approx, void *data){
	XCons out;
	out.len = 0;
	ApproxCons base = ((const Write_cb_data*)data)->base;
	export_construction(&out, &base, approx->steps_len, approx->steps);
	return write_xcons(((const Write_cb_data*)data)->fd, &out);
}

XCons *read_file(int fd, size_t *_len){
	size_t len = 0, cap = 1024;
	XCons *candidates = malloc(cap*sizeof(XCons));
	if(!candidates){
		return NULL;
	}
	for(int n = 1;; ++n){
		XCons construction;
		void *buf = &construction;
		for(size_t nbyte = offsetof(XCons, construction); nbyte;){
			ssize_t len = read(fd, buf, nbyte);
			while(len == -1){
				switch(errno){
				case EINTR:
				case EAGAIN:
					len = read(fd, buf, nbyte);
					continue;
				default:
					free(candidates);
					return NULL;
				}
			}
			if(!len){
				candidates = realloc(candidates, n*sizeof(XCons)) ?: candidates;
				*_len = n;
				return candidates;
			}
			nbyte -= len;
			buf += len;
		}
		for(size_t nbyte = construction.len*sizeof(XConsStep); nbyte;){
			ssize_t len = read(fd, buf, nbyte);
			while(len == -1){
				switch(errno){
				case EINTR:
				case EAGAIN:
					len = read(fd, buf, nbyte);
					continue;
				default:
					free(candidates);
					return NULL;
				}
			}
			if(!len){
				free(candidates);
				return NULL;
			}
			nbyte -= len;
			buf += len;
		}
		if(len == cap){
			size_t new_cap = cap << 1;
			XCons *tmp = realloc(candidates, new_cap*sizeof(XCons));
			if(!tmp){
				free(candidates);
				return NULL;
			}
			candidates = tmp;
			cap = new_cap;
		}
		candidates[len++] = construction;
	}
}

void *print_candidates(void *data){
	int fd = (int)(uintptr_t)data;
	for(int n = 1;; ++n){
		XCons construction;
		void *buf = &construction;
		for(size_t nbyte = offsetof(XCons, construction); nbyte;){
			ssize_t len = read(fd, buf, nbyte);
			while(len == -1){
				switch(errno){
				case EINTR:
				case EAGAIN:
					len = read(fd, buf, nbyte);
					continue;
				default:
					return NULL;
				}
			}
			if(!len){
				return NULL;
			}
			nbyte -= len;
			buf += len;
		}
		for(size_t nbyte = construction.len*sizeof(XConsStep); nbyte;){
			ssize_t len = read(fd, buf, nbyte);
			while(len == -1){
				switch(errno){
				case EINTR:
				case EAGAIN:
					len = read(fd, buf, nbyte);
					continue;
				default:
					return NULL;
				}
			}
			if(!len){
				return NULL;
			}
			nbyte -= len;
			buf += len;
		}
		printf("%d:\n", n);
		for(int i = 0; i < construction.len; ++i){
			XConsStep step = construction.construction[i];
			if(step.type == GEOM_DEP_LLP){
				printf("L%d L%d -> P%d\n", step.i1, step.i2, step.o1);
			}else if(step.type == GEOM_DEP_PPL){
				printf("P%d P%d -> L%d\n", step.i1, step.i2, step.o1);
			}else if(step.type == GEOM_DEP_PPC){
				printf("P%d P%d -> C%d\n", step.i1, step.i2, step.o1);
			}else{
				if(step.type == GEOM_DEP_CLP){
					printf("C%d L%d -> ", step.i1, step.i2);
				}else{
					printf("C%d C%d -> ", step.i1, step.i2);
				}
				if(step.o1 == -1){
					printf("-- ");
				}else{
					printf("P%d ", step.o1);
				}
				if(step.o2 == -1){
					printf("--\n");
				}else{
					printf("P%d\n", step.o2);
				}
			}
		}
		printf("c_%d r_%d\n", construction.i_c, construction.i_r);
	}
}

