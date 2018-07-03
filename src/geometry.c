#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "geometry.h"

int intersect_LL(ApproxCons *self, const Line *restrict a, const Line *restrict b){
	double daxdb = a->dx*b->dy - a->dy*b->dx;
	if(fabs(daxdb) < EPSILON){
		return 0;
	}
	double ambx = a->x - b->x;
	double amby = a->y - b->y;
	double t = (amby*b->dx - ambx*b->dy)/daxdb;
	self->points[self->points_len++] = (Point){a->x + t*a->dx, a->y + t*a->dy};
	return 1;
}

int intersect_CC(ApproxCons *self, const Circle *restrict a, const Circle *restrict b){
	double d = hypot(b->x - a->x, b->y - a->y);
	if(d >= a->r + b->r + EPSILON || d <= fabs(b->r - a->r) - EPSILON){
		return 0;
	}else if(fabs(a->r + b->r - d) <= EPSILON || fabs(fabs(b->r - a->r) - d) <= EPSILON){//there is only one intersection
		if(a->r <= b->r && d < b->r + a->r - EPSILON){
			d = -d;
		}
		d = a->r/d;
		self->points[self->points_len++] = (Point){a->x + d*(b->x - a->x), a->y + d*(b->y - a->y)};
		return 1;
	}//otherwise there are two intersections
	double al = (a->r*a->r - b->r*b->r + d*d)/(2*d);
	double h = sqrt(a->r*a->r - al*al);
	al /= d;
	h /= d;
	double dx = b->y - a->y;
	double dy = b->x - a->x;
	double x = a->x + al*dy;
	double y = a->y + al*dx;
	dx *= h;
	dy *= h;
	self->points[self->points_len++] = (Point){x + dx, y - dy};
	self->points[self->points_len++] = (Point){x - dx, y + dy};
	return 2;
}

int intersect_CL(ApproxCons *self, const Circle *restrict C, const Line *restrict L){
	double d = (L->x - C->x)*(-L->dy) + (L->y - C->y)*L->dx;
	double ox = d*-L->dy;
	double oy = d*L->dx;
	d = fabs(d);
	if(d >= C->r + EPSILON){
		return 0;
	}else if(fabs(d - C->r) <= EPSILON){
		self->points[self->points_len++] = (Point){C->x + ox, C->y + oy};
		return 1;
	}//otherwise there are two intersections
	double h = d < EPSILON ? C->r : sqrt(C->r*C->r - d*d);
	double dx = h*L->dx;
	double dy = h*L->dy;
	ox += C->x;
	oy += C->y;
	self->points[self->points_len++] = (Point){ox + dx, oy + dy};
	self->points[self->points_len++] = (Point){ox - dx, oy - dy};
	return 2;
}

/* To begin with, we have 10 points and 5 geometric objects (p_0=10, g_0=5).
 * At each step, we can choose 2 points and then one of three ways to make a new
 * geometric object out of them.  This means g_(n+1) <= g_n + 1 and p_(n+1) <= p_n + 2*g_n.
 * So in the worst case, g_(n+1) = g_n + 1 and p_(n+1) = p_n + 2*n + 10.  We can solve this as
 * a system of homogenous linear equations:
 * [p_(n+1)]   [[1 2 0]][p_n]
 * [g_(n+1)] = [[0 1 1]][g_n]
 * [1      ]   [[0 0 1]][1  ]
 * 
 * [p_n]   [[1 2 0]]**n   [10]   [[1 2*n n*(n-1)]] [10]   [10 + 10*n + n*(n - 1)]
 * [g_n] = [[0 1 1]]    * [5 ] = [[0 1   n      ]]*[5 ] = [5 + n                ]
 * [1  ]   [[0 0 1]]      [1 ]   [[0 0   1      ]] [1 ]   [1                    ]
 * 
 * So p_n = n*(n - 1) + 10*n + 10 and g_n = n + 5.  The number of options for geometric objects
 * we can draw at step n + 1 is < 3*(p_n choose 2) - g_n, so the number of possible constructions of
 * length n is < (3*(p_0 choose 2) - g_0)* ... *(3*(p_(n-1) choose 2) - g_(n-1))
 * Notice 3*(p_n choose 2) - g_n = 3*(n*(n - 1) + 10*n + 10)*(n*(n - 1) + 10*n + 9)/2 - n - 5
 * = 3/2*n**4 + 27*n**3 + 150*n**2 + 511/2*n + 130
 * 
 * We can find the overall product using python:
 * import operator, functools
 * def constructions_ub(n):
 *     return functools.reduce(operator.mul, (((((3*k + 54)*k + 300)*k + 511)*k + 260)//2 for k in range(n)))
 * 
 * print(constructions_ub(6))#18038147914226419200
 * 
 * This is about 18 quintillion, which is pretty much intractible for brute forcing.
 * However, the last step must be drawing the circle, which requires the center point to have been found, which
 * means we must have two lines or circles through the center by step 5, so step 5 must be drawing a line or circle
 * through the center, and after step 4 we should already have a line or circle through the center.
 * We also need a point on the circle we draw in step 6.  In the construction we actually have due to Kontorovitch
 * which I think is optimal, this happens to be a point of tangency and I think it is likely a point of
 * tangency will usually or always provide the point on the circle.
 */

/* Our construction has to work for arbitrary circles with radii a, b, and c variable.  However, computing with
 * mathematical expressions is hugely more expensive than computing with floating point numbers, so we will
 * take advantage of the fact that if a construction works in general it has to work in any specific case and just
 * look at constructions that work on a specific scalene set of three tangent circles.  These will be checked to
 * see if they work in general.  Now I have to think about the data structures that will be used.
 * 
 * We need an iterable dictionary of circles and lines, and an iterable dictionary of points.  We need to determine
 * when we would be adding a duplicate circle, line, or point, which is a bit tricky since we will be using floating
 * point numbers which are not exact and so there might be some error.  For the points I think a quad tree is a good
 * choice, and it should be iterable.  We can also use a modified quadtree to store the circles, since we also need
 * to discriminate based on the radius, but the lines are a little trickier.  The points on the lines we use in the
 * Line data structure can't be used to discriminate lines, although their slope vectors dx, dy can.
 * Distance from the origin could be a good third coordinate.
 * 
 * So we will have
 * - a linked quadtree of points indexed in x, y space (maximum p_6 = 100 points)
 * - a linked quadtree of circles indexed in x, y, r space (maximum g_6 - 2 = 9 circles)
 * - a linked quadtree of lines indexed in dx, dy, d space (maximum g_6 - 3 = 8 lines)
 * 
 * Based on the small number of circles and lines, it may be best to not store them in special data structures at all
 * We also need to know the sequence of steps we took to get to a configuration, so that if it happens to be a valid
 * construction we can verify it using computer algebra.  One of the points that was used to create each shape is clear:
 * the one that shares its x and y coordinates.  Assuming that finding the points of intersection with a newly added
 * shape happens in a deterministic order, the points will be in a deterministic order.  So we just have to keep a record
 * of what steps we took, like i, j, GEOM_CIRCLE to mean a circle around the ith point through the jth point.  It
 * shouldn't matter if this record is efficiently indexed or if we can find the ith and jth points efficiently since
 * we will find very few valid configurations if we find any at all so looking at this record is not something that will
 * frequently happen, it just has to be possible.
 */

int eq_lines(const Line *restrict a, const Line *restrict b){
	if(fabs(a->dx*b->dx + a->dy*b->dy) < 1 - EPSILON){
		return 0;
	}
	double dx = b->x - a->x;
	double dy = b->y - a->y;
	return fabs(dx*-a->dy + dy*a->dx) < EPSILON;
}

int eq_circles(const Circle *restrict a, const Circle *restrict b){
	double dx = b->x - a->x;
	double dy = b->y - a->y;
	if(dx*dx + dy*dy > EPSILON){
		return 0;
	}
	return fabs(a->r - b->r) < EPSILON;
}

int eq_points(const Point *restrict a, const Point *restrict b){
	double dx = b->x - a->x;
	double dy = b->y - a->y;
	return dx*dx + dy*dy < EPSILON;
}

void add_line_unchecked(ApproxCons *self, const Line *restrict a){
	for(int i = 0; i < self->lines_len; ++i){
		intersect_LL(self, a, self->lines + i);
	}
	for(int i = 0; i < self->circles_len; ++i){
		intersect_CL(self, self->circles + i, a);
	}
	self->lines[self->lines_len++] = *a;
}

int add_line_single(ApproxCons *self, const Line *restrict a){
	for(int i = 0; i < self->lines_len; ++i){
		if(eq_lines(a, self->lines + i)){
			return 0;
		}
	}
	add_line_unchecked(self, a);
	return 1;
}

void add_circle_unchecked(ApproxCons *self, const Circle *restrict a){
	for(int i = 0; i < self->lines_len; ++i){
		intersect_CL(self, a, self->lines + i);
	}
	for(int i = 0; i < self->circles_len; ++i){
		intersect_CC(self, a, self->circles + i);
	}
	self->circles[self->circles_len++] = *a;
}

int add_circle_single(ApproxCons *self, const Circle *restrict a){
	for(int i = 0; i < self->circles_len; ++i){
		if(eq_circles(a, self->circles + i)){
			return 0;
		}
	}
	add_circle_unchecked(self, a);
	return 1;
}

//removing points like this could make it impossible to recover the steps from the ConsSteps
void remove_duplicate_points(ApproxCons *self, int prev_len){
	for(int i = 0; i < prev_len; ++i){
		const Point *restrict a = self->points + i;
		for(int j = prev_len; j < self->points_len; ++j){
			const Point *restrict b = self->points + j;
			if(eq_points(a, b)){
				if(j == self->points_len - 1){
					self->points_len = j;
				}else{
					self->points[j--] = self->points[--self->points_len];
				}
			}
		}
	}
	for(int i = prev_len; i + 1 < self->points_len; ++i){
		const Point *restrict a = self->points + i;
		for(int j = i + 1; j < self->points_len; ++j){
			const Point *restrict b = self->points + j;
			if(eq_points(a, b)){
				if(j == self->points_len - 1){
					self->points_len = j;
				}else{
					self->points[j--] = self->points[--self->points_len];
				}
			}
		}
	}
}

void construct_line(Line *out, const Point *restrict a, const Point *restrict b){
	double dx = b->x - a->x;
	double dy = b->y - a->y;
	double h = hypot(dx, dy);
	dx /= h;
	dy /= h;
	*out = (Line){a->x, a->y, dx, dy};
}

void construct_circle(Circle *out, const Point *restrict o, const Point *restrict r){
	*out = (Circle){o->x, o->y, hypot(r->x - o->x, r->y - o->y)};
}

void record_step(ApproxCons *self, int i, int j, GeomType type){
	self->steps[self->steps_len++] = (ConsStep){i, j, type};
}

int is_on_line(const Line *l, const Point *p){
	double dx = p->x - l->x;
	double dy = p->y - l->y;
	return fabs(dx*-l->dy + dy*l->dx) < EPSILON;
}

int is_on_circle(const Circle *c, const Point *p){
	return fabs(c->r - hypot(p->x - c->x, p->y - c->y)) < EPSILON;
}

int record_construction(ConsVec *candidates, const ApproxCons *base){
	if(candidates->len == candidates->cap){
		int new_cap = candidates->cap << 1;
		void *tmp = realloc(candidates->constructions, new_cap*candidates->depth*sizeof(ConsStep));
		if(!tmp){
			fprintf(stderr, "\e[1;31mERROR: Out of memory!\e[0m\n");
			exit(1);
			return 0;
		}
		candidates->constructions = tmp;
		candidates->cap = new_cap;
	}
	memcpy(candidates->constructions + candidates->len*candidates->depth, base->steps, base->steps_len*sizeof(ConsStep));
	if(base->steps_len < candidates->depth){
		candidates->constructions[candidates->len*candidates->depth + base->steps_len].type = GEOM_COUNT;
	}
	++candidates->len;
	return 1;
}

