#ifndef _GEOMETRY_H__
#define _GEOMETRY_H__

#include <mpfi.h>

#define MAX_CONS_STEPS 6
#define MAX_CONS_LINES 8
#define MAX_CONS_CIRCLES 9
#define MAX_CONS_POINTS 100

#define LENGTHOF(a) (sizeof(a)/sizeof(*a))

#define EPSILON 0.0000005

typedef struct{
	mpfi_t x, y;
} Point;

typedef struct{
	mpfi_t x, y, dx, dy;
} Line;

typedef struct{
	mpfi_t x, y, r;
} Circle;

typedef enum{
	GEOM_LINE,
	GEOM_CIRCLE,
	GEOM_COUNT,
} GeomType;

typedef struct{
	int i, j;
	GeomType type;
} ConsStep;

typedef struct{
	ConsStep *constructions;
	int len, cap, depth;
} ConsVec;

typedef struct{
	Point points[MAX_CONS_POINTS];
	Line lines[MAX_CONS_LINES];
	Circle circles[MAX_CONS_CIRCLES];
	ConsStep steps[MAX_CONS_STEPS];
	int points_len, lines_len, circles_len, steps_len;
	Circle goal;
	mpfi_t scratch[6];
} ApproxCons;

int intersect_LL(ApproxCons *self, const Line *restrict a, const Line *restrict b);
int intersect_CC(ApproxCons *self, const Circle *restrict a, const Circle *restrict b);
int intersect_CL(ApproxCons *self, const Circle *restrict C, const Line *restrict L);

int eq_lines(ApproxCons *self, const Line *restrict a, const Line *restrict b);
int eq_circles(ApproxCons *self, const Circle *restrict a, const Circle *restrict b);
int eq_points(ApproxCons *self, const Point *restrict a, const Point *restrict b);
int is_on_line(ApproxCons *self, const Line *l, const Point *p);
int is_on_circle(ApproxCons *self, const Circle *c, const Point *p);

void add_line_unchecked(ApproxCons *self, const Line *restrict a);
int add_line_single(ApproxCons *self, const Line *restrict a);
void add_circle_unchecked(ApproxCons *self, const Circle *restrict a);
int add_circle_single(ApproxCons *self, const Circle *restrict a);
void construct_line(ApproxCons *self, Line *out, const Point *restrict a, const Point *restrict b);
void construct_circle(ApproxCons *self, Circle *out, const Point *restrict o, const Point *restrict r);

void remove_duplicate_points(ApproxCons *self, int prev_len);

void record_step(ApproxCons *self, int i, int j, GeomType type);
int record_construction(ConsVec *candidates, const ApproxCons *base);

#endif

