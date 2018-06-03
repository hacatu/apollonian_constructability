#ifndef _GEOMETRY_H__
#define _GEOMETRY_H__

#define LENGTHOF(a) (sizeof(a)/sizeof(*a))

#define EPSILON 0.0000005

typedef struct{
	double x, y;
} Point;

typedef struct{
	double x, y, dx, dy;
} Line;

typedef struct{
	double x, y, r;
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
	Point points[100];
	Line lines[8];
	Circle circles[9];
	ConsStep steps[6];
	int points_len, lines_len, circles_len, steps_len;
	Circle goal;
} ApproxCons;

int intersect_LL(ApproxCons *self, const Line *restrict a, const Line *restrict b);
int intersect_CC(ApproxCons *self, const Circle *restrict a, const Circle *restrict b);
int intersect_CL(ApproxCons *self, const Circle *restrict C, const Line *restrict L);

void init_geometry_345(ApproxCons *self);

int eq_lines(const Line *restrict a, const Line *restrict b);
int eq_circles(const Circle *restrict a, const Circle *restrict b);
int eq_points(const Point *restrict a, const Point *restrict b);
int is_on_line(const Line *l, const Point *p);
int is_on_circle(const Circle *c, const Point *p);

void add_line_unchecked(ApproxCons *self, const Line *restrict a);
int add_line_single(ApproxCons *self, const Line *restrict a);
void add_circle_unchecked(ApproxCons *self, const Circle *restrict a);
int add_circle_single(ApproxCons *self, const Circle *restrict a);
void construct_line(Line *out, const Point *restrict a, const Point *restrict b);
void construct_circle(Circle *out, const Point *restrict o, const Point *restrict r);

void remove_duplicate_points(ApproxCons *self, int prev_len);

void record_step(ApproxCons *self, int i, int j, GeomType type);
int record_construction(ConsVec *candidates, const ApproxCons *base);

#endif

