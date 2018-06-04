#ifndef _DFS_H__
#define _DFS_H__

#include "geometry.h"

typedef enum{
	GEOM_DEP_PPL,
	GEOM_DEP_PPC,
	GEOM_DEP_LLP,
	GEOM_DEP_CLP,
	GEOM_DEP_CCP,
	GEOM_DEP_COUNT,
} GeomDepType;

typedef struct{
	int i1, i2, o1, o2;
	GeomDepType type;
} XConsStep;

typedef struct{
	XConsStep *construction;
	int len, cap;
	int i_c, i_r;
} XCons;

void dfs_cons(const ApproxCons *base, ConsVec *candidates, int depth);
int apply_construction(ApproxCons *base, int steps_len, const ConsStep steps[static steps_len]);
int record_xstep(XCons *self, int i1, int i2, int o1, int o2, GeomDepType type);
int export_construction(XCons *out, ApproxCons *base, int steps_len, const ConsStep steps[static steps_len]);
void identify_goal(XCons *self, const ApproxCons *base);
int record_new_line_points(XCons *self, const ApproxCons *base, int last_len);
int record_new_circle_points(XCons *self, const ApproxCons *base, int last_len);

#endif

