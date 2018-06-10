#ifndef _CONSTRUCTION_H__
#define _CONSTRUCTION_H__

#include "geometry.h"

#define MAX_XCONS_STEPS 112

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
	int len;
	int i_c, i_r;
	XConsStep construction[MAX_XCONS_STEPS];
} XCons;

typedef struct{
	ApproxCons base;
	int fd;
} Write_cb_data;

int apply_construction(ApproxCons *base, int steps_len, const ConsStep steps[static steps_len]);
int export_construction(XCons *out, ApproxCons *base, int steps_len, const ConsStep steps[static steps_len]);
int apply_xcons(ApproxCons *base, const XCons *steps);

int record_construction_cb(const ApproxCons *base, void *data);

int init_write_cb_data(Write_cb_data *self, const char *name);
int write_cb(const ApproxCons *approx, void *data);
void *print_candidates(void *data);

#endif

