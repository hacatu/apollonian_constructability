#ifndef _INIT_GEOMETRY_H__
#define _INIT_GEOMETRY_H__

#include <mpfi.h>
#include "geometry.h"

void init_geometry_222(ApproxCons *self);
void init_geometry_556(ApproxCons *self);
void init_geometry_345(ApproxCons *self);

void init_geometry_right(ApproxCons *self, mpfi_srcptr a, mpfi_srcptr b);
int init_geometry_scalene_lt120(ApproxCons *self, mpfi_srcptr a, mpfi_srcptr b, mpfi_srcptr c);

#endif

