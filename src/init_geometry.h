#ifndef _INIT_GEOMETRY_H__
#define _INIT_GEOMETRY_H__

#include "geometry.h"

void init_geometry_222(ApproxCons *self);
void init_geometry_556(ApproxCons *self);
void init_geometry_345(ApproxCons *self);

void init_geometry_right(ApproxCons *self, double a, double b);
int init_geometry_scalene_lt120(ApproxCons *self, double a, double b, double c);

#endif

