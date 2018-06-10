#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "geometry.h"
#include "dfs.h"
#include "construction.h"
#include "init_geometry.h"

int main(){
	ApproxCons base;
	init_geometry_556(&base);
	apply_construction(&base, 4, (const ConsStep[]){
		{0, 1, GEOM_LINE},
		{2, 3, GEOM_LINE},
		{8, 4, GEOM_CIRCLE},
		{0, 11, GEOM_LINE},
	});
}

