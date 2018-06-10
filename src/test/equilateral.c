#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "geometry.h"
#include "dfs.h"
#include "construction.h"
#include "init_geometry.h"

int main(){
	ConsVec candidates = {malloc(4096*2*sizeof(ConsStep)), 0, 4096, 2};
	if(!candidates.constructions){
		fprintf(stderr, "\e[1;31mERROR: Could not allocate candidate solution buffer.\e[0m\n");
		exit(1);
	}
	ApproxCons base;
	init_geometry_222(&base);
	dfs_cons(&base, record_construction_cb, &candidates, 3);
	printf("Found %d 3-step constructions for the interior Appolonian circle of an equilateral set:\n", candidates.len);
	for(int i = 0; i < candidates.len; ++i){
		const ConsStep *cons = candidates.constructions + i*candidates.depth;
		printf(" %5d: %2d %2d %s\n", i, cons[0].i, cons[0].j,
			cons[0].type == GEOM_LINE ? "line" : "circle");
		printf("      : %2d %2d %s\n", cons[1].i, cons[1].j,
			cons[1].type == GEOM_LINE ? "line" : "circle");
	}
	free(candidates.constructions);
}

