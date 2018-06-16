#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include "geometry.h"
#include "dfs.h"
#include "construction.h"
#include "init_geometry.h"

int main(){
	Write_cb_data cb_data;
	if(!init_write_cb_data(&cb_data, "scalene.dat")){
		fprintf(stderr, "\e[1;31mERROR: Could not open file.\e[0m\n");
		exit(1);
	}
	init_geometry_345(&cb_data.base);
	ApproxCons base = cb_data.base;
	dfs_cons(&base, write_cb, &cb_data, 6);
	close(cb_data.fd);
}

