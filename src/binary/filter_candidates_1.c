#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include "construction.h"
#include "init_geometry.h"

int main(int argc, char **argv){
	if(argc != 3){
		printf("Please specify the input and output file names.\n");
		exit(1);
	}
	int fd = open(argv[1], O_RDONLY);
	if(fd == -1){
		printf("Could not open file \"%s\"\n", argv[1]);
		exit(1);
	}
	size_t len;
	XCons *candidates = read_file(fd, &len);
	close(fd);
	if(!candidates){
		printf("Could not read file \"%s\"\n", argv[1]);
		exit(1);
	}
	printf("Loaded %zu candidates\n", len);
	fd = open(argv[2], O_WRONLY | O_CREAT | O_TRUNC, 0777);
	if(fd == -1){
		printf("Could not open file \"%s\"\n", argv[2]);
		exit(1);
	}
	ApproxCons base;
	const Point *o = (const Point*)(const union{Point p; Circle c;}*)&base.goal;
	int c = 0;
	for(int i = 0; i < len; ++i){
		init_geometry_right(&base, 5, 12);
		//init_geometry_right(&base, 3, 4);
		//init_geometry_345(&base);
		if(!apply_xcons(&base, candidates + i)){
			continue;
		}else if(!eq_points(o, base.points + candidates[i].i_c)){
			continue;
		}else if(!is_on_circle(&base.goal, base.points + candidates[i].i_r)){
			continue;
		}
		/*
		init_geometry_scalene_lt120(&base, 30, 13, 37);
		if(!apply_xcons(&base, candidates + i)){
			continue;
		}else if(!eq_points(g, base.points + candidates[i].i_c)){
			continue;
		}else if(!is_on_circle(g, base.points + candidates[i].i_r)){
			continue;
		}
		*/
		init_geometry_scalene_lt120(&base, 40, 13, 37);
		if(!apply_xcons(&base, candidates + i)){
			continue;
		}else if(!eq_points(o, base.points + candidates[i].i_c)){
			continue;
		}else if(!is_on_circle(&base.goal, base.points + candidates[i].i_r)){
			continue;
		}
		write_xcons(fd, candidates + i);
		++c;
	}
	close(fd);
	printf("%d candidates passed the checks in this filter (5-12-13, 40-13-37)\n", c);
}

