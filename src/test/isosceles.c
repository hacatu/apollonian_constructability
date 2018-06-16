#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <pthread.h>
#include "geometry.h"
#include "dfs.h"
#include "construction.h"
#include "init_geometry.h"

int main(){
	Write_cb_data cb_data;
	/*
	int pipe_fd = init_write_cb_data(&cb_data);
	if(pipe_fd == -1){
		fprintf(stderr, "\e[1;31mERROR: Could not open pipe.\e[0m\n");
		exit(1);
	}
	*/
	if(!init_write_cb_data(&cb_data, "isosceles.dat")){
		fprintf(stderr, "\e[1;31mERROR: Could not open file.\e[0m\n");
		exit(1);
	}
	init_geometry_556(&cb_data.base);
	ApproxCons base = cb_data.base;
	/*
	pthread_t consumer_tid;
	if(pthread_create(&consumer_tid, NULL, print_candidates, (void*)(uintptr_t)pipe_fd)){
		fprintf(stderr, "\e[1;31mERROR: Could not create candidate processing thread.\e[0m\n");
		exit(1);
	}
	*/
	dfs_cons(&base, write_cb, &cb_data, 5);
	
	close(cb_data.fd);
	//pthread_join(consumer_tid, NULL);
}

