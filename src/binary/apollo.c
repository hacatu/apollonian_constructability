#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <fcntl.h>
#include <pthread.h>
#include "geometry.h"
#include "dfs.h"
#include "construction.h"
#include "init_geometry.h"

#define NUM_THREADS 6

typedef struct{
	ApproxCons base;
	pthread_mutex_t *mutex;
	int fd;
	int tid;
	int num_threads;
} Task;

int filter_write_cb(const ApproxCons *approx, void *data){
	Task *task = data;
	XCons out = {};
	out.len = 0;
	export_construction(&out, &task->base, approx->steps_len, approx->steps);
	pthread_mutex_lock(task->mutex);
	int ret = write_xcons(task->fd, &out);
	pthread_mutex_unlock(task->mutex);
	return ret;
}

void *run_task(void *_task){
	Task *task = _task;
	dfs_cons_parallel(&task->base, filter_write_cb, _task, 6, task->tid, task->num_threads);
	return NULL;
}

int main(){
	int fd = open("apollo.dat", O_WRONLY | O_CREAT | O_TRUNC, 0777);
	if(fd == -1){
		fprintf(stderr, "\e[1;31mERROR: Could not open file.\e[0m\n");
		exit(1);
	}
	ApproxCons base;
	init_geometry_345(&base);
	pthread_mutex_t fd_lock = PTHREAD_MUTEX_INITIALIZER;
	Task thread_args[NUM_THREADS];
	pthread_t thread_ids[NUM_THREADS];
	for(int i = 0; i < NUM_THREADS; ++i){
		thread_args[i] = (Task){.base=base, .mutex=&fd_lock, .fd=fd, .tid=i, .num_threads=NUM_THREADS};
		if(pthread_create(thread_ids + i, NULL, run_task, thread_args + i)){
			fprintf(stderr, "\e[1;31mERROR: Could not create thread.\e[0m\n");
			exit(1);
		}
	}
	for(int i = 0; i < NUM_THREADS; ++i){
		if(pthread_join(thread_ids[i], NULL)){
			fprintf(stderr, "\e[1;31mERROR: Thread exploded.\e[0m\n");
			exit(1);
		}
	}
	close(fd);
	pthread_mutex_destroy(&fd_lock);
}

