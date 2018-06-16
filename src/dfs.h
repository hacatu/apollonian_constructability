#ifndef _DFS_H__
#define _DFS_H__

#include "geometry.h"

void dfs_cons(const ApproxCons *base, int (*candidate_cb)(const ApproxCons *base, void *data), void *cb_data, int depth);
void dfs_cons_parallel(const ApproxCons *base, int (*candidate_cb)(const ApproxCons *base, void *data), void *cb_data, int depth, int tid, int num_threads);

#endif

