#ifndef _DFS_H__
#define _DFS_H__

#include "geometry.h"

void dfs_cons(const ApproxCons *base, int (*candidate_cb)(const ApproxCons *base, void *data), void *cb_data, int depth);

#endif

