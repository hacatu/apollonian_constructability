#include <iostream>
#include <cmath>
#include <vector>
#include <algorithm>
#include <functional>
#include "geometry.hxx"
#include "dfs.hxx"
#include "init_geometry.hxx"

int main(){
	ApproxCons base = EquilateralFactory(2, Point(0, 0), 0)();
	DFS dfs_controller(base);
	dfs_controller.do_step(base);
	std::cout << dfs_controller.results.size() << std::endl;
}

