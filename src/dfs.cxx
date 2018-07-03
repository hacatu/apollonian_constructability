#include <cmath>
#include <vector>
#include <algorithm>
#include <functional>
#include "geometry.hxx"
#include "dfs.hxx"

DFS::Resources::Resources(size_t steps, size_t arb_points) : steps(steps), arb_points(arb_points) {}
DFS::Conditions::Conditions(DFS &controller) : controller(controller) {}
bool DFS::Conditions::operator()(const ApproxCons &base){
	switch(controller.resources.steps){
	case 0:
		if(std::find(base.points.begin(), base.points.end(), base.goal.c) != base.points.end()){
			if(std::find_if(base.points.begin(), base.points.end(), std::bind(&Circle::contains, base.goal, std::placeholders::_1)) != base.points.end()){
				controller.results.push_back(base.steps);
			}
		}
		return false;
	case 1:
		return std::find_if(base.lines.begin(), base.lines.end(), std::bind(&Line::contains, std::placeholders::_1, base.goal.c)) != base.lines.end()
			|| std::find_if(base.circles.begin(), base.circles.end(), std::bind(&Circle::contains, std::placeholders::_1, base.goal.c)) != base.circles.end();
	}
	return true;
}

DFS::DFS(DFS::Resources &&resources) : resources(resources), conditions(*this) {}
DFS::DFS(size_t steps, size_t arb_points) : resources(steps, arb_points), conditions(*this) {}
DFS::DFS(const ApproxCons &base) : DFS(base.limits.n, 2*(base.limits.n - base.limits.m)) {}

void DFS::do_step(ApproxCons &base){
	if(!this->conditions(base)){
		return;
	}
	const ApproxCons::Progress progress = base.getProgress();
	const DFS::Resources start_res(this->resources);
	for(size_t i = 0; i < base.points.size() - 1; ++i){
		for(size_t j = i + 1; j < base.points.size(); ++j){
			base.resetProgress(progress);
			this->resources = start_res;
			base.add_line(Line::construct(base.points[i], base.points[j]));
			base.record_step({i, j, ApproxCons::Step::PPL});
			--this->resources.steps;
			do_step(base);
			
			base.resetProgress(progress);
			this->resources = start_res;
			base.add_circle(Circle::construct(base.points[i], base.points[j]));
			base.record_step({i, j, ApproxCons::Step::PPC});
			--this->resources.steps;
			do_step(base);
			
			base.resetProgress(progress);
			this->resources = start_res;
			base.add_circle(Circle::construct(base.points[j], base.points[i]));
			base.record_step({j, i, ApproxCons::Step::PPC});
			--this->resources.steps;
			do_step(base);
		}
	}
}

