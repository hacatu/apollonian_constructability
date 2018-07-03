#ifndef _DFS_HXX__
#define _DFS_HXX__

#include <vector>
#include "geometry.hxx"

class DFS {
	public:
	class Resources {
		public:
		size_t steps, arb_points;
		
		Resources(size_t steps, size_t arb_points);
	};
	
	class Conditions {
		public:
		DFS &controller;
		
		Conditions(DFS &controller);
		
		virtual bool operator()(const ApproxCons &base);
	};
	
	Resources resources;
	Conditions conditions;
	std::vector<std::vector<ApproxCons::Step>> results;
	
	DFS(Resources &&resources);
	DFS(size_t steps, size_t arb_points);
	DFS(const ApproxCons &base);
	
	void do_step(ApproxCons &base);
};

#endif

