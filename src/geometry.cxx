#include <cmath>
#include <vector>
#include <array>
#include <algorithm>
#include <stdexcept>
#include "geometry.hxx"

ApproxCons::Progress::Progress(size_t points_len, size_t lines_len, size_t circles_len, size_t lsegs_len, size_t csegs_len, size_t regions, size_t steps_len) :
	points_len(points_len), lines_len(lines_len), circles_len(circles_len), lsegs_len(lsegs_len), csegs_len(csegs_len), regions(regions), steps_len(steps_len)
{}

ApproxCons::Progress::Progress(const ApproxCons &base) :
	ApproxCons::Progress(base.points.size(), base.lines.size(), base.circles.size(), base.line_segments.size(), base.circle_segments.size(), base.regions, base.steps.size())
{}

ApproxCons::Step::Step(size_t i, size_t j, ApproxCons::Step::Type type) : i(i), j(j), type(type) {}

ApproxCons::ApproxCons(Limits &&limits) : regions(0), limits(limits) {
	this->points.reserve(this->limits.max_points());
	this->lines.reserve(this->limits.max_lines());
	this->circles.reserve(this->limits.max_circles());
	this->line_segments.reserve(this->limits.max_lsegs());
	this->circle_segments.reserve(this->limits.max_csegs());
	this->p_ls_adj.reserve(this->limits.max_lsegs());
	this->p_cs_adj.reserve(this->limits.max_csegs());
	this->p_region_adj.reserve(this->limits.max_regions());
	this->ls_region_adj.reserve(this->limits.max_regions());
	this->cs_region_adj.reserve(this->limits.max_regions());
	this->steps.reserve(this->limits.n);
}

ApproxCons::ApproxCons(size_t p0, size_t l0, size_t c0, size_t n, size_t m) : ApproxCons(ApproxCons::Limits(p0, l0, c0, n, m)) {}

void ApproxCons::init_topology(){
	p_region_adj.emplace_back(this->points.size(), true);
	this->regions = 1;
}

ApproxCons::Progress ApproxCons::getProgress(){
	return Progress(*this);
}

void ApproxCons::resetProgress(const ApproxCons::Progress &progress){
	this->points.resize(progress.points_len);
	this->lines.resize(progress.lines_len);
	this->circles.resize(progress.circles_len);
	this->line_segments.resize(progress.lsegs_len);
	this->circle_segments.resize(progress.csegs_len);
	this->regions = progress.regions;
	this->p_ls_adj.resize(progress.lsegs_len);
	if(progress.points_len){
		this->p_ls_adj[0].resize(progress.points_len);
	}
	this->p_cs_adj.resize(progress.csegs_len);
	if(progress.points_len){
		this->p_cs_adj[0].resize(progress.points_len);
	}
	this->p_region_adj.resize(progress.regions);
	if(progress.points_len){
		this->p_region_adj[0].resize(progress.points_len);
	}
	this->ls_region_adj.resize(progress.regions);
	if(progress.lsegs_len){
		this->ls_region_adj[0].resize(progress.lsegs_len);
	}
	this->cs_region_adj.resize(progress.regions);
	if(progress.csegs_len){
		this->cs_region_adj[0].resize(progress.csegs_len);
	}
	this->steps.resize(progress.steps_len);
}

bool ApproxCons::add_line(Line &&a){
	if(std::find(this->lines.begin(), this->lines.end(), a) != this->lines.end()){
		return false;
	}
	add_line_unchecked(std::move(a));
	return true;
}

void ApproxCons::add_line_unchecked(Line &&a){
	size_t old_len = this->points.size();
	for(const Line &b : this->lines){
		size_t start_len = this->points.size();
		intersect(a, b);
		remove_duplicate_points(old_len, start_len);
	}
	for(const Circle &b : this->circles){
		size_t start_len = this->points.size();
		intersect(a, b);
		remove_duplicate_points(old_len, start_len);
	}
	this->lines.push_back(a);
}

bool ApproxCons::add_circle(Circle &&a){
	if(std::find(this->circles.begin(), this->circles.end(), a) != this->circles.end()){
		return false;
	}
	add_circle_unchecked(std::move(a));
	return true;
}

void ApproxCons::add_circle_unchecked(Circle &&a){
	size_t old_len = this->points.size();
	for(const Line &b : this->lines){
		size_t start_len = this->points.size();
		intersect(a, b);
		remove_duplicate_points(old_len, start_len);
	}
	for(const Circle &b : this->circles){
		size_t start_len = this->points.size();
		intersect(a, b);
		remove_duplicate_points(old_len, start_len);
	}
	this->circles.push_back(a);
}

void ApproxCons::add_arbitrary_lseg_point(size_t seg){
	Line::Segment &sa = this->line_segments[seg];
	const Point p = sa.a->unit_combination(std::cbrt(.25), *sa.b);
	this->points.push_back(p);
	this->line_segments.emplace_back(*sa.whole, this->points.back(), *sa.b);
	sa.b = std::addressof(this->points.back());
	const Line::Segment &sb = this->line_segments.back();
	this->p_ls_adj.emplace_back(this->points.size());
	this->p_ls_adj.back().reserve(this->limits.max_points());
	for(size_t i = 0; i < this->points.size() - 1; ++i){
		if(std::addressof(this->points[i]) == sb.b){
			this->p_ls_adj[seg][i] = false;
			this->p_ls_adj.back()[i] = true;
			break;
		}
	}
	this->p_cs_adj[seg].back() = this->p_cs_adj.back().back() = true;
	for(size_t i = 0; i < this->regions; ++i){
		this->p_region_adj[i][seg] = this->ls_region_adj[i].back() = this->ls_region_adj[i][seg];
	}
}

void ApproxCons::add_arbitrary_cseg_point(size_t seg){
	Circle::Segment &sa = this->circle_segments[seg];
	double angle = std::asin((*sa.a - sa.whole->c).cross(*sa.b - sa.whole->c)/(sa.whole->r*sa.whole->r));
	if(angle < EPSILON){
		angle = M_2_PI;
	}
	const Point p = sa.whole->c.rotate(angle/3, *sa.a);
	this->points.push_back(p);
	this->circle_segments.emplace_back(*sa.whole, this->points.back(), *sa.b);
	sa.b = std::addressof(this->points.back());
	const Circle::Segment &sb = this->circle_segments.back();
	this->p_cs_adj.emplace_back(this->points.size());
	this->p_cs_adj.back().reserve(this->limits.max_points());
	for(size_t i = 0; i < this->points.size() - 1; ++i){
		if(std::addressof(this->points[i]) == sb.b){
			this->p_cs_adj[seg][i] = false;
			this->p_cs_adj.back()[i] = true;
			break;
		}
	}
	this->p_cs_adj[seg].back() = this->p_cs_adj.back().back() = true;
	for(size_t i = 0; i < this->regions; ++i){
		this->p_region_adj[i][seg] = this->cs_region_adj[i].back() = this->cs_region_adj[i][seg];
	}
}

void ApproxCons::add_arbitrary_region_point(size_t reg){
	auto ait = std::find(this->cs_region_adj[reg].begin(), this->cs_region_adj[reg].end(), true);
	Point p;
	if(ait == this->cs_region_adj[reg].end()){
		auto begin = this->p_region_adj[reg].begin(), end = this->p_region_adj[reg].end(), pit = begin;
		const Point &ps[3];
		for(size_t i = 0; i < 3; ++i){
			pit = std::find(pit, end, true);
			ps[i] = this->points[pit++ - begin];
		}
		double t = (1 - std::cbrt(.25))/2;
		p = ps->unit_combination(t, ps[1], t, ps[2]);
	}else{//so we really need to cache some data about each region because this isn't going to be efficient
		//namely I think adjacency lists would be better than adjacency matrices since they obviate searching
		//and make the order of neighbors obvious.  For regions, it would be best to know some stuff like
		//an arbitrary point in them.  For convex regions, we can just take a linear combination of the corners
		//with manhattan norm one, but for concave regions things get dumb.  One solution is to store a diameter
		//of one of the circles which is a third of the way around some concave arc, find the first of its
		//intersections with the other boundary shapes projected onto it, and pick the point cbrt(.25) between
		//them.  There's a small problem though: I can't immediately tell which side of the arc is the inside.
		//
		
	}
	this->points.push_back(p);
	for(std::vector<bool> &adj_pts : this->p_ls_adj){
		adj_pts.push_back(false);
	}
	for(std::vector<bool> &adj_pts : this->p_cs_adj){
		adj_pts.push_back(false);
	}
	for(size_t i = 0; i < this->p_region_adj.size(); ++i){
		this->p_region_adj[i].push_back(i == reg);
	}
}

void ApproxCons::remove_duplicate_points(size_t old_len, size_t start_len){
	for(auto b = this->points.begin() + start_len; b < this->points.end();){
		if(std::find(this->points.begin(), this->points.begin() + old_len, *b) != this->points.begin() + old_len){
			b = this->points.erase(b);
			continue;
		}
		++b;
	}
}

void ApproxCons::record_step(Step &&step){
	this->steps.push_back(step);
}

std::vector<Point> intersect(const Line &a, const Line &b){
	double daxdb = a.d.cross(b.d);
	if(std::fabs(daxdb) < EPSILON){
		return std::vector{};
	}
	double t = b.d.cross(a - b)/daxdb;
	return std::vector{a.p + t*a.d};
}

std::vector<Point> intersect(const Circle &a, const Line &b){
	double d = (l.p - c.c)*l.d.ccw();
	Point o = d*l.d.ccw();
	d = std::fabs(d);
	if(d >= c.r + EPSILON){
		return std::vector{};
	}else if(std::fabs(d - c.r) <= EPSILON){
		return std::vector{c.c + o};
	}//otherwise there are two intersections
	double h = d < EPSILON ? c.r : std::sqrt(c.r*c.r - d*d);
	const Point f = h*l.d;
	o = o + c.c;
	return std::vector{o + f, o - f};
}

std::vector<Point> intersect(const Circle &a, const Circle &b){
	double d = b.c.distance(a.c);
	if(d >= a.r + b.r + EPSILON || d <= std::fabs(b.r - a.r) - EPSILON){
		return std::vector{};
	}
	Point f = (b.c - a.c).transpose();
	if(std::fabs(a.r + b.r - d) <= EPSILON || std::fabs(std::fabs(b.r - a.r) - d) <= EPSILON){//there is only one intersection
		if(a.r <= b.r && d < b.r + a.r - EPSILON){
			d = -d;
		}
		d = a.r/d;
		return std::vector{a.c + d*f};
	}//otherwise there are two intersections
	double al = (a.r*a.r - b.r*b.r + d*d)/(2*d);
	double h = std::sqrt(a.r*a.r - al*al);
	al /= d;
	h /= d;
	const Point o = a.c + al*f.transpose();
	f = h*f;
	return std::vector{o + f, o - f};
}

