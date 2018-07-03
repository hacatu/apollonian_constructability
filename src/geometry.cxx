#include <cmath>
#include <vector>
#include <algorithm>
#include <stdexcept>
#include "geometry.hxx"

Point::Point(double x, double y) : x(x), y(y) {}

bool Point::operator==(const Point &other) const {
	double dx = other.x - this->x;
	double dy = other.y - this->y;
	return dx*dx + dy*dy < EPSILON;
}

const Point Point::operator+(const Point &other) const {
	return Point(this->x + other.x, this->y + other.y);
}

const Point Point::rotate(double angle, const Point &other) const {
	double x = other.x - this->x;
	double y = other.y - this->y;
	return Point(this->x + x*std::cos(angle) - y*std::sin(angle),
		this->y + x*std::sin(angle) + y*std::cos(angle));
}

Circle::Circle(double x, double y, double r) : c(x, y), r(r) {}
Circle::Circle(const Point &c, double r) : c(c), r(r) {}

Circle Circle::construct(const Point &a, const Point &b){
	return Circle(a, std::hypot(b.x - a.x, b.y - a.y));
}

bool Circle::contains(const Point &p) const {
	return std::fabs(std::hypot(p.x - c.x, p.y - c.y) - r) < EPSILON;
}

Circle::Segment::Segment(const Circle &whole, const Point &a, const Point &b) : whole(whole), a(a), b(b) {}

Circle::Segment Circle::makeSegment(const Point &a, const Point &b) const {
	return Circle::Segment(*this, a, b);
}

bool Circle::operator==(const Circle &other) const {
	return this->c == other.c && std::fabs(other.r - this->r) < EPSILON;
}

Line::Line(double x, double y, double dx, double dy) : p(x, y), dx(dx), dy(dy) {}
Line::Line(const Point &p, double dx, double dy) : p(p), dx(dx), dy(dy) {}

Line Line::construct(const Point &a, const Point &b){
	double dx = b.x - a.x;
	double dy = b.y - a.y;
	double h = std::hypot(dx, dy);
	return Line(a, dx/h, dy/h);
}

bool Line::contains(const Point &p) const {
	return std::fabs((p.x - this->p.x)*-dy + (p.y - this->p.y)*dx) < EPSILON;
}

Line::Segment::Segment(const Line &whole, const Point &a, const Point &b) : whole(whole), a(a), b(b) {}

Line::Segment Line::makeSegment(const Point &a, const Point &b) const {
	return Line::Segment(*this, a, b);
}

bool Line::operator==(const Line &other) const {
	if(std::fabs(this->dx*other.dx + this->dy*other.dy) < 1 - EPSILON){
		return false;
	}
	double dx = other.p.x - this->p.x;
	double dy = other.p.y - this->p.y;
	return std::fabs(dx*-this->dy + dy*other.dx) < EPSILON;
}

ApproxCons::Progress::Progress(size_t points_len, size_t lines_len, size_t circles_len, size_t lsegs_len, size_t csegs_len, size_t regions, size_t steps_len) :
	points_len(points_len), lines_len(lines_len), circles_len(circles_len), lsegs_len(lsegs_len), csegs_len(csegs_len), regions(regions), steps_len(steps_len)
{}

ApproxCons::Progress::Progress(const ApproxCons &base) :
	ApproxCons::Progress(base.points.size(), base.lines.size(), base.circles.size(), base.line_segments.size(), base.circle_segments.size(), base.regions, base.steps.size())
{}

ApproxCons::Step::Step(size_t i, size_t j, ApproxCons::Step::Type type) : i(i), j(j), type(type) {}

ApproxCons::ApproxCons(Limits &&limits) : regions(0), limits(limits) {
	points.reserve(this->limits.max_points());
	lines.reserve(this->limits.max_lines());
	circles.reserve(this->limits.max_circles());
	line_segments.reserve(this->limits.max_lsegs());
	circle_segments.reserve(this->limits.max_csegs());
	p_ls_adj.reserve(this->limits.max_points());
	p_cs_adj.reserve(this->limits.max_points());
	ls_region_adj.reserve(this->limits.max_lsegs());
	cs_region_adj.reserve(this->limits.max_csegs());
	steps.reserve(this->limits.n);
}

ApproxCons::ApproxCons(size_t p0, size_t l0, size_t c0, size_t n, size_t m) : ApproxCons(ApproxCons::Limits(p0, l0, c0, n, m)) {}

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
	this->p_ls_adj.resize(progress.points_len);
	if(progress.points_len){
		this->p_ls_adj[0].resize(progress.lsegs_len);
	}
	this->p_cs_adj.resize(progress.points_len);
	if(progress.points_len){
		this->p_cs_adj[0].resize(progress.csegs_len);
	}
	this->p_region_adj.resize(progress.points_len);
	if(progress.points_len){
		this->p_region_adj[0].resize(progress.regions);
	}
	this->ls_region_adj.resize(progress.lsegs_len);
	if(progress.lsegs_len){
		this->ls_region_adj[0].resize(progress.regions);
	}
	this->cs_region_adj.resize(progress.csegs_len);
	if(progress.csegs_len){
		this->cs_region_adj[0].resize(progress.regions);
	}
	this->steps.resize(progress.steps_len);
}

int ApproxCons::intersect(const Line &a, const Line &b){
	double daxdb = a.dx*b.dy - a.dy*b.dx;
	if(std::fabs(daxdb) < EPSILON){
		return 0;
	}
	double ambx = a.p.x - b.p.x;
	double amby = a.p.y - b.p.y;
	double t = (amby*b.dx - ambx*b.dy)/daxdb;
	this->points.emplace_back(a.p.x + t*a.dx, a.p.y + t*a.dy);
	return 1;
}

int ApproxCons::intersect(const Circle &c, const Line &l){
	double d = (l.p.x - c.c.x)*(-l.dy) + (l.p.y - c.c.y)*l.dx;
	double ox = d*-l.dy;
	double oy = d*l.dx;
	d = std::fabs(d);
	if(d >= c.r + EPSILON){
		return 0;
	}else if(std::fabs(d - c.r) <= EPSILON){
		this->points.emplace_back(c.c.x + ox, c.c.y + oy);
		return 1;
	}//otherwise there are two intersections
	double h = d < EPSILON ? c.r : std::sqrt(c.r*c.r - d*d);
	double dx = h*l.dx;
	double dy = h*l.dy;
	ox += c.c.x;
	oy += c.c.y;
	this->points.emplace_back(ox + dx, oy + dy);
	this->points.emplace_back(ox - dx, oy - dy);
	return 2;
}

inline int ApproxCons::intersect(const Line &a, const Circle &b){
	return intersect(b, a);
}

int ApproxCons::intersect(const Circle &a, const Circle &b){
	double d = std::hypot(b.c.x - a.c.x, b.c.y - a.c.y);
	if(d >= a.r + b.r + EPSILON || d <= std::fabs(b.r - a.r) - EPSILON){
		return 0;
	}
	double dx = b.c.y - a.c.y;
	double dy = b.c.x - a.c.x;
	if(std::fabs(a.r + b.r - d) <= EPSILON || std::fabs(std::fabs(b.r - a.r) - d) <= EPSILON){//there is only one intersection
		if(a.r <= b.r && d < b.r + a.r - EPSILON){
			d = -d;
		}
		d = a.r/d;
		this->points.emplace_back(a.c.x + d*dx, a.c.y + d*dy);
		return 1;
	}//otherwise there are two intersections
	double al = (a.r*a.r - b.r*b.r + d*d)/(2*d);
	double h = std::sqrt(a.r*a.r - al*al);
	al /= d;
	h /= d;
	double x = a.c.x + al*dy;
	double y = a.c.y + al*dx;
	dx *= h;
	dy *= h;
	this->points.emplace_back(x + dx, y - dy);
	this->points.emplace_back(x - dx, y + dy);
	return 2;
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

