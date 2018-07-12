#ifndef _GEOMETRY_HXX__
#define _GEOMETRY_HXX__

#include <vector>
#include "primitives.hxx"
#include "topology.hxx"

class ApproxCons {
	public:
	
	class Limits {
		public:
		size_t p0, l0, c0, n, m;
		
		constexpr Limits(size_t p0, size_t l0, size_t c0, size_t n, size_t m) : p0(p0), l0(l0), c0(c0), n(n), m(m) {}
		
		constexpr size_t max_points() const {
			return p0 + 2*n*(l0 + c0) + n*n - 2*m;
		}

		constexpr size_t max_lines() const {
			return l0 + n;
		}
		
		constexpr size_t max_circles() const {
			return c0 + n;
		}
		
		constexpr size_t max_lsegs() const {
			size_t l = max_lines();
			return l*l + 2*l*c0;
		}
		
		constexpr size_t max_csegs() const {
			size_t c = max_circles();
			return 2*c*(c - 1) + 2*l0*c;
		}
		
		constexpr size_t max_regions() const {
			return 1 + max_lsegs() + max_csegs() - max_points();
		}
	};

	class Step {
		public:
		size_t i, j;
		enum Type{
			PPL, PPC
		} type;
		Step() = default;
		Step(size_t i, size_t j, Type type);
	};
	
	class Progress {
		public:
		size_t
			points_len,
			lines_len,
			circles_len,
			lsegs_len,
			csegs_len,
			regions,
			steps_len;
		Progress(size_t points_len, size_t lines_len, size_t circles_len, size_t lsegs_len, size_t csegs_len, size_t regions, size_t steps_len);
		Progress(const ApproxCons &base);
	};
	
	std::vector<Point> points;
	std::vector<Line> lines;
	std::vector<Circle> circles;
	//std::vector<Line::Segment> line_segments;
	//std::vector<Circle::Segment> circle_segments;
	size_t regions;
	std::vector<std::vector<bool>> p_ls_adj;
	std::vector<std::vector<bool>> p_cs_adj;
	std::vector<std::vector<bool>> p_region_adj;
	std::vector<std::vector<bool>> ls_region_adj;
	std::vector<std::vector<bool>> cs_region_adj;
	std::vector<Step> steps;
	Circle goal;
	Limits limits;
	
	ApproxCons(Limits &&limits);
	ApproxCons(size_t p0, size_t l0, size_t c0, size_t n, size_t m);
	
	void init_topology();
	
	Progress getProgress();
	void resetProgress(const Progress &progress);
	
	int intersect(const Line &a, const Line &b);
	int intersect(const Circle &c, const Line &l);
	inline int intersect(const Line &a, const Circle &b);
	int intersect(const Circle &a, const Circle &b);
	
	bool add_line(Line &&a);
	void add_line_unchecked(Line &&a);
	bool add_circle(Circle &&a);
	void add_circle_unchecked(Circle &&a);
	
	void add_arbitrary_lseg_point(size_t seg);
	void add_arbitrary_cseg_point(size_t seg);
	void add_arbitrary_region_point(size_t reg);
	
	void remove_duplicate_points(size_t old_len, size_t start_len);
	
	void record_step(Step &&step);
};

std::vector<Point> intersect(const Line &a, const Line &b);
std::vector<Point> intersect(const Circle &a, const Line &b);

inline std::vector<Point> intersect(const Line &a, const Circle &b){
	return intersect(b, a);
}

std::vector<Point> intersect(const Circle &a, const Circle &b);

inline std::vector<Point> intersect(const Geom &a, const Geom &b){
	if(typeid(a) == typeid(const Line&)){
		if(typeid(b) == typeid(const Line&)){
			return intersect(static_cast<const Line&>(a), static_cast<const Line&>(b));
		}
		return intersect(static_cast<const Line&>(a), static_cast<const Circle&>(b));
	}else if(typeid(b) == typeid(const Line&)){
		return intersect(static_cast<const Circle&>(a), static_cast<const Line&>(b));
	}
	return intersect(static_cast<const Circle&>(a), static_cast<const Circle&>(b));
}

inline std::vector<Point> intersect(const Line &a, const Geom &b){
	if(typeid(b) == typeid(const Line&)){
		return intersect(a, static_cast<const Line&>(b));
	}
	return intersect(a, static_cast<const Circle&>(b));
}

inline std::vector<Point> intersect(const Circle &a, const Geom &b){
	if(typeid(b) == typeid(const Line&)){
		return intersect(a, static_cast<const Line&>(b));
	}
	return intersect(a, static_cast<const Circle&>(b));
}

inline std::vector<Point> intersect(const Geom &a, const Line &b){
	if(typeid(a) == typeid(const Line&)){
		return intersect(static_cast<const Line&>(a), b);
	}
	return intersect(static_cast<const Circle&>(a), b);
}

inline std::vector<Point> intersect(const Geom &a, const Circle &b){
	if(typeid(a) == typeid(const Line&)){
		return intersect(static_cast<const Line&>(a), b);
	}
	return intersect(static_cast<const Circle&>(a), b);
}

#endif
