#ifndef _TOPOLOGY_CXX__
#define _TOPOLOGY_CXX__

#include <vector>
#include <memory>
#include "comparator.hxx"
#include "primitives.hxx"

class Topology {
	public:
	
	class Point;
	class Segment;
	class Region;
	
	class Simplex {
		public:
		virtual ~Simplex() = 0;
	};
	
	class Point : public Simplex {
		public:
		
		::Point point;
		std::vector<std::pair<Segment*, Segment*>> adj_segments;//counterclockwise order and the pairs show what the other segment on the same geometry is
		std::vector<Region*> adj_regions;//I'm not sure if the order of this matters or if it is even needed
		template<typename... Args>
		Point(Args... args);
		
		constexpr static const Point &getInfinitePoint(const Point &d){
			const Ord x{sgn(d.point.x)};
			const Ord y{sgn(d.point.y)};
			if(x == Ord::EQ && y == Ord::EQ){
				throw std::domain_error("Can't construct infinite point with unspecified direction");
			}
			return getInfinitePoint(x, y);
		}
		
		private:
		
		constexpr static const Point &getInfinitePoint(Ord x, Ord y){
			switch(x){
				case Ord::LT: return getInfinitePoint<Ord::LT>(y);
				case Ord::GT: return getInfinitePoint<Ord::GT>(y);
			}
			return getInfinitePoint<Ord::EQ>(y);
		}
		
		template<Ord x>
		constexpr static const Point &getInfinitePoint(Ord y){
			switch(y){
				case Ord::LT: return infinite_point<x, Ord::LT>;
				case Ord::GT: return infinite_point<x, Ord::GT>;
			}
			return infinite_point<x, Ord::EQ>;
		}
	};
	
	class Segment : public Simplex {
		public:
		const Geom *whole;
		Point *a, *b;
		Region *interior, *exterior;//or left and right for lines
		bool forwards;//is the segment oriented in the same direction as the interior's counterclockwise boundary
		Segment(const Geom &whole, Point &a, Point &b);
		Segment(const Segment &other) = default;
		::Point getArbitraryPoint() const;
		bool contains(const ::Point &p) const;
	};
	
	class Region {
		public:
		std::vector<Segment*> boundaries;//in counterclockwise order
		std::vector<Point*> isolated_pts;//order should not matter, also this vector should never have more than one member and should usually have none so maybe it should be a uniqueptr
		bool compact, convex;
		size_t holes;
		Region();
		::Point getArbitraryPoint();
		
		private:
		::Point arbitrary_circle();
		::Point arbitrary_gibbous();
		::Point arbitrary_convex_compact();
		::Point arbitrary_convex_infinite();
		::Point arbitrary_concave();
		::Point arbitrary_annular();
	};
	
	std::vector<Point> points;
	std::vector<Segment> segments;
	std::vector<Region> regions;
	std::vector<std::pair<Line, Segment*>> lines;
	std::vector<std::pair<Circle, Segment*>> circles;
	
	Topology();
	
	void add_arbitrary_segment_point(size_t seg);
	void add_arbitrary_region_point(size_t reg);
	bool construct_line(size_t pt1, size_t pt2);
	bool construct_circle(size_t pt1, size_t pt2);
	
	private:
	
	enum class NewIntersectionType {Line, Circle, Point};
	
	void add_segment_point(Segment &seg, Topology::Point &p);
	
	template<typename G>
	void construct_line_intersect(
		const Line &a,
		const std::vector<std::pair<G, Topology::Segment*>> &geoms,
		std::vector<std::tuple<Topology::Point*, size_t, Topology::NewIntersectionType>> &xs,
		std::unordered_set<size_t> &xxs,
		Topology::NewIntersectionType type
	);
	
	template<typename G>
	void construct_line_intersect_unchecked(
		const Line &a,
		const std::vector<std::pair<G, Topology::Segment*>> &geoms,
		std::vector<std::tuple<Topology::Point*, size_t, Topology::NewIntersectionType>> &xs,
		Topology::NewIntersectionType type
	);
	
	Segment *find_containing_segment(const ::Point &p, const Line &l, Segment *base_seg);
	Segment *find_containing_segment(const ::Point &p, const Circle &c, Segment *base_seg);
	
	Region *find_directional_infinite_region(const Point &x, const ::Point &d);
	Region *find_directional_region(const Point &x, const ::Point &d);
	
	void construct_line_bifrucate(const Line &a, std::vector<std::tuple<Topology::Point*, size_t, Topology::NewIntersectionType>> &xs);
	void construct_line_bifrucate_unchecked(const Line &a, std::vector<std::tuple<Topology::Point*, size_t, Topology::NewIntersectionType>> &xs);
	
	void construct_line_bifrucate_region(Region &reg, Segment &cut_seg, Point *tail_bound, Point *head_bound);
	
	void construct_line_bifrucate_convex_compact(Region &reg, Segment &cut_seg, Point &tail_pt, Point &head_pt);
	void construct_line_bifrucate_convex_infinite_head(Region &reg, Segment &cut_seg, Point &tail_pt);
	void construct_line_bifrucate_convex_infinite_tail(Region &reg, Segment &cut_seg, Point &head_pt);
	
	constexpr static double infinity_from_sgn(int s){
		return s ? s*INFINITY : 0.;
	}
	
	template<Ord x, Ord y>
	static const Point infinite_point;
};

#endif

