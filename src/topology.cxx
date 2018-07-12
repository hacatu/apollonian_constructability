#include <cmath>
#include <vector>
#include <unordered_set>
#include <algorithm>
#include <memory>
#include "topology.hxx"
#include "geometry.hxx"
#include "comparator.hxx"

template<typename... Args>
Topology::Point::Point(Args... args):
	point(std::forward<Args>(args)...),
	adj_segments(),
	adj_regions() {}

Topology::Segment::Segment(const Geom &whole, Point &a, Point &b):
	whole(std::addressof(whole)),
	a(std::addressof(a)),
	b(std::addressof(b)),
	interior(),
	exterior(),
	forwards() {}

Point Topology::Segment::getArbitraryPoint() const {
	return this->whole->getArbitraryPoint(this->a->point, this->b->point);
}

bool Topology::Segment::contains(const ::Point &p) const {
	return this->whole->contains(this->a->point, this->b->point, p);
}

Topology::Region::Region() : boundaries() {}

Point Topology::Region::getArbitraryPoint() {
	if(this->convex){
		if(this->compact){
			switch(this->boundaries.size()){
				case 1: return this->arbitrary_circle();
				case 2: return this->arbitrary_gibbous();
				default: return this->arbitrary_convex_compact();
			}
		}
		return this->arbitrary_convex_infinite();
	}
	if(!this->holes){
		return this->arbitrary_concave();
	}
	return this->arbitrary_annular();
}

Point Topology::Region::arbitrary_circle(){
	const ::Point &c = static_cast<const Circle*>(this->boundaries[0]->whole)->c;
	return c.unit_combination(std::cbrt(.25), this->boundaries[0]->a->point);
}

Point Topology::Region::arbitrary_gibbous(){
	const Segment *s1 = this->boundaries[0];
	const Segment *s2 = this->boundaries[1];
	const Circle *c1 = dynamic_cast<const Circle*>(s1->whole);
	const Circle *c2;
	const Line *l;
	if(c1){
		if((c2 = dynamic_cast<const Circle*>(s2->whole))){
			return s1->a->point.unit_combination(std::cbrt(.25), s2->a->point);
		}
		l = static_cast<const Line*>(s2->whole);
	}else{
		c1 = static_cast<const Circle*>(s2->whole);
		l = static_cast<const Line*>(s1->whole);
	}
	const ::Point m = s1->a->point.unit_combination(.5, s2->a->point);
	double t = std::cbrt(.25)*c1->r + (1 - std::cbrt(.25))*std::sqrt(c1->r*c1->r - s1->a->point.distance(s2->a->point)/2);
	return (1 - t)*c1->c + t*m;
}

Point Topology::Region::arbitrary_convex_compact(){
	double t = (1 - std::cbrt(.25))/2;
	const ::Point *a = std::addressof(this->boundaries[0]->a->point);
	const ::Point *b = std::addressof(this->boundaries[1]->a->point);
	size_t i = 2;
	const ::Point *c;
	for(; i < this->boundaries.size(); ++i){
		c = std::addressof(this->boundaries[i]->a->point);
		if((*b - *a).cross(*c - *a) >= EPSILON){
			break;
		}
	}
	if(i >= this->boundaries.size()){
		for(i = 0;; ++i){
			const Segment *s = this->boundaries[i];
			if(typeid(s->whole) == typeid(const Circle*)){
				return a->unit_combination(t, *b, t, s->getArbitraryPoint());
			}
		}
	}
	return a->unit_combination(t, *b, t, *c);
}

Point Topology::Region::arbitrary_convex_infinite(){
	if(this->boundaries.empty()){
		return ::Point(std::cbrt(.25), 0);//TODO: we need to make sure these cube roots are actually not in the field of numbers constructable from the starting points
	}
	const Segment *ls1 = this->boundaries[0];
	const Segment *ls2 = this->boundaries.back();
	const Line *l1 = static_cast<const Line*>(ls1->whole);
	const Line *l2 = static_cast<const Line*>(ls2->whole);
	if(ls1->whole == ls2->whole){
		const ::Point &p = this->boundaries.size() == 1 ? static_cast<const Line*>(ls1->whole)->p : ls1->b->point;
		if(this == this->boundaries[0]->interior){
			return p + std::cbrt(.25)*l1->d.ccw();
		}
		return p + std::cbrt(.25)*l1->d.cw();
	}
	
	const ::Point *a, *b;
	if(l1->isInfiniteHead(ls1->a->point, ls1->b->point)){
		a = std::addressof(l1->p);
		b = std::addressof(l2->p);
	}else{
		a = std::addressof(ls1->b->point);
		b = std::addressof(ls2->a->point);
	}
	return a->unit_combination(std::cbrt(.25), *b);
}

Point Topology::Region::arbitrary_concave(){
	//project a normal from an arbitrary point on the first segment and find the closest intersection
	//alternatively, find the first pair of segments that are not colinear and find a point close enough
	//to them that no convex circle could cover it unless it were isolated (the annular case)
	size_t i = 0;
	Line l;
	::Point p;
	for(;; ++i){
		const Segment *s = this->boundaries[i];
		if(const Circle *c = dynamic_cast<const Circle*>(s->whole)){
			if(s->exterior == this){
				p = s->getArbitraryPoint();
				l = Line(p, (p - c->c).unit());
				break;
			}
		}
	}
	double min = INFINITY;
	::Point min_x;
	for(size_t j = 0; j < this->boundaries.size(); ++j){
		if(j == i){
			continue;
		}
		for(const ::Point &x : intersect(l, *this->boundaries[j]->whole)){
			double dot = (x - p)*l.d;
			if(dot < min){
				min = dot;
				min_x = x;
			}
		}
	}
	if(std::isfinite(min)){
		return p.unit_combination(std::cbrt(.25), min_x);
	}
	return p + std::cbrt(.25)*l.d;
}

Point Topology::Region::arbitrary_annular(){
	const Segment *s = this->boundaries.back();
	::Point p = s->getArbitraryPoint();
	Line l(p, (p - static_cast<const Circle*>(s->whole)->c).unit());
	double min = INFINITY;
	::Point min_x;
	for(size_t i = 0; i < this->boundaries.size() - 1; ++i){
		for(const ::Point &x : intersect(l, *this->boundaries[i]->whole)){
			double dot = (x - p)*l.d;
			if(dot < min){
				min = dot;
				min_x = x;
			}
		}
	}
	if(std::isfinite(min)){
		return p.unit_combination(std::cbrt(.25), min_x);
	}
	return p + std::cbrt(.25)*l.d;
}

Topology::Topology() = default;

void Topology::add_segment_point(Segment &seg, Point &p){
	Segment *h = std::addressof(seg);
	Segment *t = std::addressof(this->segments.emplace_back(*h));
	for(Region *r : {h->interior, h->exterior}){
		auto it = std::find(r->boundaries.begin(), r->boundaries.end(), h);
		r->boundaries.insert(it + h->forwards, t);
	}
	p.adj_segments = {std::make_pair(h, t), std::make_pair(t, h)};
	p.adj_regions = {h->interior, h->exterior};
	h->b = t->a = std::addressof(p);
}

void Topology::add_arbitrary_segment_point(size_t seg){
	Segment *h = std::addressof(this->segments[seg]);
	Point &p = this->points.emplace_back(h->getArbitraryPoint());
	add_segment_point(*h, p);
}

void Topology::add_arbitrary_region_point(size_t reg){
	Region *r = std::addressof(this->regions[reg]);
	Point &p = this->points.emplace_back(r->getArbitraryPoint());
	p.adj_regions = {r};
	r->isolated_pts.push_back(std::addressof(this->points.back()));
}

template<typename G>
void Topology::construct_line_intersect(
	const Line &a,
	const std::vector<std::pair<G, Segment*>> &geoms,
	std::vector<std::tuple<Point*, size_t, NewIntersectionType>> &xs,
	std::unordered_set<size_t> &xxs,
	NewIntersectionType type
){
	for(size_t i = 0; i < geoms.size(); ++i){
		const G &b = geoms[i].first;
		for(const ::Point &x : intersect(a, b)){
			for(size_t j = 0; j < this->points.size(); ++j){
				const Point &p = this->points[j];
				if(x == p.point){
					xxs.insert(j);
					goto CONTINUE_LINE_INTERSECTION_LOOP;
				}
			}
			xs.emplace_back(std::addressof(this->points.emplace_back(x)), i, type);
			CONTINUE_LINE_INTERSECTION_LOOP:;
		}
	}
}

template<typename G>
void Topology::construct_line_intersect_unchecked(
	const Line &a,
	const std::vector<std::pair<G, Segment*>> &geoms,
	std::vector<std::tuple<Point*, size_t, NewIntersectionType>> &xs,
	NewIntersectionType type
){
	for(size_t i = 0; i < geoms.size(); ++i){
		const G &b = geoms[i].first;
		for(const ::Point &x : intersect(a, b)){
			if(a.p != x){
				xs.emplace_back(std::addressof(this->points.emplace_back(x)), i, type);
			}
		}
	}
}

bool Topology::construct_line(size_t pt1, size_t pt2){
	Point *p1 = std::addressof(this->points[pt1]), *p2 = std::addressof(this->points[pt2]);
	if(p1->adj_segments.size() > p2->adj_segments.size()){
		std::swap(p1, p2);
		std::swap(pt1, pt2);
	}
	for(auto [s, _] : p1->adj_segments){
		if(s->whole->contains(p2->point)){
			return false;
		}
	}
	std::vector<std::tuple<Point*, size_t, NewIntersectionType>> xs;
	xs.reserve(this->lines.size() + 2*this->circles.size());
	const Line a = Line::construct(p2->point, p1->point);//note the order of p1 and p2.  This ensures the base point of the line is an existing intersection, not an isolated arbitrary point
	bool uses_arb = p1->adj_segments.empty();
	//we only need p1 during bifrucation if it is already an intersection point, which it can't be if we are using an arbitrary point.
	//so if we aren't using an arbitrary point, p1 and p2 are intersections so they will show up in xxs and go into xs anyway
	//if we are using arbitrary points, p1 should be added later after bisection is done.  we can always change this to happen in the main bisection code anyway
	//xs.emplace_back(p1, pt1, Topology::NewIntersectionType::Point);
	if(!uses_arb){
		std::unordered_set<size_t> xxs;
		xxs.reserve((this->lines.size() + 2*this->circles.size())/2);
		this->construct_line_intersect(a, this->lines, xs, xxs, NewIntersectionType::Line);
		this->construct_line_intersect(a, this->circles, xs, xxs, NewIntersectionType::Circle);
		for(size_t i : xxs){
			xs.emplace_back(std::addressof(this->points[i]), i, NewIntersectionType::Point);
		}
	}else{
		this->construct_line_intersect_unchecked(a, this->lines, xs, NewIntersectionType::Line);
		this->construct_line_intersect_unchecked(a, this->circles, xs, NewIntersectionType::Circle);
		xs.emplace_back(p2, pt2, NewIntersectionType::Point);
	}
	std::sort(xs.begin(), xs.end(),
		comparing<std::tuple<Point*, size_t, NewIntersectionType>, double>(
			[&](const std::tuple<Point*, size_t, NewIntersectionType> &p){
				return a.d*std::get<0>(p)->point;
	}));
	if(!uses_arb){
		this->construct_line_bifrucate(a, xs);
	}else{
		this->construct_line_bifrucate_unchecked(a, xs);
		//TODO: handle putting p1 on the correct segment
	}
	return true;
}

Topology::Segment *Topology::find_containing_segment(const ::Point &p, const Line &l, Segment *base_seg){
	for(Segment *s = base_seg;;){
		if(s->contains(p)){
			return s;
		}else if(static_cast<const Line*>(s->whole)->isInfiniteTail(s->a->point, s->b->point)){
			break;
		}
		const Point *b = s->b;
		for(size_t i = 0;; i += 1){
			if(b->adj_segments[i].first == s){
				s = b->adj_segments[i].second;
				break;
			}
		}
	}
	for(Segment *s = base_seg; !static_cast<const Line*>(s->whole)->isInfiniteHead(s->a->point, s->b->point);){
		const Point *a = s->a;
		for(size_t i = 0;; ++i){
			if(a->adj_segments[i].first == s){
				s = a->adj_segments[i].second;
				break;
			}
		}
	}
	return nullptr;
}

Topology::Segment *Topology::find_containing_segment(const ::Point &p, const Circle &c, Segment *base_seg){
	while(true){
		Segment *s = base_seg;
		if(s->contains(p)){
			return s;
		}
		const Point *b = s->b;
		for(size_t i = 0;; ++i){
			if(b->adj_segments[i].first == s){
				s = b->adj_segments[i].second;
				break;
			}
		}
	}
}

Topology::Region *Topology::find_directional_infinite_region(const Point &x, const ::Point &d){
	for(Region *reg : x.adj_regions){
		if(!reg->compact){
			::Point a = static_cast<const Line*>(reg->boundaries[0]->whole)->d;
			::Point b = static_cast<const Line*>(reg->boundaries.back()->whole)->d;
			if(reg->boundaries[0]->forwards){
				a = -a;
			}
			if(!reg->boundaries.back()->forwards){
				b = -b;
			}
			if(b.cross(d) > 0 && d.cross(a) > 0){
				return reg;
			}
		}
	}
	return nullptr;
}

Topology::Region *Topology::find_directional_region(const Point &x, const ::Point &d){
	bool was_ccw;
	for(size_t i = 0; i < x.adj_segments.size(); ++i){
		Segment *seg = x.adj_segments[i].first;
		::Point a;
		if(const Line *l = dynamic_cast<const Line*>(seg->whole)){
			//we need to know the direction along seg away from x.
			//this is +/-d of the underlying line.  Can we determine +/- from seg->forwards?
			//forwards depends on the positions of the endpoints so no
			a = l->d;
			if(seg->a == std::addressof(x)){
				if(a*(seg->b->point - x.point) < 0){
					a = -a;
				}
			}else if(a*(seg->a->point - x.point) < 0){
				a = -a;
			}
		}else{//seg is on a Circle
			const Circle *c = static_cast<const Circle*>(seg->whole);
			a = (x.point - c->c).unit();
			if((seg->a == std::addressof(x)) == seg->forwards){
				a = a.ccw();
			}else{
				a = a.cw();
			}
		}
		double t = a.cross(d);
		if(t < 0 || d == -a){//d is counterclockwise with respect to seg
			if(was_ccw){
				Segment *prev_seg = x.adj_segments[i - 1].first;
				if(seg->exterior == prev_seg->interior || seg->exterior == prev_seg->exterior){
					return seg->exterior;
				}
				return seg->interior;
			}
		}else{
			was_ccw = true;
		}
	}
	Segment *seg = x.adj_segments[0].first, *prev_seg = x.adj_segments.back().first;
	if(seg->exterior == prev_seg->interior || seg->exterior == prev_seg->exterior){
		return seg->exterior;
	}
	return seg->interior;
}

//should be similar to construct_line_bifrucate_unchecked except we don't need to handle arbitrary points, but we are factoring that out anyway
void Topology::construct_line_bifrucate(const Line &a, std::vector<std::tuple<Point*, size_t, NewIntersectionType>> &xs){
	//TODO
}

void Topology::construct_line_bifrucate_unchecked(const Line &a, std::vector<std::tuple<Point*, size_t, NewIntersectionType>> &xs){
	/* 1: Find the first 2D region and its 1 or 0 D boundary:
	 * 2: Be on the lookout for the isolated arbitrary point
	 * 3: Find the next 2D region and its other 1 or 0 D boundary (and repeat)
	 * 4: Handle the last 2D region
	 * 
	 * 1: Find the first 2D region and its 1 or 0 D boundary:
	 * 1.1a.1: if the first intersection along Line a is on a circle or line, find the segment it is on by linear search
	 * 1.1a.2a: if the first intersection is on a circle, the region at the tail of Line a is exterior to the segment we just found
	 * 1.1a.2b: otherwise, the first intersection is on a line b.
	 *            If a.d.cross(b.d) is positive, we use the left region (interior), if it is negative, we use the right region (exterior)
	 * 1.1b.1: otherwise, the first intersection is at an existing intersection (we constructed a.p to be this existing intersection). 
	 *            We need to find the region around this point that contains the tail of a.
	 *            We can look at all infinite regions around the point.
	 *            Each one has as its first boundary an infinite line segment and as its last boundary an infinite line segment
	 *            since we don't store the edge at infinity.  We can just look at the directions of these lines for all such
	 *            regions and find the one that includes the tail's direction
	 * 1.2: Create the new segment and actually split the region.
	 *            In this case the region is infinite and both of the resulting regions must be infinite, although this is not always true.
	 *            (The counterexample is partially split regions where a new line's tail is tangent or secant to a circle)
	 *            If a region is convex and we add a line, both of the resulting regions are convex.
	 *            If a region is concave, we need to check which parts remain concave.
	 *            If one part turns out to be convex, the other must be concave, but both can also be concave.
	 * 1.3: Going forwards we will now always have a previous 2D region, a previous 1D segment on the new line, and a previous 1 or 0 D boundary of the region
	 * 2: Be on the lookout for the isolated arbitrary point:
	 * 2.1a: If the previous boundary is 1 D we know exactly what region we are in and can start splicing the new segment in immediately.
	 *            It might be smarter to just save the segment and call add_point on it (currently we have add arbitrary point but we could split that into two functions for utility)
	 *            Yes this is much smarter, in fact I think the arbitrary point should not even be added to the xs list and instead we should walk over the segments of the line afterwords
	 * 2.1b: Otherwise, the previous boundary is 0 D (a point) and the regions around a point are stored in counterclockwise order.
	 *            However, because of tangent circles this does not imply the head of the line is in the region opposite the previous region.
	 *            Also because of circles we cannot do a simple comparison of line directions.
	 *            Finding the proper next region is actually hard in this case so I will leave it TODO for now
	 * 3: Find the next 2D region and its other 1 or 0 D boundary (and repeat):
	 * 3.1: We have already seen more or less how to find the next 2D region, and the next 1 or 0 D boundary isn't too hard since we already know every intersection.
	 *            If it is a 0 D boundary we have it automatically.
	 *            Otherwise it is a 1 D boundary and we can find it by walking the boundary of the region and looking at the boundary segments which overlay the geometry
	 *            associated with the forthcoming intersection.  There could be several such segments becuase of tangent circles so if there are multiple we will have to check.
	 *            They might not be consecutive because of secant circles so we might as well check all of them.
	 * 4: Handle the last 2D region:
	 * 4.1: This should go down pretty much exactly like the first region except we even know what 0 or 1 D boundary we are intersecting this time so we just have to split
	 */
	auto [p, i, type] = xs[0];
	Region *reg, *next_reg;
	Segment *prev_cut = nullptr, *cut_seg = std::addressof(this->segments.emplace_back(a, const_cast<Point&>(Point::getInfinitePoint(-a.d)), *p));
	Point *tail_pt = nullptr, *head_pt;
	if(type == NewIntersectionType::Line){
		auto &[l, base_seg] = this->lines[i];
		Segment *head_bound = this->find_containing_segment(p->point, l, base_seg);
		if(a.d.cross(l.d)){
			reg = head_bound->interior;
			next_reg = head_bound->exterior;
		}else{
			reg = head_bound->exterior;
			next_reg = head_bound->interior;
		}
		this->add_segment_point(*head_bound, *p);
	}else if(type == NewIntersectionType::Circle){
		auto &[c, base_seg] = this->circles[i];
		Segment *head_bound = this->find_containing_segment(p->point, c, base_seg);
		reg = head_bound->exterior;
		next_reg = head_bound->interior;
		this->add_segment_point(*head_bound, *p);
	}else{//type == Point
		reg = this->find_directional_infinite_region(*p, -a.d);
		next_reg = this->find_directional_region(*p, a.d);
	}
	head_pt = p;
	//now we have to actually insert the new segment *cut_seg.  There will be 8 cases in general for the edges:
	//each edge can be a point, a segment, or infinite, but at most one can be infinite.
	//Also, the two regions that get created have to have their convexity and finiteness determined
	//by pre-splicing segments, we can make sure the boundaries are not segments, so there are 3 cases for the edges:
	//infinite-point, point-infinite, and point-point.
	this->construct_line_bifrucate_region(*reg, *cut_seg, tail_pt, head_pt);
	for(size_t xi = 1; xi < xs.size(); ++xi){
		auto [p, i, type] = xs[xi];
		reg = next_reg;
		prev_cut = cut_seg;
		cut_seg = std::addressof(this->segments.emplace_back(a, *cut_seg->b, *p));
		tail_pt = head_pt;
		//now we just have to compute the new head_bound and next_reg and call construct_line_bifrucate_single again
		if(type != NewIntersectionType::Point){
			Segment *head_bound;
			if(type == NewIntersectionType::Line){
				auto &[l, base_seg] = this->lines[i];
				head_bound = this->find_containing_segment(p->point, l, base_seg);
			}else{
				auto &[c, base_seg] = this->circles[i];
				head_bound = this->find_containing_segment(p->point, c, base_seg);
			}
			if(type == NewIntersectionType::Line ||
				a.d*(p->point - static_cast<const Circle*>(head_bound->whole)->c) >= EPSILON
			){
				if(static_cast<Segment*>(head_bound)->interior == reg){
					next_reg = head_bound->exterior;
				}else{
					next_reg = head_bound->interior;
				}
			}else{//tangent circle
				next_reg = reg;
			}
		}else{//type == Point
			next_reg = this->find_directional_region(*p, a.d);
		}
		head_pt = p;
		this->construct_line_bifrucate_region(*reg, *cut_seg, tail_pt, head_pt);
	}
	cut_seg = std::addressof(this->segments.emplace_back(a, *cut_seg->b, const_cast<Point&>(Point::getInfinitePoint(a.d))));
	this->construct_line_bifrucate_region(*next_reg, *cut_seg, head_pt, nullptr);
}

void Topology::construct_line_bifrucate_region(Region &reg, Segment &cut_seg, Point *tail_pt, Point *head_pt){
	//we add the point when it is at the head, but when should we add the new segments to it?
	//right now we have both segments for the point at tail_bound but only the tail segment for the point at head_bound
	//and we will have the regions before tail_bound conceptually in this function and be making
	//the ones between tail_bound and head_bound in reg.  So it makes sense that we should only
	//touch the point and segment at tail_bound and not the one at head_bound.
	if(reg.convex){//can't have any tangent circles on the boundary
		if(reg.compact){
			this->construct_line_bifrucate_convex_compact(reg, cut_seg, *tail_pt, *head_pt);
		}else if(tail_pt){
			this->construct_line_bifrucate_convex_infinite_head(reg, cut_seg, *tail_pt);
		}else{
			this->construct_line_bifrucate_convex_infinite_tail(reg, cut_seg, *head_pt);
		}
	}else{
		//if the region is concave, we might still be able to get away with doing something simple.
		//if there are only two intersection points on the boundary, we do pretty much the same thing as for a convex region
		//if the second intersection is on a concave segment though there might be more intersections in this region
		//ideally we should handle these by adding segments to point adjacencies that have nullptr partners and then filling them in later
		//I'm not sure how that would work for regions.  Since we pre-bifrucate segments, all the boundaries of the regions exist except segments on the head of the cutting line
		//If reg has holes and cut_seg connects them to the boundary, we need to extend the boundary of reg in a weird way that will include cut_seg twice.
		//This duplicated cut_seg will be separated when a segment on the head of the cutting line actually crosses the other boundary of the annular region reg.
		//Finally, there are some edge cases involving isolated points and constructing the initial geometries where some assumptions are not valid that we need to make sure we get these right
	}
}

void Topology::construct_line_bifrucate_convex_compact(Region &reg, Segment &cut_seg, Point &tail_pt, Point &head_pt){
	//TODO
}

void Topology::construct_line_bifrucate_convex_infinite_head(Region &reg, Segment &cut_seg, Point &tail_pt){
	//TODO
}

void Topology::construct_line_bifrucate_convex_infinite_tail(Region &reg, Segment &cut_seg, Point &head_pt){
	//TODO
}
	
bool Topology::construct_circle(size_t pt1, size_t pt2){
	return false;//Oh boy this might be worse than the line one
}

template<Ord x, Ord y>
const Topology::Point Topology::infinite_point(Topology::infinity_from_sgn(static_cast<int>(x)), Topology::infinity_from_sgn(static_cast<int>(y)));

