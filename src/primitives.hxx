#ifndef _PRIMITIVES_HXX__
#define _PRIMITIVES_HXX__
#include <memory>
#include <array>
#include <any>

#define EPSILON 5e-6

class Point {
	public:
	double x, y;
	constexpr Point() : Point(0, 0) {}
	constexpr Point(const Point&) = default;
	constexpr Point(double x, double y) : x(x), y(y) {}
	
	constexpr Point &operator=(Point&&) = default;
	constexpr Point &operator=(const Point&) = default;
	
	constexpr bool operator==(const Point &other) const {
		double dx = other.x - this->x;
		double dy = other.y - this->y;
		return dx*dx + dy*dy < EPSILON;
	}
	
	constexpr bool operator!=(const Point &other) const {
		return !this->operator==(other);
	}
	
	constexpr Point operator-() const {
		return Point(-this->x, -this->y);
	}
	
	constexpr Point operator+(const Point &other) const {
		return Point(this->x + other.x, this->y + other.y);
	}
	
	constexpr Point operator-(const Point &other) const {
		return Point(this->x - other.x, this->y - other.y);
	}
	
	constexpr double operator*(const Point &other) const {
		return this->x*other.x + this->y*other.y;
	}
	
	constexpr double cross(const Point &other) const {
		return this->x*other.y - this->y*other.x;
	}
	
	inline Point unit() const {
		return *this / magnitude();
	}
	
	inline double distance(const Point &other) const {
		return std::hypot(this->x - other.x, this->y - other.y);
	}
	
	inline double magnitude() const {
		return std::hypot(this->x, this->y);
	}
	
	constexpr Point transpose() const {
		return Point(this->y, this->x);
	}
	
	inline Point rotate(double angle) const {
		return Point(this->x*std::cos(angle) - y*std::sin(angle),
			this->x*std::sin(angle) + y*std::cos(angle));
	}
	
	constexpr Point ccw() const {
		return Point(-this->y, this->x);
	}
	
	constexpr Point cw() const {
		return Point(this->y, -this->x);
	}
	
	inline Point rotate(double angle, const Point &other) const {
		double x = other.x - this->x;
		double y = other.y - this->y;
		return Point(this->x + x*std::cos(angle) - y*std::sin(angle),
			this->y + x*std::sin(angle) + y*std::cos(angle));
	}
	
	template<typename ...T>
	static constexpr void unit_combination_typecheck(double a, const Point &p, T ...terms){
		if constexpr(sizeof...(T) != 0){
			unit_combination_typecheck(terms...);
		}
	}
	
	template<typename... T>
	constexpr Point unit_combination(T... terms) const {
		if constexpr(sizeof...(T) != 0){
			unit_combination_typecheck(terms...);
		}
		const std::any args[] = {terms...};
		double t = 1, x = 0, y = 0;
		for(size_t i = 0; i < sizeof...(T); i += 2){
			double a(std::any_cast<double>(args[i]));
			t -= a;
			const Point &p = std::any_cast<const Point&>(args[i + 1]);
			x += a*p.x;
			y += a*p.y;
		}
		return Point(t*this->x + x, t*this->y + y);
	}
	
	constexpr Point operator/(double n) const {
		return Point(this->x/n, this->y/n);
	}
};

constexpr Point operator*(double t, const Point &self){
	return Point(t*self.x, t*self.y);
}

class Geom {
	public:
	
	virtual bool contains(const Point &p) const = 0;
	virtual bool contains(const Point &a, const Point &b, const Point &p) const = 0;
	virtual Point getArbitraryPoint(const Point &a, const Point &b) const = 0;
	virtual ~Geom() = default;
};

class Circle : public Geom {
	public:
	
	Point c;
	double r;
	Circle() = default;
	Circle(const Circle&) = default;
	Circle(double x, double y, double r);
	Circle(const Point &c, double r);
	
	static Circle construct(const Point &a, const Point &b);
	
	bool contains(const Point &p) const;
	bool contains(const Point &a, const Point &b, const Point &p) const;
	
	Point getArbitraryPoint(const Point &a, const Point &b) const;
	
	Circle &operator=(Circle&&) = default;
	
	bool operator==(const Circle &other) const;
};

class Line : public Geom {
	public:
	
	Point p, d;
	Line() = default;
	Line(const Line&) = default;
	Line(double x, double y, double dx, double dy);
	Line(const Point &p, double dx, double dy);
	Line(double x, double y, const Point &d);
	Line(const Point &p, const Point &d);
	
	static Line construct(const Point &a, const Point &b);
	
	bool contains(const Point &p) const;
	bool contains(const Point &a, const Point &b, const Point &p) const;
	
	Point getArbitraryPoint(const Point &a, const Point &b) const;
	bool isInfinite(const Point &a, const Point &b) const;
	bool isInfiniteTail(const Point &a, const Point &b) const;
	bool isInfiniteHead(const Point &a, const Point &b) const;
	bool isInfiniteBoth(const Point &a, const Point &b) const;

	bool operator==(const Line &other) const;
};

#endif

