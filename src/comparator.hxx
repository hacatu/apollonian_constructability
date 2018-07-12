#ifndef _COMPARATOR_HXX__
#define _COMPARATOR_HXX__
#include <functional>

constexpr int sgn(double v){
	return (0 < v) - (v < 0);
}

enum class Ord {LT = -1, EQ = 0, GT = 1};

template<typename D, typename K>
std::function<bool(const D&, const D&)> comparing(std::function<K(const D&)> key);

#endif

