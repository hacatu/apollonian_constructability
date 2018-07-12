#include "comparator.hxx"

template<typename D, typename K>
std::function<bool(const D&, const D&)> comparing(std::function<K(const D&)> key){
	return [](const D &a, const D &b){
		return key(a) < key(b);
	}
}

