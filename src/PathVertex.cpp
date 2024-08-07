#include "PathVertex.hpp"

std::ostream& operator<<(std::ostream& os, const PathVertex& v) {
    os << "position: " << v.inter.coords << " alpha: " << v.alpha;
    return os;
}