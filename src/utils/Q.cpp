#include "Q.h"

namespace nl {
namespace utils {

Q::Q(int n /*= 6*/, double val /*= 0.0*/)
{
	values_ = std::vector(n, val);
}

Q::Q(std::initializer_list<double> vals)
{
	values_ = std::vector(vals);
}

}
}