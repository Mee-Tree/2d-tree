#include "primitives.h"

#include <cmath>
#include <ostream>
#include <algorithm>
#include <experimental/iterator>
#include <tuple>
#include <iostream>

/* ----------Point Implementation---------- */

Point::Point() : Point(0., 0.) {}

Point::Point(double x, double y)
        : m_x(x), m_y(y) {}

double Point::x() const {
    return m_x;
}

double Point::y() const {
    return m_y;
}

double Point::distance(const Point & other) const {
    double delta_x = m_x - other.m_x;
    double delta_y = m_y - other.m_y;
    return std::hypot(delta_x, delta_y);
}

bool Point::operator < (const Point & other) const {
    return std::tie(m_x, m_y) < std::tie(other.m_x, other.m_y);
}

bool Point::operator > (const Point & other) const {
    return other < *this;
}

bool Point::operator <= (const Point & other) const {
    return !(*this > other);
}

bool Point::operator >= (const Point & other) const {
    return other <= *this;
}

bool Point::operator == (const Point & other) const {
    return m_x == other.m_x && m_y == other.m_y;
}

bool Point::operator != (const Point & other) const {
    return !(*this == other);
}

std::ostream & operator << (std::ostream & out, const Point & point) {
    out << "(" << point.m_x << ", " << point.m_y << ")";
    return out;
}

/* -----------Rect Implementation---------- */

Rect::Rect()
    : Rect(Point(std::numeric_limits<double>::lowest(), std::numeric_limits<double>::lowest()),
           Point(std::numeric_limits<double>::max(), std::numeric_limits<double>::max())) {}

Rect::Rect(const Point & left_bottom, const Point & right_top)
    : m_left_bottom(left_bottom), m_right_top(right_top) {}

double Rect::xmin() const {
    return m_left_bottom.x();
}

double Rect::ymin() const {
    return m_left_bottom.y();
}

double Rect::xmax() const {
    return m_right_top.x();
}

double Rect::ymax() const {
    return m_right_top.y();
}

double Rect::distance(const Point & p) const {
    Point c(std::max(std::min(p.x(), xmax()), xmin()),
            std::max(std::min(p.y(), ymax()), ymin()));
    return c.distance(p);
}

bool Rect::contains(const Point & p) const {
    return distance(p) == 0.;
}

bool Rect::intersects(const Rect & other) const {
    return xmax() >= other.xmin() && xmin() <= other.xmax()
        && ymax() >= other.ymin() && ymin() <= other.ymax();
}

/* ---------------------------------------- */

namespace rbtree {

PointSet::PointSet() = default;

bool PointSet::empty() const {
    return m_data.empty();
}

std::size_t PointSet::size() const {
    return m_data.size();
}

void PointSet::put(const Point & p) {
    m_data.insert(p);
}

bool PointSet::contains(const Point & p) const {
    return m_data.count(p) > 0;
}

PointSet::ForwardIt PointSet::begin() const {
    return utils::forward_iterator_wrapper(m_data);
}

PointSet::ForwardIt PointSet::end() const {
    return utils::forward_iterator_wrapper(m_data, size());
}

std::pair<PointSet::ForwardIt, PointSet::ForwardIt>
PointSet::range(const Rect & rect) const {
    PointSet set;
    std::copy_if(begin(), end(), utils::putter(set),
        [&rect](const Point & p) {
            return rect.contains(p);
    });
    return std::make_pair(set.begin(), set.end());
}

std::optional<Point> PointSet::nearest(const Point & p) const {
    if (empty()) {
        return std::nullopt;
    }
    return *std::min_element(begin(), end(), utils::distance_cmp(p));
}

std::pair<PointSet::ForwardIt, PointSet::ForwardIt>
PointSet::nearest(const Point & p, std::size_t k) const {
    std::vector<Point> result(std::min(k, size()));
    std::partial_sort_copy(begin(), end(), result.begin(), result.end(), utils::distance_cmp(p));
    PointSet set;
    std::copy(result.begin(), result.end(), utils::putter(set));
    return std::make_pair(set.begin(), set.end());
}

std::ostream & operator << (std::ostream & strm, const PointSet & ps) {
    strm << "{";
    std::copy(ps.begin(), ps.end(),
        std::experimental::make_ostream_joiner(strm, "; "));
    return strm << "}";
}

} // end of namespace rbtree
