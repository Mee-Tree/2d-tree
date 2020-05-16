#pragma once

#include <ostream>
#include <optional>
#include <vector>
#include <set>

class Point {
    double m_x;
    double m_y;

public:
	Point(double x, double y);

	double x() const;
	double y() const;
	double distance(const Point &) const;

	bool operator< (const Point &) const;
	bool operator> (const Point &) const;
	bool operator<= (const Point &) const;
	bool operator>= (const Point &) const;
	bool operator== (const Point &) const;
	bool operator!= (const Point &) const;

	friend std::ostream & operator << (std::ostream &, const Point &);

};

class Rect {
    Point m_left_bottom;
    Point m_right_top;

public:
	Rect(const Point & left_bottom, const Point & right_top);

	double xmin() const;
	double ymin() const;
	double xmax() const;
	double ymax() const;
	double distance(const Point & p) const;

	bool contains(const Point & p) const;
	bool intersects(const Rect &) const;
};

namespace utils {

static inline auto distance_cmp(const Point & p) {
    return [&p](const Point & a, const Point & b) {
        return p.distance(a) < p.distance(b);
    };
}

} // end of namespace utils

namespace rbtree {

class PointSet {
    using Data = std::set<Point>;
    Data m_data;

public:
    using ForwardIt = Data::const_iterator;

    PointSet();

    bool empty() const;
    std::size_t size() const;
    void put(const Point &);
    bool contains(const Point &) const;

    std::pair<ForwardIt, ForwardIt> range(const Rect &) const;
    ForwardIt begin() const;
    ForwardIt end() const;

    std::optional<Point> nearest(const Point &) const;
    std::pair<ForwardIt, ForwardIt> nearest(const Point & p, std::size_t k) const;

    friend std::ostream & operator << (std::ostream &, const PointSet &);

};

} // end of namespace rbtree

namespace kdtree {

class PointSet {
    struct Node;
    class dfs_iterator;

    Node * m_root;
    std::size_t m_size;

public:
    using ForwardIt = dfs_iterator;

    PointSet();
    ~PointSet();

    bool empty() const;
    std::size_t size() const;
    void put(const Point &);
    bool contains(const Point &) const;

    std::pair<ForwardIt, ForwardIt> range(const Rect &) const;
    ForwardIt begin() const;
    ForwardIt end() const;

    std::optional<Point> nearest(const Point &) const;
    std::pair<ForwardIt, ForwardIt> nearest(const Point & p, std::size_t k) const;

    friend std::ostream & operator << (std::ostream &, const PointSet &);

private:
    void put(Node * &, const Point &, const Rect &, unsigned);
    bool contains(const Node *, const Point &) const;
    void range(const Node *, const Rect &, PointSet &) const;

    template <typename Set>
    void nearest(const Node *, const Point &, std::size_t, Set &) const;

};

class PointSet::dfs_iterator {
    friend class PointSet;

    dfs_iterator(const PointSet &, std::size_t);
    void traverse(Node *);

public:
    using value_type = const Point;
    using difference_type = std::ptrdiff_t;
    using iterator_category = std::forward_iterator_tag;
    using pointer = value_type *;
    using reference = value_type &;

    dfs_iterator(const dfs_iterator &) = default;
    dfs_iterator(dfs_iterator &&) = default;

    reference operator * () const;
    pointer operator -> () const;

    dfs_iterator & operator ++ ();
    dfs_iterator operator ++ (int);

    bool operator == (const dfs_iterator & other) const;
    bool operator != (const dfs_iterator & other) const;

private:
    std::size_t m_current;
    std::vector<Point> m_traversal;
};

} // end of namespace kdtree
