#pragma once

#include <ostream>
#include <optional>
#include <vector>
#include <set>
#include <memory>

struct Point {
    Point();
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

private:
    double m_x;
    double m_y;
};

struct Rect {
    Rect();
	Rect(const Point & left_bottom, const Point & right_top);

	double xmin() const;
	double ymin() const;
	double xmax() const;
	double ymax() const;
	double distance(const Point & p) const;

	bool contains(const Point & p) const;
	bool intersects(const Rect &) const;

private:
    Point m_left_bottom;
    Point m_right_top;
};

namespace utils {

static inline auto distance_cmp(const Point & p) {
    return [&p](const Point & a, const Point & b) {
        return p.distance(a) < p.distance(b);
    };
}

template <class Container>
class put_iterator {
    Container * container;

public:
    using iterator_category = std::output_iterator_tag;
    using value_type = void;
    using difference_type = void;
    using pointer = void;
    using reference = void;
    using container_type = Container;

    put_iterator (Container& c) : container(std::addressof(c)) {}

    auto & operator = (const typename Container::value_type & value) {
        container->put(value);
        return *this;
    }
    auto & operator = (typename Container::value_type && value) {
        container->put(std::move(value));
        return *this;
    }
    auto & operator * ()   { return *this; }
    auto & operator ++ ()  { return *this; }
    auto operator ++ (int) { return *this; }
};

template <class Container>
static inline put_iterator<Container> putter(Container & c) {
    return put_iterator(c);
}

template <class T>
class forward_iterator_wrapper {
    std::vector<T> m_data;
    std::size_t m_cur;

public:
    using iterator_category = std::forward_iterator_tag;
    using value_type = const T;
    using difference_type = ptrdiff_t;
    using pointer = value_type *;
    using reference = value_type &;

    template <template <class ...> class Container>
    forward_iterator_wrapper(const Container<T> & c, std::size_t ind = 0)
        : m_data(c.size()), m_cur(ind) {
        std::copy(c.begin(), c.end(), m_data.begin());
    }

    reference operator * () const { return m_data[m_cur]; }
    pointer operator -> ()  const { return &m_data[m_cur]; }

    auto & operator ++ ()  { ++m_cur; return *this; }
    auto operator ++ (int) { auto tmp(*this); ++m_cur; return tmp; }

    bool operator == (const forward_iterator_wrapper & other) const {
        return m_data == other.m_data && m_cur == other.m_cur;
    }
    bool operator != (const forward_iterator_wrapper & other) const {
        return !(*this == other);
    }
};

} // end of namespace utils

namespace rbtree {

class PointSet {
    using Data = std::set<Point>;

    struct Query {
        using value_type = Data::value_type;
        using const_iterator = utils::forward_iterator_wrapper<value_type>;

        Query() = default;
        Query(std::vector<value_type> && other) : m_query(std::move(other)) {}

        auto begin() const { return const_iterator(m_query); }
        auto end()   const { return const_iterator(m_query, m_query.size()); }

        void push_back(const value_type & v) { m_query.emplace_back(v); }
        void push_back(value_type && v)      { m_query.emplace_back(std::move(v)); }

    private:
        std::vector<value_type> m_query;
    };

public:
    using ForwardIt = Data::const_iterator;
    using QueryIt = Query::const_iterator;

    PointSet();

    bool empty() const;
    std::size_t size() const;
    void put(const Point &);
    bool contains(const Point &) const;

    std::pair<QueryIt, QueryIt> range(const Rect &) const;
    ForwardIt begin() const;
    ForwardIt end() const;

    std::optional<Point> nearest(const Point &) const;
    std::pair<QueryIt, QueryIt> nearest(const Point & p, std::size_t k) const;

    friend std::ostream & operator << (std::ostream &, const PointSet &);

private:
    Data m_data;

};

} // end of namespace rbtree

namespace kdtree {

class PointSet {
    struct Node;
    class dfs_iterator;

    using NodePtr = std::unique_ptr<Node>;
public:
    using ForwardIt = dfs_iterator;

    PointSet();
    PointSet(const PointSet &);
    PointSet(PointSet &&) = default;

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
    void put(NodePtr &, const Point &, const Rect &, unsigned);
    bool contains(const NodePtr &, const Point &) const;
    void range(const NodePtr &, const Rect &, PointSet &) const;

    template <typename Set>
    void nearest(const NodePtr &, const Point &, std::size_t, Set &) const;

    NodePtr m_root;
    std::size_t m_size;

};

class PointSet::dfs_iterator {
    friend class PointSet;

    dfs_iterator(const PointSet &, std::size_t);
    void traverse(const NodePtr &);

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

    dfs_iterator & operator = (const dfs_iterator & other) = default;
    dfs_iterator & operator = (dfs_iterator && other) = default;

private:
    std::size_t m_current;
    std::vector<Point> m_traversal;
};

} // end of namespace kdtree
