#include "primitives.h"

#include <vector>
#include <experimental/iterator>

namespace kdtree {

/* ------------------Node------------------- */

struct PointSet::Node {
    const Point point;
    const Rect rect;
    unsigned depth;

    Node * left;
    Node * right;

    Node(const Point & p, const Rect & r, unsigned depth = 0)
            : point(p), rect(r), depth(depth)
            , left(nullptr), right(nullptr) {}

    bool operator < (const Point & other) const {
        if (depth % 2 == 0) {
            return point < other;
        }
        return point.y() < other.y() ||
               (point.y() == other.y() && point.x() < other.x());
    }

    bool operator == (const Point & other) const {
        return point == other;
    }

    Point resize(const Point & p) {
        if (depth % 2 == 0) {
            return Point(point.x(), p.y());
        }
        return Point(p.x(), point.y());
    }

    Rect get_resized_rect(const Point & p) {
        Point left_bottom(rect.xmin(), rect.ymin());
        Point right_top(rect.xmax(), rect.ymax());
        if (*this < p) {
            return Rect(resize(left_bottom), right_top);
        }
        return Rect(left_bottom, resize(right_top));
    }
};

/* ---------------DfsIterator--------------- */

PointSet::dfs_iterator::dfs_iterator(const PointSet & ps, std::size_t ind = 0)
        : m_current(ind) {
    m_traversal.reserve(ps.size());
    traverse(ps.m_root);
}

void PointSet::dfs_iterator::traverse(Node * node) {
    m_traversal.push_back(node->point);
    if (node->left != nullptr) {
        traverse(node->left);
    }
    if (node->right != nullptr) {
        traverse(node->right);
    }
}

PointSet::dfs_iterator::reference PointSet::dfs_iterator::operator * () const {
    return m_traversal[m_current];
}

PointSet::dfs_iterator::pointer PointSet::dfs_iterator::operator -> () const {
    return &m_traversal[m_current];
}

PointSet::dfs_iterator & PointSet::dfs_iterator::operator ++ () {
    ++m_current;
    return *this;
}

PointSet::dfs_iterator PointSet::dfs_iterator::operator ++ (int) {
    auto tmp(*this);
    operator ++ ();
    return tmp;
}

bool PointSet::dfs_iterator::operator == (const dfs_iterator & other) const {
    return m_current == other.m_current &&
           m_traversal == other.m_traversal;
}

bool PointSet::dfs_iterator::operator != (const dfs_iterator & other) const {
    return !(operator == (other));
}

/* ----------------PointSet----------------- */

PointSet::PointSet()
    : m_root(nullptr), m_size(0) {}

PointSet::~PointSet() {
    delete m_root;
}

PointSet::ForwardIt PointSet::begin() const {
    return dfs_iterator(*this);
}

PointSet::ForwardIt PointSet::end() const {
    return dfs_iterator(*this, size());
}

bool PointSet::empty() const {
    return m_size == 0;
}

std::size_t PointSet::size() const {
    return m_size;
}

void PointSet::put(const Point & p) {
    put(m_root, p, Rect(Point(0., 0.), Point(1., 1.)), 0);
}

void PointSet::put(Node * & node, const Point & p, const Rect & rect, unsigned depth) {
    if (node == nullptr) {
        node = new Node(p, rect, depth);
        ++m_size;
    } else if (*node == p) {
        // ignored
    } else if (*node < p) {
        put(node->right, p, node->get_resized_rect(p), depth + 1);
    } else {
        put(node->left, p, node->get_resized_rect(p), depth + 1);
    }
}

bool PointSet::contains(const Point & p) const {
    return !empty() && contains(m_root, p);
}

bool PointSet::contains(const Node * node, const Point & p) const {
    if (node == nullptr) {
        return false;
    } else if (*node == p) {
        return true;
    } else if (*node < p) {
        return contains(node->right, p);
    }
    return contains(node->left, p);
}

std::optional<Point> PointSet::nearest(const Point & p) const {
    if (empty()) {
        return std::nullopt;
    }
    Point res(m_root->point);
    nearest(m_root, p, res);
    return res;
}

void PointSet::nearest(const Node * node, const Point & p, Point & res) const {
    if (node == nullptr || node->rect.distance(p) > p.distance(res)) {
        return;
    } else if (p.distance(node->point) < p.distance(res)) {
        res = node->point;
    }
    if (node->left != nullptr && node->right != nullptr) {
        double left_dist = node->left->rect.distance(p);
        double right_dist = node->right->rect.distance(p);

        if (left_dist < right_dist) {
            nearest(node->left, p, res);
            if (right_dist < p.distance(res)) {
                nearest(node->right, p, res);
            }
        } else {
            nearest(node->right, p, res);
            if (left_dist < p.distance(res)) {
                nearest(node->left, p, res);
            }
        }
    } else if (node->left != nullptr && node->left->rect.distance(p) < p.distance(res)) {
        nearest(node->left, p, res);
    } else if (node->right != nullptr && node->right->rect.distance(p) < p.distance(res)) {
        nearest(node->right, p, res);
    }
}

std::pair<PointSet::ForwardIt, PointSet::ForwardIt>
PointSet::range(const Rect & rect) const {
    PointSet result;
    range(m_root, rect, result);
    return std::make_pair(result.begin(), result.end());
}

void PointSet::range(const Node * node, const Rect & rect, PointSet & res) const {
    if (node == nullptr || !node->rect.intersects(rect)) {
        return;
    } else if (rect.contains(node->point)) {
        res.put(node->point);
    }
    range(node->left, rect, res);
    range(node->right, rect, res);
}

std::pair<PointSet::ForwardIt, PointSet::ForwardIt>
PointSet::nearest(const Point & p, std::size_t k) const {
    PointSet result;
    std::set<Point, decltype(utils::distance_cmp(p))> set(utils::distance_cmp(p));
    nearest(m_root, p, k, set);
    for (auto point : set) {
        result.put(point);
    }
    return std::make_pair(result.begin(), result.end());
}

template <typename Set>
void PointSet::nearest(const Node * node, const Point & p, std::size_t k, Set & set) const {
    if (node == nullptr) {
        return;
    } else if (set.size() < k || p.distance(node->point) <= p.distance(*set.rbegin())) {
        set.insert(node->point);
        if (set.size() > k) {
            set.erase(*set.rbegin());
        }
    }
    if (node->left != nullptr && node->right != nullptr) {
        double left_dist = node->left->rect.distance(p);
        double right_dist = node->right->rect.distance(p);

        if (left_dist < right_dist) {
            nearest(node->left, p, k, set);
            if (right_dist < p.distance(*set.rbegin())) {
                nearest(node->right, p, k, set);
            }
        } else {
            nearest(node->right, p, k, set);
            if (left_dist < p.distance(*set.rbegin())) {
                nearest(node->left, p, k, set);
            }
        }
    } else if (node->left != nullptr && node->left->rect.distance(p) < p.distance(*set.rbegin())) {
        nearest(node->left, p, k, set);
    } else if (node->right != nullptr && node->right->rect.distance(p) < p.distance(*set.rbegin())) {
        nearest(node->right, p, k, set);
    }
}

std::ostream & operator << (std::ostream & strm, const PointSet & ps) {
    strm << "{";
    std::copy(ps.begin(), ps.end(),
              std::experimental::make_ostream_joiner(strm, "; "));
    return strm << "}";
}

} // end of namespace kdtree
