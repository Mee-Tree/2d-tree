#pragma once

class Point {
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

namespace rbtree {

class PointSet {
public:

    using ForwardIt = ...;

    PointsSet();

    bool empty() const;
    std::size_t size() const;
    void put(const Point &);
    bool contains(const Point &) const;

    std::pair<ForwardIt, ForwardIt> range(const Rect &) const;
    ForwardIt begin() const;
    ForwardIt end() const;

    std::optional<Point> nearest(const Point &) const;

    friend std::ostream & operator << (const PointsSet &);

};

}

namespace kdtree {

class PointSet {
public:

    using ForwardIt = ...;

    PointsSet();

    bool empty() const;
    std::size_t size() const;
    void put(const Point &);
    bool contains(const Point &) const;

    std::pair<ForwardIt, ForwardIt> range(const Rect &) const;
    ForwardIt begin() const;
    ForwardIt end() const;

    std::optional<Point> nearest(const Point &) const;

    friend std::ostream & operator << (const PointsSet &);

};

}
