#include <gtest/gtest.h>

#include "utils.h"
#include "primitives.h"

#include <iostream>
#include <fstream>

void load_data(const std::string & filename, PointsSet & p)
{
	try {
		std::ifstream fs(filename);

		double x, y;
		while (!fs.eof()) {
			fs >> x >> y;
			p.put(Point(x, y));
		}

		std::cout << "Loaded " << p.size() << " points.\n";
	} catch(...) {
		std::cout << "Can't read " << filename << ".\n";
	}
}

TEST(PointSetTest, Point)
{
	ASSERT_TRUE(Point(1., 2.) == Point(1., 2.));
	ASSERT_TRUE(Point(1., 2.) != Point(5., 4.));
	ASSERT_EQ(Point(0, 0).distance(Point(1, 0)), 1.);
	ASSERT_EQ(Point(0, 0).distance(Point(0, 1)), 1.);
	ASSERT_EQ(Point(0, 4).distance(Point(3, 0)), 5.);
}

TEST(PointSetTest, Rect)
{
	Rect r(Point(1., 1.), Point(2., 2.));
	ASSERT_EQ(r.distance(Point(1., 1.)), 0.);
	ASSERT_EQ(r.distance(Point(1.5, 1.5)), 0.);
	ASSERT_EQ(r.distance(Point(0., 1.)), 1.);
	ASSERT_TRUE(r.contains(Point(1.5, 1.5)));
	ASSERT_FALSE(r.contains(Point(.9, 1.5)));
	ASSERT_TRUE(r.intersects(Rect(Point(0., 0.), Point(1.5, 1.5))));
	ASSERT_TRUE(r.intersects(Rect(Point(0.5, 0.5), Point(3.5, 3.5))));
	ASSERT_FALSE(r.intersects(Rect(Point(1.1, 0.1), Point(3.5, 1.9))));
}

TEST(PointSetTest, PointSetBasic)
{
	PointsSet p;
	ASSERT_TRUE(p.empty());
	ASSERT_EQ(p.size(), 0);

	p.put(Point(0, 0));
	p.put(Point(1., 1.));
	p.put(Point(.5, .5));
	ASSERT_FALSE(p.empty());
	ASSERT_EQ(p.size(), 3);
	ASSERT_TRUE(p.contains(Point(0,0)));
	ASSERT_FALSE(p.contains(Point(0.5,0)));
}

TEST(PointSetTest, PointSetSearch)
{
	PointsSet p;
	p.put(Point(0, 0));
	p.put(Point(1., 1.));
	p.put(Point(.5, .5));
	
	auto n = p.nearest(Point(.4, .4));
	ASSERT_TRUE(Point(.5, .5) == *n);

	auto it_p = p.range(Rect(Point(0.3, 0.3), Point(.7, .7)));
	ASSERT_TRUE(*it_p.first == *it_p.second);
	EXPECT_DOUBLE_EQ(it_p.first->x(), 0.5);
	EXPECT_DOUBLE_EQ((*it_p.first).y(), 0.5);
}

TEST(PointSetTest, PointSetSearch2)
{
	PointsSet p;

	load_data("test/test0.dat", p);
	auto n = p.nearest(Point(.74, .29));
	ASSERT_TRUE(Point(0.725, 0.338) == *n);
}

TEST(PointSetTest, PointSetSearch3)
{
	PointsSet p;

	load_data("test/test1.dat", p);

	auto it_p = p.range(Rect(Point(0.634, 0.276), Point(.818, .42)));
	
	auto it = it_p.first;
	ASSERT_TRUE(*it == Point(0.655, 0.382));
	ASSERT_TRUE(*(++it) == Point(0.725, 0.311));
	ASSERT_TRUE(*(++it) == Point(0.794, 0.299));
	ASSERT_TRUE(*it == *it_p.second); 
}

TEST(PointSetTest, PointSetSearch4)
{
	PointsSet p;

	load_data("test/test2.dat", p);
	auto n = p.nearest(Point(.712, .567));
	ASSERT_TRUE(Point(0.718, 0.555) == *n);
}
