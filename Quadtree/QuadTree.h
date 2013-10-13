#pragma once

#include <set>
#include <memory>
#include <vector>

struct Point;

struct PointCompare
{
	bool operator()(const Point* lhs, const Point* rhs);

	static Point comparisonPoint;
};

struct Node;

// This class implements a Region QuadTree.
class QuadTree
{
public:
	QuadTree(int max_size);
	~QuadTree(void);

	void Insert(const Point* p);
	std::vector<const Point*> SearchQuadrantsAroundPoint(const Point& p, float radius, int max_quad_results);

	int m_max_node_size;

	std::shared_ptr<Node> m_tree_root;
};

