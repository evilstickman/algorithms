#pragma once
#include <set>
#include <memory>
#include <vector>

struct Point;

// Node for a Region QuadTree. A point tree would be more efficient, but is 
// harder to understand and maintain. The optimization exists, though, if 
// necessary
struct Node
{
	Node(int max_data, Point lowerLeft, Point upperRight);
	void InsertData(const Point* p);

	bool IsLeaf();

	bool IsPointInNode(const Point* p);
	
	void AddPointToCorrectSubnode(const Point* p);

	std::vector<const Point*> RecursiveSearch(const Point& p, float radius);

	bool IntersectsWithCircle(const Point& p, float radius);

	int m_max_data_size;
	std::vector<const Point*> m_data;
	std::shared_ptr<Node> m_northwest;
	std::shared_ptr<Node> m_southwest;
	std::shared_ptr<Node> m_southeast;
	std::shared_ptr<Node> m_northeast;
	std::shared_ptr<Point> m_lower_left_bound;
	std::shared_ptr<Point> m_upper_right_bound;
};