#include "StdAfx.h"
#include "QuadTree.h"
#include "Node.h"
#include "SpatialSearch.h"
#include <math.h>

// Initialize point comparator
Point PointCompare::comparisonPoint = Point();


bool PointCompare::operator()(const Point* lhs, const Point* rhs)
{
	// There are two ways to manage this. At its core, we want to see which 
	// point is closer to the reference point, and handle insertion into the
	// result map based upon that data.

	// Along those lines, we can simply use the point distance formula. However, 
	// doing a sqrt() at every insertion could potentially get expensive. In 
	// most cases, the square of the distance (which is much quicker to 
	// calculate) can be more than sufficient. Only makes sense to add the sqrt
	// if we start seeing conflicts between colocated points
	double lhsDist, rhsDist;

	lhsDist = pow(lhs->x - PointCompare::comparisonPoint.x,2) + pow(lhs->y - PointCompare::comparisonPoint.y,2);
	rhsDist = pow(rhs->x - PointCompare::comparisonPoint.x,2) + pow(rhs->y - PointCompare::comparisonPoint.y,2);

	return lhsDist < rhsDist;
}



QuadTree::QuadTree(int max_size)
{
	m_max_node_size = max_size;
	Point ll, ur;
	// Choose a suitably 'large' region for this Quadtree
	ll.x = ll.y = -999999999;
	ur.x = ur.y = 999999999;
	m_tree_root = std::shared_ptr<Node>(new Node(max_size, ll, ur));
}


QuadTree::~QuadTree(void)
{
}


void QuadTree::Insert(const Point* p)
{
	m_tree_root->InsertData(p);
}

std::vector<const Point*> QuadTree::SearchQuadrantsAroundPoint(const Point& p, float radius, int max_quad_results)
{
	std::vector<const Point*> nodes_to_check = m_tree_root->RecursiveSearch(p, radius);
	std::multiset<const Point*,PointCompare> quad_0, quad_1, quad_2, quad_3;

	// Copy the point into the comparator's static variable, so that comparisons
	// are conducted correctly when inserting into multimaps
	PointCompare::comparisonPoint.x = p.x;
	PointCompare::comparisonPoint.y = p.y;
	float radius_squared = radius * radius;


	// for each point in the node
	for(std::vector<const Point*>::iterator point_iter = nodes_to_check.begin();
		point_iter != nodes_to_check.end();
		++point_iter)
	{
		Point new_point;
		new_point.x = (*point_iter)->x;
		new_point.y = (*point_iter)->y;
		// if point inside radius
		float dist_squared = pow(new_point.x - p.x, 2) + pow(new_point.y - p.y, 2);
		if(dist_squared <= radius_squared)
		{
			// determine quadrant
			if(p.x == new_point.x && p.y == new_point.y)
			{
				quad_0.insert((*point_iter));
			}
			// handle special cases of on the lines
			else if(new_point.x == p.x)
			{
				if(new_point.y > p.y)
				{
					quad_1.insert((*point_iter));
				}
				else
				{
					quad_3.insert((*point_iter));
				}
			}
			else if(new_point.y == p.y)
			{
				if(new_point.x > p.x)
				{
					quad_0.insert((*point_iter));
				}
				else
				{
					quad_2.insert((*point_iter));
				}
			}
			// add to appropriate quadrant result
			else if(new_point.x > p.x && new_point.y > p.y) //quad 0
			{
				quad_0.insert((*point_iter));
			}
			else if(new_point.x < p.x && new_point.y > p.y) // quad 1
			{
				quad_1.insert((*point_iter));
			}
			else if(new_point.x < p.x && new_point.y < p.y) // quad 2
			{
				quad_2.insert((*point_iter));
			}
			else // only remaining case is quad 3
			{
				quad_3.insert((*point_iter));
			}
		}
	}
	// concatenate X points from each quadrant
	std::vector<const Point*> result;
	int added_so_far = 0;
	for(std::multiset<const Point*,PointCompare>::iterator iter = quad_0.begin();
		iter != quad_0.end() && added_so_far < max_quad_results;
		++iter)
	{
		result.push_back((*iter));
		++ added_so_far;
	}
	added_so_far = 0;
	for(std::multiset<const Point*,PointCompare>::iterator iter = quad_1.begin();
		iter != quad_1.end() && added_so_far < max_quad_results;
		++iter)
	{
		result.push_back((*iter));
		++ added_so_far;
	}
	added_so_far = 0;

	for(std::multiset<const Point*,PointCompare>::iterator iter = quad_2.begin();
		iter != quad_2.end() && added_so_far < max_quad_results;
		++iter)
	{
		result.push_back((*iter));
		++ added_so_far;
	}
	added_so_far = 0;
	for(std::multiset<const Point*,PointCompare>::iterator iter = quad_3.begin();
		iter != quad_3.end() && added_so_far < max_quad_results;
		++iter)
	{
		result.push_back((*iter));
		++ added_so_far;
	}
	return result;
}

