#include "Node.h"
#include "SpatialSearch.h"


Node::Node(int max_data, Point lowerLeft, Point upperRight) : m_max_data_size(max_data),
				m_northwest(std::shared_ptr<Node>(0)),
				m_southwest(std::shared_ptr<Node>(0)),
				m_southeast(std::shared_ptr<Node>(0)),
				m_northeast(std::shared_ptr<Node>(0)),
				m_lower_left_bound(std::shared_ptr<Point>(new Point(lowerLeft))),
				m_upper_right_bound(std::shared_ptr<Point>(new Point(upperRight)))
{
	this->m_data.reserve(max_data);
}

void Node::InsertData(const Point* p)
{
	// insert the point into the quadtree using the following steps
	
	// 1 - See if point is included in the current node. 
	if(IsPointInNode(p))
	{
		// 2 - See if children are null. If children are null, this is a leaf and 
		//     we can insert it into the current data array. If not, go to 
		//     appropriate sub-child based on position.
		if(IsLeaf())
		{
			this->m_data.push_back(p);
			
			// 3 - If we've inserted into current data array
			//     a - if data length is > node array max size, split
			if(this->m_data.size() > this->m_max_data_size)
			{
				//split into four quadrants, then repopulate children with node data
				Point midpoint;
				midpoint.x = (m_lower_left_bound->x + m_upper_right_bound->x)/2.0;
				midpoint.y = (m_lower_left_bound->y + m_upper_right_bound->y)/2.0;
			
				// create northwest
				Point nwll, nwur;
				nwll.x = m_lower_left_bound->x;
				nwll.y = midpoint.y;
				nwur.x = midpoint.x;
				nwur.y = m_upper_right_bound->y;
				m_northwest = std::shared_ptr<Node>(new Node(m_max_data_size, nwll, nwur));

				// create northeast		
				m_northeast = std::shared_ptr<Node>(new Node(m_max_data_size, midpoint, *m_upper_right_bound));

				// create southwest
				m_southwest = std::shared_ptr<Node>(new Node(m_max_data_size, *m_lower_left_bound, midpoint));

				// create southeast
				Point sell, seur;
				sell.x = midpoint.x;
				sell.y = m_lower_left_bound->y;
				seur.x = m_upper_right_bound->x;
				seur.y = midpoint.y;
				m_southeast = std::shared_ptr<Node>(new Node(m_max_data_size, sell, seur));
			
				// For each point, determine the correct quadrant and add it.
				for(std::vector<const Point*>::iterator iter = m_data.begin();
					iter != m_data.end();
					++iter)
				{
					AddPointToCorrectSubnode((*iter));
				}
				// finally, clear the array
				m_data.clear();
			}
		}
		else
		{
			AddPointToCorrectSubnode(p);
		}
	}
}

bool Node::IsLeaf()
{
	if(!m_northwest && !m_southwest && !m_southeast && !m_northeast)
		return true;
	else
		return false;
}

bool Node::IsPointInNode(const Point* p)
{
	return (p->x >= m_lower_left_bound->x 
			&& p->x <= m_upper_right_bound->x 
			&& p->y >= m_lower_left_bound->y 
			&& p->y <= m_upper_right_bound->y);
}

void Node::AddPointToCorrectSubnode(const Point* p)
{
	// send it to the proper sub-node
	if(m_southeast->IsPointInNode(p))
		m_southeast->InsertData(p);
	else if(m_southwest->IsPointInNode(p))
		m_southwest->InsertData(p);
	else if(m_northwest->IsPointInNode(p))
		m_northwest->InsertData(p);
	else if(m_northeast->IsPointInNode(p))
		m_northeast->InsertData(p);
	else
	{ 
		// this should only be hit if there is a bug in the code. Handle 
		// appropriately
	}
}

std::vector<const Point*> Node::RecursiveSearch(const Point& p, float radius)
{
	std::vector<const Point*> result;
	// hit each of the four quadrants
	if(m_northeast->IntersectsWithCircle(p, radius))
	{
		if(m_northeast->IsLeaf())
		{
			result.insert(result.end(), m_northeast->m_data.begin(), m_northeast->m_data.end());
		}
		else
		{
			std::vector<const Point*> interim = m_northeast->RecursiveSearch(p, radius);
			result.insert(result.end(), interim.begin(), interim.end());
		}
	}
	
	if(m_northwest->IntersectsWithCircle(p, radius))
	{
		if(m_northwest->IsLeaf())
		{
			result.insert(result.end(), m_northwest->m_data.begin(), m_northwest->m_data.end());
		}
		else
		{
			std::vector<const Point*> interim = m_northwest->RecursiveSearch(p, radius);
			result.insert(result.end(), interim.begin(), interim.end());
		}
	}

	if(m_southeast->IntersectsWithCircle(p, radius))
	{
		if(m_southeast->IsLeaf())
		{
			result.insert(result.end(), m_southeast->m_data.begin(), m_southeast->m_data.end());
		}
		else
		{
			std::vector<const Point*> interim = m_southeast->RecursiveSearch(p, radius);
			result.insert(result.end(), interim.begin(), interim.end());
		}
	}
	
	if(m_southwest->IntersectsWithCircle(p, radius))
	{
		if(m_southwest->IsLeaf())
		{
			result.insert(result.end(), m_southwest->m_data.begin(), m_southwest->m_data.end());
		}
		else
		{
			std::vector<const Point*> interim = m_southwest->RecursiveSearch(p, radius);
			result.insert(result.end(), interim.begin(), interim.end());
		}
	}
	return result;
}

bool Node::IntersectsWithCircle(const Point& p, float radius)
{	
	// Code adapted from http://stackoverflow.com/questions/401847/circle-rectangle-collision-detection-intersection
	Point midpoint;
	midpoint.x = (m_lower_left_bound->x + m_upper_right_bound->x)/2.0;
	midpoint.y = (m_lower_left_bound->y + m_upper_right_bound->y)/2.0;
			
	Point circleDistance;
	
	circleDistance.x = abs(p.x - midpoint.x);
    circleDistance.y = abs(p.y - midpoint.y);

	float width, height;
	width = m_upper_right_bound->x - m_lower_left_bound->x;
	height = m_upper_right_bound->y - m_lower_left_bound->y;

    if (circleDistance.x > (width/2 + radius)) { return false; }
    if (circleDistance.y > (height/2 + radius)) { return false; }

    if (circleDistance.x <= (width/2)) { return true; } 
    if (circleDistance.y <= (height/2)) { return true; }

	float cornerDistance_sq;
    cornerDistance_sq = pow((circleDistance.x - width/2),2) +
                         pow((circleDistance.y - height/2),2);

    return (cornerDistance_sq <= (pow(radius, 2)));
}