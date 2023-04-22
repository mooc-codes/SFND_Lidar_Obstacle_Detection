#include <vector>
#include <cstddef>
#include <math.h>
// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	void insertNode(Node** node, int depth, std::vector<float> point, int id)
	{
		if(*node == NULL)
		{
			*node = new Node(point, id);
		}

		else
		{
			int d = depth % 3; //3D point so we have to alternate 3 dims
			if(point[d] > (*node)->point[d])
			{
				insertNode(&((*node)->right), depth + 1, point, id);
			}
			else
			{
				insertNode(&((*node)->left), depth + 1, point, id);
			}
			
		}
	}

	void insert(std::vector<float> point, int id)
	{
		insertNode(&root, 0, point, id);		
	}

	void search(std::vector<int>& ids, std::vector<float>& target, float distanceTol, Node* node, int depth)
	{
		if (node != NULL)
		{
		std::vector<float> current = node->point;  
		std::vector<float> xLims{target[0] - distanceTol, target[0] + distanceTol};
		std::vector<float> yLims{target[1] - distanceTol, target[1] + distanceTol};
		std::vector<float> zLims{target[2] - distanceTol, target[2] + distanceTol};
		bool x_within_bounds = current[0] >= xLims[0] && current[0] <= xLims[1];
		bool y_within_bounds = current[1] >= yLims[0] && current[1] <= yLims[1];
		bool z_within_bounds = current[2] >= zLims[0] && current[2] <= zLims[1];
		// Check if current node's point is within the box
		if (x_within_bounds && y_within_bounds && z_within_bounds)
		{
			//Compute distance of point from target.
			float dx = (target[0] - current[0]) * (target[0] - current[0]);
			float dy = (target[1] - current[1]) * (target[1] - current[1]);
			float dz = (target[2] - current[2]) * (target[2] - current[2]);
			float dist = sqrt( dx + dy + dz);
			if(dist <= distanceTol)
			{
				ids.push_back(node->id);
			}
		}

		// If current node is out of box
		int dim = depth % 3; // The dimension to be compared.
		if((target[dim] - distanceTol) < current[dim])
		{
			search(ids, target, distanceTol, node->left, depth+1);
		}
		if( (target[dim] + distanceTol) > current[dim])
		{
			search(ids, target, distanceTol, node->right, depth+1);
		}
		}
	}

	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		search(ids, target, distanceTol, root, 0);
		return ids;
	}
	
};
