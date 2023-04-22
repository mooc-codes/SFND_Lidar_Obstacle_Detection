
#include <vector>

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
			int d = depth % 2;
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

		bool x_within_bounds = current[0] >= xLims[0] && current[0] <= xLims[1];
		bool y_within_bounds = current[1] >= yLims[0] && current[1] <= yLims[1];
		// Check if current node's point is within the box
		if (x_within_bounds && y_within_bounds)
		{
			//Compute distance of point from target.
			float dx = (target[0] - current[0]) * (target[0] - current[0]);
			float dy = (target[1] - current[1]) * (target[1] - current[1]);
			float dist = sqrt( dx + dy );
			if(dist <= distanceTol)
			{
				ids.push_back(node->id);
			}
		}

		// If current node is out of box
		int dim = depth % 2; // The dimension to be compared.
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