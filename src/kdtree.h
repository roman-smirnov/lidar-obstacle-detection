#ifndef KDTREE_H_
#define KDTREE_H_

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
	int dim;

	KdTree(int dim)
	: root(NULL), dim(dim)
	{}

	void insertHelpler(Node** node, uint depth, std::vector<float> point, int id)
	{
		if(*node == NULL)
			*node = new Node(point, id);
		else
		{
			// Current cluster dim 
			uint cd = depth % dim;
			if(point[cd] < ((*node)->point[cd]))
				insertHelpler(&((*node)->left), depth+1, point, id);
			else
				insertHelpler(&((*node)->right), depth+1, point, id);

		}
	}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		insertHelpler(&root, 0, point, id);
	}

	float distSquare(std::vector<float>& p1, std::vector<float>& p2)
	{
		float dis2 = 0.;
		for(int i=0; i < dim; i++)
			dis2 += (p1[i] - p2[i])*(p1[i] - p2[i]);

		return dis2;
	}

	bool isWithinBox(std::vector<float>& p1, std::vector<float>& p2, float distanceTol)
	{
		bool res = true;
		for(int i=0; i < dim; i++)
			res = res && (p1[i]-distanceTol)<p2[i] && (p1[i]+distanceTol)>p2[i];

		return res;
	}

	void searchHelpler(Node* node, uint depth, std::vector<float> point, std::vector<int>& ids, float distanceTol)
	{
		if(node == NULL)
			return;
		// Current cluster dim 
		uint cd = depth % dim;
		if(isWithinBox(point, node->point, distanceTol) && distSquare(point, node->point) < distanceTol*distanceTol)
		{
			ids.push_back(node->id);
			searchHelpler(node->left, depth+1, point, ids, distanceTol);
			searchHelpler(node->right, depth+1, point, ids, distanceTol);
		} 
		else if(point[cd] < (node->point[cd]))
		{
			searchHelpler(node->left, depth+1, point, ids, distanceTol);
			if((point[cd]-distanceTol) < node->point[cd] && (point[cd]+distanceTol) > node->point[cd])
				searchHelpler(node->right, depth+1, point, ids, distanceTol);
		}
		else
		{
			searchHelpler(node->right, depth+1, point, ids, distanceTol);
			if((point[cd]-distanceTol) < node->point[cd] && (point[cd]+distanceTol) > node->point[cd])
				searchHelpler(node->left, depth+1, point, ids, distanceTol);
		}
		
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelpler(root, 0, target, ids, distanceTol);
		return ids;
	}
	

};


#endif /* KDTREE_H_*/
