/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(nullptr), right(nullptr)
	{}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(nullptr)
	{}

	void insert(std::vector<float> point, int id)
	{
		insertHelper(&root, 0, point, id);

	}

	void insertHelper(Node** node, uint depth, std::vector<float> point, int id)
	{
		//Case Empty Tree
		if(* node == nullptr)
		{
			*node = new Node(point, id);
		}
		else
		{
			//even or odd?
			uint cd = depth%2;

			if(point[cd] < ((*node)->point[cd])) 
				insertHelper( &((*node)->left),  depth+1, point, id);
			else 
				insertHelper( &((*node)->right), depth+1, point, id);
		}
			
	}

	void searchHelper( std::vector<float> target, Node* node, int depth, float distanceTol, std::vector<int> & ids)
	{
		if(node != nullptr)
		{
			//check the distance and if it is under tol then add to ids vector
			if( (node->point[0] >= (target[0] - distanceTol) && node->point[0] <= (target[0] + distanceTol))  //x
				&& (node->point[1] >= (target[1] - distanceTol) && node->point[1] <= (target[1] + distanceTol)) ) //y
			{
				float distance = sqrt(((node->point[0] - target[0]) * (node->point[0] - target[0])) + ((node->point[1] - target[1]) * (node->point[1] - target[1])));
				if (distance <= distanceTol) ids.push_back(node->id);
			}
			//check boundary
			if((target[depth % 2] - distanceTol) < node->point[depth % 2])
			{
				if(node->left != nullptr) searchHelper(target, node->left, depth+1, distanceTol, ids);
			}
			if((target[depth % 2] + distanceTol) > node->point[depth % 2]){
				if(node->right != nullptr)	searchHelper(target, node->right,  depth+1, distanceTol, ids);
			}	
		}
	}
	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search( std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(target, root, 0, distanceTol, ids);
		return ids;
	}
	

};




