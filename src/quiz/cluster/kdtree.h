/* \author Aaron Brown */
// Quiz on implementing kd tree

#include <cassert>
#include <algorithm>

#include "../../render/render.h"

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

float get_dist(std::vector<float>& target, std::vector<float>& point) {
	float dist = 0.0;
	float diff;
	assert(target.size() == point.size());
	int n_dims = target.size();

	for (int c=0; c < n_dims; c++) {
		diff = target[c] - point[c];
		dist += diff*diff;
	}

	return std::sqrt(dist);
}

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	void insert(Node *&node, std::vector<float> point, int id, int depth)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		uint axis = depth % 2;

		if (node == NULL) {
			node = new Node(point, id);
		}
		else if (point[axis] < node->point[axis]) {
			insert(node->left, point, id, ++depth);
		}
		else {
			insert(node->right, point, id, ++depth);
		}

	}

	void searchHelper(Node *&node, std::vector<float>& target, float distanceTol, int depth, std::vector<int>& ids)
	{

		if (node != NULL) {
			float dist = get_dist(target, node->point);
			if (dist <= distanceTol)
				ids.push_back(node->id);
			
			if ( (target[depth%2] - distanceTol) < node->point[depth%2] )
				searchHelper(node->left, target, distanceTol, ++depth, ids);

			if ( (target[depth%2] + distanceTol) > node->point[depth%2] )
				searchHelper(node->right, target, distanceTol, ++depth, ids);
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		// TODO: tree search
		std::vector<int> ids;
		searchHelper(root, target, distanceTol, 0, ids);
		return ids;
	}
	

};




