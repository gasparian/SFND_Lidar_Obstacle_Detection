#include <set>
#include <vector>
#include <algorithm>

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
	: root(NULL) {}

	void insert(Node *&node, std::vector<float> point, int id, int depth)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		uint k = point.size();
		uint axis = depth % k; 

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

	void insertBalanced(Node *&node, std::vector<std::pair<int, std::vector<float>>> points, int depth)
	{	
		int N = points.size();
		int id = N / 2;
		if (N == 2) id = 0;

		if ( N > 0 ) {
			uint k = points[0].second.size();
			uint axis = depth % k; 

			// sort the points in ascending order
			std::sort(points.begin(), points.end(),
            		  [axis](std::pair<int, std::vector<float>> el1, std::pair<int, std::vector<float>> el2) {return el1.second[axis] < el2.second[axis];});

			node = new Node(points[id].second, points[id].first);

			std::vector<std::pair<int, std::vector<float>>> left_points(points.begin(), points.begin()+id);
			std::vector<std::pair<int, std::vector<float>>> right_points(points.begin()+id+1, points.end());

			insertBalanced(node->left, left_points, ++depth);
			insertBalanced(node->right, right_points, ++depth);
		}
	}

	// get the l2 distance between N-dim vectors
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

	void searchHelper(Node *&node, std::vector<float>& target, float distanceTol, int depth, std::vector<int>& ids)
	{

		uint k = target.size();
		if (node != NULL) {
			if ( (node->point[0]>=(target[0]-distanceTol)&&(node->point[0]<=(target[0]+distanceTol))) &&
			     (node->point[1]>=(target[1]-distanceTol)&&(node->point[1]<=(target[1]+distanceTol))) &&
				 (node->point[2]>=(target[2]-distanceTol)&&(node->point[2]<=(target[2]+distanceTol))) ) {
					 
				float dist = get_dist(target, node->point);
				if (dist <= distanceTol)
					ids.push_back(node->id);
			}
			
			if ( (target[depth%k] - distanceTol) < node->point[depth%k] )
				searchHelper(node->left, target, distanceTol, ++depth, ids);

			if ( (target[depth%k] + distanceTol) >= node->point[depth%k] )
				searchHelper(node->right, target, distanceTol, ++depth, ids);
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		// tree search
		std::vector<int> ids;
		searchHelper(root, target, distanceTol, 0, ids);
		return ids;
	}

};