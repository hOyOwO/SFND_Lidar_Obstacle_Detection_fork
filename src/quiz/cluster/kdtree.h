#ifndef KDTREE_H_
#define KDTREE_H_

//#include "../../render/render.h"
#include <vector>
#include <cmath>
#include <unordered_set>

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

	~Node()
	{
		delete left;
		delete right;
	}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	~KdTree()
	{
		delete root;
	}

	void insertHelper(Node** node, uint depth, std::vector<float> point, int id)
	{
		if(*node == NULL)
		{
			*node = new Node(point, id);
		}
		else
		{
			if(point[depth%2] < ((*node)->point[depth%2]) )
			{
				insertHelper( &((*node)->left), depth+1, point, id);
			}
			else
			{
				insertHelper( &((*node)->right), depth+1, point, id);
			}
				
		}
	}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		insertHelper(&root, 0, point, id);

		

	}

	void searchHelper(std::vector<float> target, Node* node, int depth, float distanceTol, std::vector<int> &ids)
	{
			if(node != NULL)
			{
				float dis_x = fabs(target[0]-node->point[0]);
				float dis_y = fabs(target[1]-node->point[1]);
				if( dis_x <= distanceTol && dis_y <= distanceTol )
				{
					float dis = sqrt(dis_x * dis_x + dis_y * dis_y);
					if (dis <= distanceTol)
						ids.push_back(node->id);
						//cout << node->id << endl;
				}
				if ( target[depth%2]- distanceTol <  node->point[depth%2]) // target의 최소값보다 node가 오른쪽에 있을때 node의 왼쪽을 살림
					searchHelper(target, node->left, depth+1, distanceTol, ids);
				if ( target[depth%2] + distanceTol >  node->point[depth%2]) // target의 최대값보다 node가 왼쪽에 있을때 node의 오른쪽을 살림
					searchHelper(target, node->right, depth+1, distanceTol, ids);
				 
			}

	}
	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(target, root, 0, distanceTol, ids);

		return ids;
	}

	
	

};

void Proximity(int index, std::vector<int> &cluster, std::vector<bool> &processed, const std::vector<std::vector<float>> points, KdTree* tree, float distanceTol);
std::vector<std::vector<int>> euclideanCluster_hyw(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol, int minSize, int maxSize);


#endif // KDTREE_H_
