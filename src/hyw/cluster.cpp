#include "kdtree_hyw.h"
#include <iostream> 

void Proximity(int index, std::vector<int> &cluster, std::vector<bool> &processed, const std::vector<std::vector<float>> points, KdTree* tree, float distanceTol)
{
	processed[index] = true;
	cluster.push_back(index);
	std::vector<int> nearby = tree->search(points[index],distanceTol);
    

	for(int j = 0 ; j < nearby.size(); j++)
    	{
            

			if(processed[nearby[j]] == false)
            {
                //std::cout << nearby.size() << std::endl;
                Proximity(nearby[j], cluster, processed, points, tree, distanceTol);

            }
                
		}
    

}

std::vector<std::vector<int>> euclideanCluster_hyw(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol, int minSize, int maxSize)
{

	// TODO: Fill out this function to return list of indices for each cluster


	std::vector<std::vector<int>> clusters; // list of clusters
	std::vector<bool> processed(points.size(), false);
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	std::vector<int> cluster;

	for (int i = 0 ; i < points.size(); i++) 
	{
		if (processed[i] == false) // iterate through each point
		{
			cluster.clear();
			Proximity(i, cluster, processed, points, tree, distanceTol);
			 if (minSize < cluster.size() && cluster.size() < maxSize) // Check cluster size
            {
                clusters.push_back (cluster); // cluster add clusters
            }
		}

    }

	return clusters;

}