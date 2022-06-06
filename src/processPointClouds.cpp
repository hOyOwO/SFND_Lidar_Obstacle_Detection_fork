// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#include "hyw/kdtree_hyw.h"
#include <unordered_set>
#include <random>
#include <cmath>



//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    //pcl::PCLPointCloud2::Ptr cloud (new pcl::PCLPointCloud2 ());
    
    //cl::PointCloud::Ptr cloud_filtered (new pcl::PointCloud());
    
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>());

    pcl::VoxelGrid<PointT> sor;
    //pcl::VoxelGrid<pcl::PointCloud> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (filterRes, filterRes, filterRes);
    sor.filter (*cloud_filtered);
    
    //CropBox
    typename pcl::PointCloud<PointT>::Ptr cloud_cropped (new pcl::PointCloud<PointT>());
    
    pcl::CropBox<PointT> crop;
    crop.setInputCloud (cloud_filtered);
    crop.setMin (minPoint);
    crop.setMax (maxPoint);
    crop.filter (*cloud_cropped);
        

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    //return cloud;
    //return cloud_filtered;
    return cloud_cropped;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr obstacle (new pcl::PointCloud<PointT> ());
    typename pcl::PointCloud<PointT>::Ptr plane (new pcl::PointCloud<PointT> ());

    for (int index : inliers->indices)
    {
        plane->points.push_back(cloud->points[index]);
    }
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (*obstacle);
       
    
    //std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloud, cloud);
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstacle, plane);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	//pcl::PointIndices::Ptr inliers;
    // TODO:: Fill in this function to find inliers for the cloud.
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (maxIterations);
    seg.setDistanceThreshold (distanceThreshold);

    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
      //break;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane_hyw(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{

	std::unordered_set<int> inliersResult;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	
    srand(time(NULL));
	while(maxIterations--)
	{
		pcl::PointXYZ point1;
		pcl::PointXYZ point2;
		pcl::PointXYZ point3;
		std::unordered_set<int> temp;

// to make unique point
        std::unordered_set<int> indices;
        while (indices.size() < 3) {
        indices.insert(rand() % cloud->points.size());
        }
        //Then to get the three unique indices:

        auto itr = indices.begin();
        const int ind1 = *itr++;
        const int ind2 = *itr++;
        const int ind3 = *itr;

        point1 = cloud->points[ind1];
		point2 = cloud->points[ind2];
		point3 = cloud->points[ind3];

/*
		point1 = cloud->points[rand()%cloud->points.size()];
		point2 = cloud->points[rand()%cloud->points.size()];
		point3 = cloud->points[rand()%cloud->points.size()];
*/


		float A = (point2.y - point1.y)* (point3.z - point1.z) - (point2.z - point1.z)*(point3.y - point1.y);
		float B = (point2.z - point1.z)* (point3.x - point1.x) - (point2.x - point1.x)*(point3.z - point1.z);
		float C = (point2.x - point1.x)* (point3.y - point1.y) - (point2.y - point1.y)*(point3.x - point1.x);
		float D = - ( A * point1.x + B * point1.y + C * point1.z);

		for(int i = 0; (cloud->points.size() - i) >0 ; i++)
		{
			float distance = abs(A*cloud->points[i].x + B*cloud->points[i].y + C*cloud->points[i].z + D )  / sqrt(A*A + B*B + C*C) ;
			
			if (distance < distanceThreshold)
			{
				temp.insert(i);
			}
		}


		if(temp.size() > inliersResult.size())
		{
			inliersResult = temp;
		}
    }   

    for (int idx : inliersResult)
    {
        inliers->indices.push_back(idx);
    }
    	
	
	
	std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;

}
 


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (clusterTolerance); // 2cm
    ec.setMinClusterSize (minSize);
    ec.setMaxClusterSize (maxSize);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);

    
    

    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    {
        //pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud = new (pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud (new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto& idx : it->indices)
        {
            cluster_cloud->push_back((*cloud)[idx]);
        }

        cluster_cloud->width = cluster_cloud->size ();
        cluster_cloud->height = 1;
        cluster_cloud->is_dense = true;

        std::cout << "PointCloud representing the Cluster: " << cluster_cloud->size () << " data points." << std::endl;
 
        clusters.push_back(cluster_cloud);
    }


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering_hyw(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
    std::vector<std::vector<float>> points;
    KdTree* tree = new KdTree;
            
		
    for (int i = 0; i < cloud->points.size(); i++)
    {
        std::vector<float> point = {cloud->points[i].x, cloud->points[i].y, cloud->points[i].z};
        points.push_back (point);
        tree->insert(point, i);
    }

    // make clusters
    std::vector<std::vector<int>> clusters_indices = euclideanCluster_hyw(points, tree, clusterTolerance, minSize, maxSize);
    
    for (std::vector<int> cluster_indices : clusters_indices)
    {
        typename pcl::PointCloud<PointT>::Ptr cluster_cloud (new pcl::PointCloud<PointT>);
        for (int idx : cluster_indices)
        {
            cluster_cloud->push_back((*cloud)[idx]);
        }
        cluster_cloud->width = cluster_cloud->size ();
        cluster_cloud->height = 1;
        cluster_cloud->is_dense = true;

        clusters.push_back (cluster_cloud);
    }

    
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}





template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}

template<typename PointT>
BoxQ ProcessPointClouds<PointT>::BoundingBoxQ(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    
    // Compute principal directions
    Eigen::Vector4f pcaCentroid;
    pcl::compute3DCentroid(*cluster, pcaCentroid);
    Eigen::Matrix3f covariance;
    computeCovarianceMatrixNormalized(*cluster, pcaCentroid, covariance);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
    eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));  
    /// This line is necessary for proper orientation in some cases. The numbers come out the same without it, but
    ///    the signs are different and the box doesn't get correctly oriented in some cases.
    /* // Note that getting the eigenvectors can also be obtained via the PCL PCA interface with something like:
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPCAprojection (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PCA<pcl::PointXYZ> pca;
    pca.setInputCloud(cloudSegmented);
    pca.project(*cloudSegmented, *cloudPCAprojection);
    std::cerr << std::endl << "EigenVectors: " << pca.getEigenVectors() << std::endl;
    std::cerr << std::endl << "EigenValues: " << pca.getEigenValues() << std::endl;
    // In this case, pca.getEigenVectors() gives similar eigenVectors to eigenVectorsPCA.
    */
   // Transform the original cloud to the origin where the principal components correspond to the axes.
    Eigen::Matrix4f projectionTransform(Eigen::Matrix4f::Identity());
    projectionTransform.block<3,3>(0,0) = eigenVectorsPCA.transpose();
    projectionTransform.block<3,1>(0,3) = -1.f * (projectionTransform.block<3,3>(0,0) * pcaCentroid.head<3>());
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudPointsProjected (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*cluster, *cloudPointsProjected, projectionTransform);
    // Get the minimum and maximum points of the transformed cloud.
    pcl::PointXYZ minPoint, maxPoint;
    pcl::getMinMax3D(*cloudPointsProjected, minPoint, maxPoint);
    const Eigen::Vector3f meanDiagonal = 0.5f*(maxPoint.getVector3fMap() + minPoint.getVector3fMap());

    const Eigen::Quaternionf bboxQuaternion(eigenVectorsPCA); //Quaternions are a way to do rotations https://www.youtube.com/watch?v=mHVwd8gYLnI
    const Eigen::Vector3f bboxTransform = eigenVectorsPCA * meanDiagonal + pcaCentroid.head<3>();


    BoxQ box;
    box.bboxQuaternion = bboxQuaternion;
    box.bboxTransform = bboxTransform;
    box.cube_width = abs(maxPoint.x - minPoint.x);
    box.cube_length = abs(maxPoint.y - minPoint.y);
    box.cube_height = abs(maxPoint.z - minPoint.z);

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}


/*
void Proximity(int index, std::vector<int> &cluster, std::vector<bool> &processed, const std::vector<std::vector<float>> points, KdTree* tree, float distanceTol)
{
	processed[index] = true;
	cluster.push_back(index);
	std::vector<int> nearby = tree->search(points[index],distanceTol);
	for(int j : nearby)
    	{
			if(processed[j] != true)
				Proximity(j, cluster, processed, points, tree, distanceTol);
			
		}

}

std::vector<std::vector<int>> euclideanCluster_hyw(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol, int minSize, int maxSize)
{

	// TODO: Fill out this function to return list of indices for each cluster


	std::vector<std::vector<int>> clusters; // list of clusters
	std::vector<bool> processed(points.size(), false);
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
	
	for (int i = 0 ; i < points.size(); i++) 
	{
		if (processed[i] == false) // iterate through each point
		{
			std::vector<int> cluster;
			Proximity(i, cluster, processed, points, tree, distanceTol);
			 if (minSize <= cluster.size() && cluster.size() <= maxSize) // Check cluster size
            {
                clusters.push_back (cluster); // cluster add clusters
            }
		}

	}

	return clusters;

}

*/