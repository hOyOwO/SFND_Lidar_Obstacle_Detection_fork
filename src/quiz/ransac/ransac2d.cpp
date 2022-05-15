/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function
	
	//랜덤하게 두 점을 뽑고, 라인을 그리고, 전체 점간의 길이를 재서 distol이하의 점을을 추출한다
	//개수를 새고
	//이걸 maxiterations 만큼 해서 개수가 더 많으면 새로 저장한다 

	while(maxIterations--)
	{
		pcl::PointXYZ point1;
		pcl::PointXYZ point2;
		std::unordered_set<int> temp;

		point1 = cloud->points[rand()%cloud->points.size()];
		point2 = cloud->points[rand()%cloud->points.size()];


		double A = point1.y - point2.y;
		double B = point2.x - point1.x;
		double C = point1.x * point2.y - point2.x * point1.y;
		

		for(int i = 0; (cloud->points.size() - i) >0 ; i++)
		{
			double distance = abs(A * cloud->points[i].x +B * cloud->points[i].y +C) / sqrt(A*A + B*B);
			
			if (distance < distanceTol)
			{
				temp.insert(i);
			}
		}
		cout << "point1.x = " << point1.x << endl; 
		cout << "temp = "<< temp.size() << endl;
		cout << "inliersResult = "<< inliersResult.size() << endl;


		if(temp.size() > inliersResult.size())
		{
			inliersResult = temp;
		}
		
	} 
	

	// For max iterations 

	// Randomly sample subset and fit line

	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier

	// Return indicies of inliers from fitted line with most inliers
	
	return inliersResult;

}

std::unordered_set<int> Ransac3D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	

	while(maxIterations--)
	{
		pcl::PointXYZ point1;
		pcl::PointXYZ point2;
		pcl::PointXYZ point3;
		std::unordered_set<int> temp;

		point1 = cloud->points[rand()%cloud->points.size()];
		point2 = cloud->points[rand()%cloud->points.size()];
		point3 = cloud->points[rand()%cloud->points.size()];

		float A = (point2.y - point1.y)* (point3.z - point1.z) - (point2.z - point1.z)*(point3.y - point1.y);
		float B = (point2.z - point1.z)* (point3.x - point1.x) - (point2.x - point1.x)*(point3.z - point1.z);
		float C = (point2.x - point1.x)* (point3.y - point1.y) - (point2.y - point1.y)*(point3.x - point1.x);
		float D = - ( A * point1.x + B * point1.y + C * point1.z);

		for(int i = 0; (cloud->points.size() - i) >0 ; i++)
		{
			float distance = abs(A*cloud->points[i].x + B*cloud->points[i].y + C*cloud->points[i].z + D )  / sqrt(A*A + B*B + C*C) ;
			
			if (distance < distanceTol)
			{
				temp.insert(i);
			}
		}
		cout << "point1.x = " << point1.x << endl; 
		cout << "temp = "<< temp.size() << endl;
		cout << "inliersResult = "<< inliersResult.size() << endl;


		if(temp.size() > inliersResult.size())
		{
			inliersResult = temp;
		}
		
	} 
	
	return inliersResult;

}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	//std::unordered_set<int> inliers = Ransac(cloud, 50, 0.5);
	std::unordered_set<int> inliers = Ransac3D(cloud, 50, 0.5);


	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
