#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/common/boost.h>
#include <vector>
#include <pcl_ros/point_cloud.h>
#include "std_msgs/String.h"
#include <boost/shared_ptr.hpp>
#include <pcl/search/impl/kdtree.hpp>
#include "velodyne_detect_person/pointCloudVector.h"


typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;


class ExtractClusters
{
  protected:
    ros::NodeHandle n;
  public:
    ros::Publisher pub = n.advertise<velodyne_detect_person::pointCloudVector> ("scene_clusters", 1);
    ros::Publisher pub2 = n.advertise<sensor_msgs::PointCloud2> ("clustersCloud", 1);
    ros::Subscriber sub;

	void extractClustersCallback(const boost::shared_ptr<sensor_msgs::PointCloud2>& inputCloud)
	{
	  //Convert ros PointCloud2 to pcl::PointCloud<pcl::pointXYZ>::Ptr
	  pcl::PCLPointCloud2 pcl_pc2;
	  pcl_conversions::toPCL(*inputCloud, pcl_pc2);
	  pcl::PointCloud<pcl::PointXYZ>::Ptr inputPclCloud(new pcl::PointCloud<pcl::PointXYZ>);
	  pcl::fromPCLPointCloud2(pcl_pc2,*inputPclCloud);
	  
	  sensor_msgs::PointCloud2::Ptr clustersCloudRos (new sensor_msgs::PointCloud2);
	  pcl::PointCloud<pcl::PointXYZ>::Ptr clustersCloud (new pcl::PointCloud<pcl::PointXYZ>);
	  sensor_msgs::PointCloud2::Ptr auxiliarCluster (new sensor_msgs::PointCloud2);
	    
	  //KdTree object for the clustering search method 
	  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	  tree->setInputCloud (inputPclCloud);
	  
	  //Perform clustering
	  std::vector<pcl::PointIndices> cluster_indices;
	  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	  ec.setClusterTolerance (0.1);
	  ec.setMinClusterSize (300);
	  ec.setMaxClusterSize (10000);
	  ec.setSearchMethod (tree);
	  ec.setInputCloud (inputPclCloud);
	  ec.extract (cluster_indices);
	  
	  
	  /*Extract each cluster and store them in:
	  		- clusterPointClouds: pointClouds vector. Each element contains a cluster. Not viewable
	  		- clustersCloud: pointcloud containing every cluster. Viewable in rviz
	  */
	  std::vector<pcl::PointIndices>::const_iterator it;
	  std::vector<int>::const_iterator pit;
	  velodyne_detect_person::pointCloudVector clusterPointClouds;
	  
	  for(it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
		for(pit = it->indices.begin(); pit != it->indices.end(); pit++) {
		  cloud_cluster->points.push_back(inputPclCloud->points[*pit]);    
		}
		pcl::toROSMsg (*cloud_cluster , *auxiliarCluster);
		auxiliarCluster->header.frame_id = "/velodyne";
	  	auxiliarCluster->header.stamp = ros::Time::now();
		clusterPointClouds.pointCloudVector.push_back(*auxiliarCluster);
		
		*clustersCloud += *cloud_cluster;
	  }
	 
	  pcl::toROSMsg (*clustersCloud , *clustersCloudRos);
	  clustersCloudRos->header.frame_id = "/velodyne";
	  clustersCloudRos->header.stamp = ros::Time::now();
	  
	  pub.publish (clusterPointClouds);
	  pub2.publish (*clustersCloudRos);
	}
	
	ExtractClusters()
    {
      sub = n.subscribe("velodyne_points", 1, &ExtractClusters::extractClustersCallback, this);
    }
};
	

int main(int argc, char **argv)
{
  ros::init(argc, argv, "extract_clusters");
  ExtractClusters exC;

  ros::spin();

  return 0;
}
