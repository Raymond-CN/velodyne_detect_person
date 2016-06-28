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
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/features/normal_3d.h>
#include <sstream>
#include <string>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;


class ExtractClusters
{
  protected:
    ros::NodeHandle n;
		ros::Time begin;
		int n_published_msgs;
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
	  
  	ROS_INFO("Points before radius outlier removal: %d ", inputPclCloud->width);
  	pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
    outrem.setInputCloud(inputPclCloud);
    outrem.setRadiusSearch(0.8);
    outrem.setMinNeighborsInRadius (2);
    outrem.filter (*inputPclCloud);
  	ROS_INFO("Points after radius outlier removal: %d ", inputPclCloud->width);
  	    
	  //KdTree object for the clustering search method 
	  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	  tree->setInputCloud (inputPclCloud);
	  
	  //Perform clustering
	  ros::Time begin_clustering = ros::Time::now ();
	  //Object for storing the normals.
		pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
			
		//Perform statistical outlier removal
		pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
		sor.setInputCloud (inputPclCloud);
		sor.setMeanK (50);
		sor.setStddevMulThresh (1.0);
		sor.filter (*inputPclCloud);
		ROS_INFO("Points after statistical outlier removal: %d \n", inputPclCloud->width);
		
		//Estimate the normals.		
		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
		normalEstimation.setInputCloud(inputPclCloud);
		normalEstimation.setRadiusSearch(0.03);
		normalEstimation.setSearchMethod(tree);
		normalEstimation.compute(*normals);
	 
		// Region growing clustering object.
		pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> clustering;
		clustering.setMinClusterSize(10);
		clustering.setMaxClusterSize(10000);
		clustering.setSearchMethod(tree);
		clustering.setNumberOfNeighbours(30);
		clustering.setInputCloud(inputPclCloud);
		clustering.setInputNormals(normals);
		// Set the angle in radians that will be the smoothness threshold
		// (the maximum allowable deviation of the normals).
		clustering.setSmoothnessThreshold(7.0 / 180.0 * M_PI); // 7 degrees.
		// Set the curvature threshold. The disparity between curvatures will be
		// tested after the normal deviation check has passed.
		clustering.setCurvatureThreshold(1.0);
	 
		std::vector<pcl::PointIndices> cluster_indices;
		clustering.extract(cluster_indices);

		double clustering_time = (ros::Time::now () - begin_clustering).toSec ();
		//ROS_INFO ("%f secs for clustering (%d clusters).", clustering_time, (int) cluster_indices.size ());
	  
	  
	  /*Extract each cluster and store them in:
	  		- clusterPointClouds: pointClouds vector. Each element contains a cluster. Not viewable
	  		- clustersCloud: pointcloud containing every cluster. Viewable in rviz
	  */
	  std::vector<pcl::PointIndices>::const_iterator it;
	  velodyne_detect_person::pointCloudVector clusterPointClouds;
	  
	  int index = 0;
	  //Creates folder to store every cluster in one image
	  double clusterTime = (ros::Time::now()-begin).toSec();
	  std::stringstream convertclusterTime;
	  convertclusterTime << clusterTime;
	  //TODO: Erase and create pruebas directory to remove old stored clusters. The command below only creates clusterX directory inside pruebas
	  std::string foldername = "pruebas/cluster" + convertclusterTime.str();
	  mkdir(foldername.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
	  
	  //For every cluster, store it into file and publisher structure
	  for(it = cluster_indices.begin(); it != cluster_indices.end(); ++it) {
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);			
		  pcl::copyPointCloud (*inputPclCloud, it->indices, *cloud_cluster);
		  std::stringstream convertIndex;		  
		  convertIndex << index;
		  std::string filename = "pruebas/cluster" + convertclusterTime.str() + "/" + convertIndex.str() + ".pcd"; 
		  pcl::io::savePCDFileASCII (filename, *cloud_cluster);
			pcl::toROSMsg (*cloud_cluster , *auxiliarCluster);
			auxiliarCluster->header.frame_id = "/velodyne";
			auxiliarCluster->header.stamp = ros::Time::now();
			clusterPointClouds.pointCloudVector.push_back(*auxiliarCluster);
			*clustersCloud += *cloud_cluster;
			index++;
	  }
	  
	 
	  pcl::toROSMsg (*clustersCloud , *clustersCloudRos);
	  clustersCloudRos->header.frame_id = "/velodyne";
	  clustersCloudRos->header.stamp = ros::Time::now();
	  
	  pub.publish (clusterPointClouds);
	  pub2.publish (*clustersCloudRos);
		n_published_msgs++;
		double elapsed_time = (ros::Time::now () - begin).toSec ();
		//ROS_INFO("Cluster publish freq: %f msgs/s - %d msgs in %f secs.", (float) n_published_msgs / elapsed_time, n_published_msgs, elapsed_time);

	}
	
	ExtractClusters()
    {
      sub = n.subscribe("/velodyne_people_detection/background_extraction/cloud", 1, &ExtractClusters::extractClustersCallback, this);
			begin = ros::Time::now();
			n_published_msgs = 0;
    }
};
	

int main(int argc, char **argv)
{
  ros::init(argc, argv, "extract_clusters");
  ExtractClusters exC;

  ros::spin();

  return 0;
}
