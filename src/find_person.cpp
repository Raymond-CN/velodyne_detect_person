#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/common/boost.h>
#include <vector>
#include <pcl_ros/point_cloud.h>
#include "geometry_msgs/Point.h"
#include <boost/shared_ptr.hpp>
#include "velodyne_detect_person/pointCloudVector.h"

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
sensor_msgs::PointCloud2::Ptr backgroundCloud (new sensor_msgs::PointCloud2);
pcl::PointCloud< pcl::PointXYZ > backgroundCloudPCL;
pcl::PointCloud< pcl::PointXYZ > clusterPCL;
int backgroundSize = 0; //Number of points in background cloud
bool backgroundGrid[100][100][30];

class FindPerson
{
  protected:
    ros::NodeHandle n;
  public:
    ros::Publisher pub = n.advertise<sensor_msgs::PointCloud2> ("person_cloud", 1);
    ros::Publisher pub2 = n.advertise<geometry_msgs::Point> ("person_position", 1);
    ros::Subscriber subBackground;
    ros::Subscriber subClusters;
    
    
	//Set background cloud in a global variable. One message every X seconds
	void findPersonBackgroundCallback(const boost::shared_ptr<sensor_msgs::PointCloud2>& inputBackgroundCloud)
	{
		backgroundCloud = inputBackgroundCloud;
		pcl::fromROSMsg(*backgroundCloud,backgroundCloudPCL);
		backgroundSize = backgroundCloud->width;
		
		
		pcl::PointCloud<pcl::PointXYZ>::iterator it;
		for(it = backgroundCloudPCL.points.begin(); it < backgroundCloudPCL.points.end(); it++)
		{
			if (-5 <= it->x && it->x < 5 && -5 <= it->y && it->y < 5 && -1.5 <= it->z && it->z < 1.5)
     	{
				backgroundGrid[int((it->x)*10+50)][int((it->y)*10+50)][int((it->z)*10+15)] = true;
			}
		}	
	}
	
	
	//Receive data and perform the actual person detection. Lots of messages every second
	void findPersonClustersCallback(const boost::shared_ptr<velodyne_detect_person::pointCloudVector>& clusterVector)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr clustersCloud (new pcl::PointCloud<pcl::PointXYZ>); //Contains every person cluster and is visible in rviz
		pcl::PointCloud<pcl::PointXYZ> auxiliarCluster;
		sensor_msgs::PointCloud2::Ptr clustersCloudRos (new sensor_msgs::PointCloud2);
	
	
		if(backgroundSize == 0) {
			std::cout << "I need a background cloud!" << std::endl;
		}
		else {		
			//For each cluster
			for(int i = 0; i < clusterVector->pointCloudVector.size(); i++) {
				int clusterPoints = clusterVector->pointCloudVector[i].width;  //cluster size
				int numCoincidentPoints = 0;
				pcl::fromROSMsg(clusterVector->pointCloudVector[i],clusterPCL);
				
				//Add 1 to counter if a point is in cluster and background
				for(int pointCluster = 0; pointCluster < clusterPoints; pointCluster++) {
					if (-5 <= clusterPCL.points[pointCluster].x &&
				  	clusterPCL.points[pointCluster].x < 5 &&
					  -5 <= clusterPCL.points[pointCluster].y &&
					  clusterPCL.points[pointCluster].y < 5 &&
					  -1.5 <= clusterPCL.points[pointCluster].z &&
					  clusterPCL.points[pointCluster].z < 1.5)
     			{								
						if(backgroundGrid[int((clusterPCL.points[pointCluster].x)*10+50)][int((clusterPCL.points[pointCluster].y)*10+50)][int((clusterPCL.points[pointCluster].z)*10+15)] == true){
							numCoincidentPoints++;
						}	
					}		
				}
				if(float(float(numCoincidentPoints)/float(clusterPoints)) < 0.2){
					pcl::fromROSMsg(clusterVector->pointCloudVector[i],auxiliarCluster);
					*clustersCloud += auxiliarCluster;
				}
			
			}
			pcl::toROSMsg (*clustersCloud , *clustersCloudRos);
	  	clustersCloudRos->header.frame_id = "/velodyne";
	  	clustersCloudRos->header.stamp = ros::Time::now();
			
			pub.publish (clustersCloudRos);
		
		}
	}
	
	FindPerson()
    {
      subBackground = n.subscribe("scene_background", 1, &FindPerson::findPersonBackgroundCallback, this);
      subClusters = n.subscribe("scene_clusters", 1, &FindPerson::findPersonClustersCallback, this);
    }
};



int main(int argc, char **argv)
{
  ros::init(argc, argv, "find_person");
  FindPerson findP;

  ros::spin();

  return 0;
}

