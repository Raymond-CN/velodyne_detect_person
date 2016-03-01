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
#include "std_msgs/String.h"
#include <boost/shared_ptr.hpp>
#include "velodyne_detect_person/pointCloudVector.h"

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
sensor_msgs::PointCloud2::Ptr backgroundCloud (new sensor_msgs::PointCloud2);
pcl::PointCloud< pcl::PointXYZ > backgroundCloudPCL;
pcl::PointCloud< pcl::PointXYZ > clusterPCL;
int backgroundSize = 0; //Number of points in background cloud

class FindPerson
{
  protected:
    ros::NodeHandle n;
  public:
    ros::Publisher pub = n.advertise<sensor_msgs::PointCloud2> ("person_cloud", 1);
    ros::Subscriber subBackground;
    ros::Subscriber subClusters;
    
    
	//Set background cloud in a global variable. One message every X seconds
	void findPersonBackgroundCallback(const boost::shared_ptr<sensor_msgs::PointCloud2>& inputBackgroundCloud)
	{
		backgroundCloud = inputBackgroundCloud;
		pcl::fromROSMsg(*backgroundCloud,backgroundCloudPCL);
		backgroundSize = backgroundCloud->width;
	}
	
	//Receive data and perform the actual person detection. Lots of messages every second
	void findPersonClustersCallback(const boost::shared_ptr<velodyne_detect_person::pointCloudVector>& clusterVector)
	{
		if(backgroundSize == 0) {
			std::cout << "I need a background cloud!" << std::endl;
		}
		else {
		
		//std::cout << &clusterVector.size() << std::endl;
		//std::cout << sizeof(clusterVector)/sizeof(clusterVector[0]) << std::endl;
		//std::cout << clusterVector.size() << std::endl;
			
			//For each cluster
			for(int i = 0; i < clusterVector->pointCloudVector.size(); i++) {
				int clusterPoints = clusterVector->pointCloudVector[i].width;
				int numCoincidentPoints = 0;
				pcl::fromROSMsg(clusterVector->pointCloudVector[i],clusterPCL);
				
				//Add 1 to counter if a point is in cluster and background
				//TODO: Points are very variable, we should fix a range
				for(int pointCluster = 0; pointCluster < clusterPoints; pointCluster++) {
					for(int pointBackground = 0; pointBackground < backgroundSize; pointBackground++) {
						if(clusterPCL.points[pointCluster].x == backgroundCloudPCL.points[pointBackground].x &&
							clusterPCL.points[pointCluster].y == backgroundCloudPCL.points[pointBackground].y &&
							clusterPCL.points[pointCluster].z == backgroundCloudPCL.points[pointBackground].z)
						{
							std::cout << backgroundCloudPCL.points[pointBackground] << std::endl;
							std::cout << "------------------------------" << std::endl;
							numCoincidentPoints++;
						}
					}				
				}
				
				if(numCoincidentPoints/clusterPoints < 0.2){
					pub.publish (clusterVector->pointCloudVector[i]);
				}
				//std::cout << (numCoincidentPoints/clusterPoints) << std::endl;
				//std::cout << numCoincidentPoints << std::endl;
				//std::cout << clusterPoints << std::endl;
				//std::cout << "------------------------------" << std::endl;
				
				
				//	If counter/numberOfPointsInCluster > X
				//		Not person
				//	If counter/numberOfPointsInCluster < X
				//		Person
			
			}
		
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
