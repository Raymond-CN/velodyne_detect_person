#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/PointCloud2.h>
#include <vector>
#include "geometry_msgs/Point.h"
#include <boost/shared_ptr.hpp>
#include <tf/transform_listener.h>
#include "geometry_msgs/PointStamped.h"
#include "velodyne_detect_person/personVector.h"
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
using namespace std;
geometry_msgs::PointStamped personCentroid;

class Tracking
{
  protected:
    ros::NodeHandle n;
  public:
    ros::Publisher pub = n.advertise<velodyne_detect_person::personVector> ("person_position", 1); //TODO: Vector de posiciones
    ros::Subscriber sub;
    tf::TransformListener listener;
    	
	void trackingCallback(const velodyne_detect_person::personVector& peoplePosition)
	{
		for(int i = 0; i < peoplePosition.personVector.size(); i++) {
			ROS_INFO ("Cluster %d: %f, %f, %f", i, peoplePosition.personVector[i].point.x, peoplePosition.personVector[i].point.y, peoplePosition.personVector[i].point.z);
		}
	}
	
	Tracking()
    {
      sub = n.subscribe("person_position", 1, &Tracking::trackingCallback, this);
    }
};



int main(int argc, char **argv)
{
  ros::init(argc, argv, "tracking");  
  Tracking tracking;  
  ros::spin();

  return 0;
}

