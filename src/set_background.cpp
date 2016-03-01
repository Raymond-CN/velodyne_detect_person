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

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

//Create a pointcloud which holds background points		
//Create a pointcloud which holds background points candidates
sensor_msgs::PointCloud2::Ptr background (new sensor_msgs::PointCloud2);
sensor_msgs::PointCloud2::Ptr backgroundCandidates (new sensor_msgs::PointCloud2);

class SetBackground
{
  protected:
    ros::NodeHandle n;
  public:
    ros::Publisher pub = n.advertise<sensor_msgs::PointCloud2> ("scene_background", 1);
    ros::Subscriber sub;
    int cont = 0;

	void setBackgroundCallback(const boost::shared_ptr<sensor_msgs::PointCloud2>& inputCloud)
	{

		
		
		//Create an initial 8 bits buffer for each point in inputCloud
		//	- For every point, set buffer to [1,1,1,1,1,1,1,1] and add it to background cloud
		//Create a 8 bits buffer associated to each new point seen
		//	- Set buffer to [1,0,0,0,0,0,0,0] and add it to background candidates cloud
		//For each point in the background candidates cloud
		//	- If seen, set first buffer bit to 1
		//	- If not seen, set first buffer bit to 0
		//	- If buffer is [0,0,0,0,0,0,0,0], remove point from candidates cloud
		//	- If buffer has 5 or more ones, move point to background cloud
		//For each point in the background cloud
		//	- If seen, set first buffer bit to 1
		//	- If not seen, set first buffer bit to 0
		//	- If buffer has 4 or less ones, move point to background candidates cloud

		//Publish background candidates cloud (scene_background)
		
		
		/* Initial approach
			- Every point in the initial point cloud is background
			- Background is not updated
		*/
		if(cont == 0){
			std::cout << "Fijo background" << std::endl;
			background = inputCloud;
			cont++;
		}
		pub.publish (background);

	}
	
	SetBackground()
    {
      sub = n.subscribe("velodyne_points", 1, &SetBackground::setBackgroundCallback, this);
    }
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "set_background");
  SetBackground setB;

  ros::spin();

  return 0;
}
