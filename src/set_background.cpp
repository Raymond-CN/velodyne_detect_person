#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <pcl/common/boost.h>
#include <pcl/filters/voxel_grid.h>
#include <vector>
#include <pcl_ros/point_cloud.h>
#include "std_msgs/String.h"
#include <boost/shared_ptr.hpp>
#include <pcl_ros/point_cloud.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
#define GRID_SIZE 0.1;

//Create a pointcloud which holds background points		
//Create a pointcloud which holds background points candidates
sensor_msgs::PointCloud2::Ptr background (new sensor_msgs::PointCloud2);
sensor_msgs::PointCloud2::Ptr backgroundCandidates (new sensor_msgs::PointCloud2);
pcl::PCLPointCloud2::Ptr inputCloudPCL (new pcl::PCLPointCloud2());
pcl::PCLPointCloud2::Ptr outputCloudPCL (new pcl::PCLPointCloud2());
pcl::PointCloud<pcl::PointXYZ> auxCloud;

//Create a background grid and a buffer for each grid point
bool backgroundGrid[100][100][30]; //TODO: backgroundGrid must be filled with falses in the first moment
bool buffer[100][100][30][8]; //TODO: Buffer must be filled with zeros in the first moment

class SetBackground
{
  protected:
    ros::NodeHandle n;
  public:
    ros::Publisher pub = n.advertise<sensor_msgs::PointCloud2> ("scene_background", 1);
    ros::Subscriber sub;
    int cont = 0; //TODO: Cont should be changed after first iteration

	  void setBackgroundCallback(const boost::shared_ptr<sensor_msgs::PointCloud2>& inputCloud)
	  {
	  	//Create voxel grid
	  	pcl_conversions::toPCL(*inputCloud, *inputCloudPCL);
	  	pcl::VoxelGrid<pcl::PCLPointCloud2> grid;
	  	grid.setInputCloud (inputCloudPCL);
	  	grid.setLeafSize (0.1f, 0.1f, 0.1f);
	  	grid.filter (*outputCloudPCL);
	  	pcl::fromPCLPointCloud2(*outputCloudPCL,auxCloud);
	  	pcl::toROSMsg (auxCloud, *background);
	  	
	  	
	  	//For each point in voxelGrid (background) and inside defined range (10m x 10m x 3m)
			for(int pointBackground = 0; pointBackground < background->width; pointBackground++){
				if (-5 <= auxCloud.points[pointBackground].x &&
     			auxCloud.points[pointBackground].x < 5 &&
	     		-5 <= auxCloud.points[pointBackground].y &&
	     		auxCloud.points[pointBackground].y < 5 &&
	     		-1.5 <= auxCloud.points[pointBackground].z &&
	     		auxCloud.points[pointBackground].z < 1.5)
     		{
     			//Set closer grid point to occupied
					backgroundGrid[int((auxCloud.points[pointBackground].x)*10+50)]
					[int((auxCloud.points[pointBackground].y)*10+50)]
					[int((auxCloud.points[pointBackground].z)*10+15)] = true;
					
					//If first input cloud, set entire buffer to occupied
					if(cont == 0){
						for(int i = 0; i < 8; i++){
							buffer[int((auxCloud.points[pointBackground].x)*10+50)]
							[int((auxCloud.points[pointBackground].y)*10+50)]
							[int((auxCloud.points[pointBackground].z)*10+15)][i] = true;
						}
					}
					
					//If not first input cloud, set first buffer value to occupied and shift old values
					else{
						for(int i = 1; i < 8; i++){
							buffer[int((auxCloud.points[pointBackground].x)*10+50)]
							[int((auxCloud.points[pointBackground].y)*10+50)]
							[int((auxCloud.points[pointBackground].z)*10+15)][i] = 
							buffer[int((auxCloud.points[pointBackground].x)*10+50)]
							[int((auxCloud.points[pointBackground].y)*10+50)]
							[int((auxCloud.points[pointBackground].z)*10+15)][i-1];
						}
						buffer[int((auxCloud.points[pointBackground].x)*10+50)]
						[int((auxCloud.points[pointBackground].y)*10+50)]
						[int((auxCloud.points[pointBackground].z)*10+15)][0] = true;	
						
					}
				}
			}		
		
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
		  
		  if(cont == 0){
			  std::cout << "Set background" << std::endl;
			  background = inputCloud;
			  cont++;
		  }
		  */
		  pub.publish (background);
		  std::cout << "Set cloud" << std::endl;

	  }
	
	  SetBackground()
    {
    	sub = n.subscribe("velodyne_points", 1, &SetBackground::setBackgroundCallback, this);
    }
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "set_background");
  std::cout << "Waiting for 10 seconds before taking the first background cloud, then taking a cloud every 5 seconds" << std::endl;
  //sleep(10);
  SetBackground setB;
  ros::Rate loop_rate(0.2);

  while (ros::ok()){
		ros::spinOnce();
		loop_rate.sleep();
	}

  return 0;
}
