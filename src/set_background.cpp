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
#define GRID_SIZE 0.1
#define X_SIZE 100
#define Y_SIZE 100
#define Z_SIZE 30
#define BUFFER_SIZE 8

//Create a pointcloud which holds background points		
//Create a pointcloud which holds background points candidates
sensor_msgs::PointCloud2::Ptr background (new sensor_msgs::PointCloud2);
sensor_msgs::PointCloud2::Ptr backgroundCandidates (new sensor_msgs::PointCloud2);
pcl::PCLPointCloud2::Ptr inputCloudPCL (new pcl::PCLPointCloud2());
pcl::PCLPointCloud2::Ptr outputCloudPCL (new pcl::PCLPointCloud2());
pcl::PointCloud<pcl::PointXYZ> auxCloud;
sensor_msgs::PointCloud2::Ptr initialBackground (new sensor_msgs::PointCloud2);
pcl::PCLPointCloud2::Ptr initialBackgroundPCL_pc2 (new pcl::PCLPointCloud2());
pcl::PointCloud<pcl::PointXYZ>::Ptr initialBackgroundPCL(new pcl::PointCloud<pcl::PointXYZ>);


//Create a background grid and a buffer for each grid point
bool backgroundGrid[X_SIZE][Y_SIZE][Z_SIZE];
bool buffer[X_SIZE][Y_SIZE][Z_SIZE][BUFFER_SIZE];

class SetBackground
{
  protected:
    ros::NodeHandle n;
  public:
    ros::Publisher pub = n.advertise<sensor_msgs::PointCloud2> ("scene_background", 1);
    ros::Subscriber sub;
    bool firstTime = true;

	  void setBackgroundCallback(const boost::shared_ptr<sensor_msgs::PointCloud2>& inputCloud)
	  {
	  	sensor_msgs::PointCloud2 publishedCloud;
	  	pcl::PointCloud<pcl::PointXYZ> publishedCloudPCL;
	  
	  	//Create voxel grid
	  	pcl_conversions::toPCL(*inputCloud, *inputCloudPCL);
	  	pcl::VoxelGrid<pcl::PCLPointCloud2> grid;
	  	grid.setInputCloud (inputCloudPCL);
	  	grid.setLeafSize (0.01f, 0.01f, 0.01f);
	  	grid.filter (*outputCloudPCL);
	  	pcl::fromPCLPointCloud2(*outputCloudPCL,auxCloud);
	  	pcl::toROSMsg (auxCloud, *background);
	  	
	  	//Add initial background cloud to prevent shadows
	  	if(firstTime){
	  		initialBackground = background;
	  		pcl_conversions::toPCL(*initialBackground, *initialBackgroundPCL_pc2);
	  		pcl::fromPCLPointCloud2(*initialBackgroundPCL_pc2,*initialBackgroundPCL);
	  	}
  		publishedCloudPCL += *initialBackgroundPCL;
	  	
	  	//For each point in grid, suppose it's not seen and set each value to false
	  	for(int a = 0; a < X_SIZE; a++){
	  		for(int b = 0; b < Y_SIZE; b++){
	  			for(int c = 0; c < Z_SIZE; c++){
	  				int bufferCont = 0;
  					if(firstTime){
  						for(int d = 0; d < BUFFER_SIZE; d++){
  							buffer[a][b][c][d] = false;
  						}
  					}
  					else{ 
  						//Shift buffer values
  						for(int d = BUFFER_SIZE-1; d > 0; d--){
								buffer[a][b][c][d] = buffer[a][b][c][d-1];
								if(buffer[a][b][c][d]){
									bufferCont++;
								}
							}
							buffer[a][b][c][0] = false;
							
							//If half or more of the buffer is true, then point is in background							
							//TODO: bufferCount doesn't know the last point value (always set to 0)
  						if(bufferCont > 3){
  							publishedCloudPCL.insert(
  								publishedCloudPCL.end(), pcl::PointXYZ(
  									(((float)a-50.0)/10.0),
  									(((float)b-50.0)/10.0),
  									(((float)c-15.0)/10.0)
  									)
  							);  							
  						}
						}
  				}
				}
			} 	
	  	
	  	//For each point actually seen in voxelGrid and inside defined range (10m x 10m x 3m)
			for(int pointBackground = 0; pointBackground < background->width; pointBackground++){
				if (-5 <= auxCloud.points[pointBackground].x &&
     			auxCloud.points[pointBackground].x < 5 &&
	     		-5 <= auxCloud.points[pointBackground].y &&
	     		auxCloud.points[pointBackground].y < 5 &&
	     		-1.5 <= auxCloud.points[pointBackground].z &&
	     		auxCloud.points[pointBackground].z < 1.5)
     		{				
					//If first input cloud, set entire buffer to occupied
					if(firstTime){			
						for(int i = 0; i < BUFFER_SIZE; i++){
							buffer[int((auxCloud.points[pointBackground].x)*10+50)]
							[int((auxCloud.points[pointBackground].y)*10+50)]
							[int((auxCloud.points[pointBackground].z)*10+15)][i] = true;
						}
					}
					//If not first input cloud, set first buffer value to occupied and shift old values
					else{		
						buffer[int((auxCloud.points[pointBackground].x)*10+50)]
						[int((auxCloud.points[pointBackground].y)*10+50)]
						[int((auxCloud.points[pointBackground].z)*10+15)][0] = true;	
					}
				}
			}
			firstTime = false;
			
			pcl::toROSMsg (publishedCloudPCL , publishedCloud);
	  	publishedCloud.header.frame_id = "/velodyne";
	  	publishedCloud.header.stamp = ros::Time::now();	

		  pub.publish (publishedCloud);
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
  sleep(10);
  SetBackground setB;
  ros::Rate loop_rate(1);

  while (ros::ok()){
		ros::spinOnce();
		loop_rate.sleep();
	}

  return 0;
}
