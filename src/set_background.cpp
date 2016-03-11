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
sensor_msgs::PointCloud2::Ptr initialBackground (new sensor_msgs::PointCloud2);
pcl::PCLPointCloud2::Ptr initialBackgroundPCL_pc2 (new pcl::PCLPointCloud2());
pcl::PointCloud<pcl::PointXYZ>::Ptr initialBackgroundPCL(new pcl::PointCloud<pcl::PointXYZ>);


//Create a background grid and a buffer for each grid point
bool backgroundGrid[100][100][30];
bool buffer[100][100][30][8];

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
	  	for(int a = 0; a < 100; a++){
	  		for(int b = 0; b < 100; b++){
	  			for(int c = 0; c < 30; c++){
	  				int bufferCont = 0;
  					if(firstTime){
  						for(int d = 0; d < 7; d++){
  							buffer[a][b][c][d] = false;
  						}
  					}
  					else{ 
  						//Shift buffer values
  						for(int d = 7; d > 0; d--){
								buffer[a][b][c][d] = buffer[a][b][c][d-1];
								if(buffer[a][b][c][d]){
									bufferCont++;
								}
							}
							buffer[a][b][c][0] = false;
							
							//If half or more of the buffer is true, then point is in background							
							//TODO: First cloud is always empty
							//TODO: bufferCount doesn't know the last point value (always set to 0)
  						if(bufferCont > 3){
  							//ENTRA 1200 VECES
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
			//std::cout << outputCloudPCL->width << std::endl;
			//std::cout << publishedCloudPCL.width << std::endl;	  	
	  	
	  	//For each point actually seen in voxelGrid and inside defined range (10m x 10m x 3m)
			for(int pointBackground = 0; pointBackground < background->width; pointBackground++){
				//ENTRA 14000 VECES
				if (-5 <= auxCloud.points[pointBackground].x &&
     			auxCloud.points[pointBackground].x < 5 &&
	     		-5 <= auxCloud.points[pointBackground].y &&
	     		auxCloud.points[pointBackground].y < 5 &&
	     		-1.5 <= auxCloud.points[pointBackground].z &&
	     		auxCloud.points[pointBackground].z < 1.5)
     		{
     			//ENTRA 14000 VECES					
					//If first input cloud, set entire buffer to occupied
					if(firstTime){
						//ENTRA 14000 VECES			
						for(int i = 0; i < 8; i++){
							buffer[int((auxCloud.points[pointBackground].x)*10+50)]
							[int((auxCloud.points[pointBackground].y)*10+50)]
							[int((auxCloud.points[pointBackground].z)*10+15)][i] = true;
							//PROBLEMA: probablemente muchas de las 14000 veces que entra, (x,y,z) sean iguales
						}
					}
					//If not first input cloud, set first buffer value to occupied and shift old values
					else{
						//ENTRA 14000 VECES			
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
