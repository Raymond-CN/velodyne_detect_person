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
#include <queue>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>

#define BUFFER_SIZE 30
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
using namespace std;
geometry_msgs::PointStamped personCentroid;
PointCloud::Ptr trackedCentroids (new PointCloud);
std::vector<bool> associatedCluster; //associatedCluster[2] tells if cluster 2 corresponds to any candidate. If true, this cluster must not be asocciated again

//TODO: Add uncertainty area

struct person {
	geometry_msgs::PointStamped lastPosition; //Contains prediction after continueTracking() and observed position after update
	geometry_msgs::PointStamped predictedPosition;
	int consecutiveFrames = 0; //Number of consecutive frames in which person has been seen till last frame
	int framesNotSeen = 0; //Number of consecutive frames in which person hasn't been seen since last apparition
	float velX = 0.001;
	float velY = 0.001;
	float heading = 0.0;
	bool lost = false;
	int r, g, b;
	bool validCandidate = false; //If it appears in more than 10 consecutive frames, it is considered valid (is not a puntual apparition)
	std::queue<float> bufferX;
	std::queue<float> bufferY;
};

class Tracking
{
  protected:
    ros::NodeHandle n;
  public:
    ros::Publisher pub = n.advertise<sensor_msgs::PointCloud2> ("tracking", 1); //One point for each tracked instance
    ros::Subscriber sub;
    tf::TransformListener listener;
    std::vector<person> personVec;
    
  //Associate each candidate with, at most, one cluster. Assume that first candidate picks first, so we remove cluster from list once picked
  std::vector<int> associateCandidates(const velodyne_detect_person::personVector& peoplePosition) {
		std::vector<int> correspondenceList(personVec.size()); //correspondenceList[4] contains the cluster that corresponds to candidate 4, or -1 if any cluster corresponds
		//Set associatedCluster to false initially
		for(int x = 0; x < peoplePosition.personVector.size(); x++) {
			associatedCluster[x] = false;
		}
  	for(int i = 0; i < personVec.size(); i++) {
  		//Initially set correspondenceList[i] as 'no correspondence'
  		correspondenceList[i] = -1;
  		if(!lostPerson(i)){
				float minimum = 9999;
				for(int j = 0; j < peoplePosition.personVector.size(); j++) {
					//If cluster j is not asociated with any candidate yet, is in range and is the closer point to our prediction, set it as the closer point
					if (!associatedCluster[j]) { 
						float dist = distance(peoplePosition.personVector[j], i);
						if(dist < 1.5 && dist < minimum) {
							minimum = dist;
							correspondenceList[i] = j;
						}
						if(correspondenceList[i] != -1) {
							associatedCluster[correspondenceList[i]] = true;
						}
					}
				}
  		}
  	}
  	return correspondenceList;
  }  
    
  //Caluclates new velocity given last BUFFER_SIZE frames  
  float calcVelocity(std::queue<float> buff){
  	std::queue<float> buffCopy = buff;
  	int size = buff.size();
  	float sum = 0; //Sum of indexes (if 30 elements, sum = 465)
  	float velAverage = 0; //Weighted sum of velocities
  	for(int i = 0; i < size; i++){
  		sum += i+1;
  	}
  	int i = 0;
  	while(buffCopy.size() > 0){
  		velAverage += (((i+1) * buffCopy.front()) / sum);
  		buffCopy.pop();
  		i++;
  	}
  	return velAverage;
  }  
    
  //Returns distance between two points
  float distance(geometry_msgs::PointStamped personObserved, int realIndex){
  	return sqrt( pow( (personObserved.point.x - personVec[realIndex].predictedPosition.point.x) ,2)
  	 + pow( (personObserved.point.y - personVec[realIndex].predictedPosition.point.y) ,2) );
  } 
  
  
  //Return true when a person is not seen in the last ~5 seconds (50 frames)
  bool lostPerson(int index) 
  {
  	if(personVec[index].framesNotSeen >= 50){
  		ROS_INFO ("Candidate %d is lost", index);
  		return true;
  	}
  	else return false;
  }
  
  //Continue tracking to predict position until lostPerson()
  void continueTracking()
  {
  	//For each stored person, calculate predicted position given old position and velocity model
		for(int i = 0; i < personVec.size(); i++) {
			if(!lostPerson(i)){			
				personVec[i].predictedPosition.point.x = personVec[i].lastPosition.point.x + personVec[i].velX;
				personVec[i].predictedPosition.point.y = personVec[i].lastPosition.point.y + personVec[i].velY;
			}
		}
  }
    	
	void trackingCallback(const velodyne_detect_person::personVector& peoplePosition)
	{
		float movX, movY;
		associatedCluster.clear();
		associatedCluster.reserve(peoplePosition.personVector.size()); 
		
		//Predict new position for each tracked person	
		continueTracking();
		
		//Associate each candidate with 1 or 0 clusters
		std::vector<int> correspondenceList = associateCandidates(peoplePosition);
		//For each candidate
		for(int i = 0; i < personVec.size(); i++) {
			//Remove oldest element if maximum buffer size is reached
			if(!personVec[i].bufferX.size() == BUFFER_SIZE) {
				personVec[i].bufferX.pop();
			}
			if(!personVec[i].bufferY.size() == BUFFER_SIZE) {
				personVec[i].bufferY.pop();
			}
			//If candidate not seen in this frame, set new position as predictedPosition
			if(correspondenceList[i] == -1) {
				//Calculate predicted movement since last frame
				movX = personVec[i].predictedPosition.point.x - personVec[i].lastPosition.point.x;
				movY = personVec[i].predictedPosition.point.y - personVec[i].lastPosition.point.y;
				personVec[i].bufferX.push(movX);
				personVec[i].bufferY.push(movY);
				personVec[i].lastPosition = personVec[i].predictedPosition;
				personVec[i].consecutiveFrames = 0;
				personVec[i].framesNotSeen++;
			}			
			else {
				ROS_INFO ("I am candidate %i and cluster %i is associated", i, correspondenceList[i]);	
				//Calculate movement since last frame
				movX = peoplePosition.personVector[correspondenceList[i]].point.x - personVec[i].lastPosition.point.x;
				movY = peoplePosition.personVector[correspondenceList[i]].point.y - personVec[i].lastPosition.point.y;
				personVec[i].bufferX.push(movX);
				personVec[i].bufferY.push(movY);
				personVec[i].lastPosition = peoplePosition.personVector[correspondenceList[i]];
				personVec[i].consecutiveFrames++;
				personVec[i].framesNotSeen = 0;
				ROS_INFO ("Consecutive Frames: %i", personVec[i].consecutiveFrames);	
				if(personVec[i].consecutiveFrames >= 10){
					personVec[i].validCandidate = true;	
				}
			}
			//Update velocity
			personVec[i].velX = calcVelocity(personVec[i].bufferX);
			personVec[i].velY = calcVelocity(personVec[i].bufferY);
			//Publish every tracked person centroid
			if(personVec[i].validCandidate){
				pcl::PointXYZRGB point = pcl::PointXYZRGB(personVec[i].r, personVec[i].g, personVec[i].b);
				point.x = personVec[i].lastPosition.point.x;
				point.y = personVec[i].lastPosition.point.y;
				point.z = 0;
				trackedCentroids->points.push_back(point);
			}
		}
		
		trackedCentroids->header.frame_id = "world";
		//trackedCentroids->header.stamp = ros::Time::now();
		pub.publish (trackedCentroids);
			
		//For each person seen, start tracking clusters without correspondence
		for(int i = 0; i < peoplePosition.personVector.size(); i++) {
			if(!associatedCluster[i]){
				person p;
				srand(time(NULL));
				p.r = rand() % 255;
				p.g = rand() % 255;
				p.b = rand() % 255;
				p.lastPosition = peoplePosition.personVector[i];
				p.consecutiveFrames++;
				p.framesNotSeen = 0;				
				personVec.push_back(p);
			}
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

