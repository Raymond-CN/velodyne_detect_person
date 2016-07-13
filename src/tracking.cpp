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

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;
using namespace std;
geometry_msgs::PointStamped personCentroid;
PointCloud::Ptr trackedCentroids (new PointCloud);

//TODO: Add uncertainty area

struct person {
	geometry_msgs::PointStamped lastPosition; //Contains prediction after continueTracking() and observed position after update
	geometry_msgs::PointStamped predictedPosition;
	int consecutiveFrames = 0; //Number of consecutive frames in which person has been seen till last frame
	int framesNotSeen = 0; //Number of consecutive frames in which person hasn't been seen since last apparition
	float velX = 0.001; //TODO: Update velocity given position change, old velocity and consecutive frames
	float velY = 0.001; //TODO: Update velocity given position change, old velocity and consecutive frames
	float heading = 0.0;
	bool lost = false;
	bool personSeen = false;
	int r, g, b;
	bool validCandidate = false; //If it appears in more than 10 consecutive frames, it is considered valid (is not a puntual apparition)
	//TODO: Add buffer with last X positions. Use weighted sum giving more importance to recent values
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
    int numPeople = 0;
    
    
  int distance(geometry_msgs::PointStamped personObserved, int realIndex){
  	return sqrt( pow( (personObserved.point.x - personVec[realIndex].predictedPosition.point.x) ,2)
  	 + pow( (personObserved.point.y - personVec[realIndex].predictedPosition.point.y) ,2) );
  } 
  
  //Check if the person seen is already tracked
  //Return the tracked person number, or a new number (numPeople) if it is a new person
  int trackedPerson(geometry_msgs::PointStamped personObserved) 
  {
  	//Check if the person seen corresponds to any tracked person
  	for(int i = 0; i < personVec.size(); i++) {
  		if(!lostPerson(i)){
				if(distance(personObserved, i) < 1.5) {
					return i;
				}
			}
  	}
  	//If the person doesn't have a correspondence, return a new person index and start tracking
  	//Create person and set color for this person
  	person p;
  	srand(time(NULL));
		p.r = rand() % 255;
		p.g = rand() % 255;
		p.b = rand() % 255;
  	personVec.push_back(p);
  	return numPeople;
  }
  
  //Return true when a person is not seen in the last ~5 seconds (~10 frames/second) and stop tracking  
  bool lostPerson(int index) 
  {
  //TODO: REMOVE PERSON
  	if(personVec[index].framesNotSeen >= 50){
  		//ROS_INFO ("Candidate %d is lost", index);
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
				ROS_INFO ("Cluster %d velocity was %f, %f, so predicted position is %f, %f", i, personVec[i].velX, personVec[i].velY, personVec[i].predictedPosition.point.x, personVec[i].predictedPosition.point.y);
			}
		}
  }
    	
	void trackingCallback(const velodyne_detect_person::personVector& peoplePosition)
	{
		int realIndex;	
		float movX, movY;
		std::vector<bool> personSeen(numPeople);
		
		//Predict new position for each tracked person	
		continueTracking();
				
		//Set every person as not seen in this frame		
		//std::fill(personSeen, personSeen+numPeople, false); //¿TODO?: Error: free(): invalid pointer
		for(int i = 0; i < personSeen.size(); i++) {
			personSeen[i] = false;
		}
		//For each person seen
		for(int i = 0; i < peoplePosition.personVector.size(); i++) {
			//Check which candidate corresponds to the person
			realIndex = trackedPerson(peoplePosition.personVector[i]);
			//Update
			personSeen[realIndex] = true;			
			//movX = personVec[realIndex].lastPosition.point.x - peoplePosition.personVector[i].point.x;
			//movY = personVec[realIndex].lastPosition.point.y - peoplePosition.personVector[i].point.y;
			movX = peoplePosition.personVector[i].point.x - personVec[realIndex].lastPosition.point.x;
			movY = peoplePosition.personVector[i].point.y - personVec[realIndex].lastPosition.point.y;
			personVec[realIndex].lastPosition = peoplePosition.personVector[i];
			personVec[realIndex].consecutiveFrames++;
			personVec[realIndex].framesNotSeen = 0;
			//ROS_INFO ("Candidate %d has %d consecutive frames", realIndex, personVec[realIndex].consecutiveFrames);
			if(personVec[realIndex].consecutiveFrames >= 10){
				personVec[realIndex].validCandidate = true;
			}
			
			//If the person seen corresponds to a candidate
			if(realIndex < numPeople){
				//ROS_INFO ("Cluster %d corresponds to candidate %d", i, realIndex);
				//Update velocity   TODO:Check if it is done correctly
				if (personVec[realIndex].consecutiveFrames > 1){
					personVec[realIndex].velX = ( (personVec[realIndex].consecutiveFrames-1) * personVec[realIndex].velX / personVec[realIndex].consecutiveFrames) + (movX / personVec[realIndex].consecutiveFrames);
					personVec[realIndex].velY = ( (personVec[realIndex].consecutiveFrames-1) * personVec[realIndex].velY / personVec[realIndex].consecutiveFrames) + (movY / personVec[realIndex].consecutiveFrames);
				}
				else{
					personVec[realIndex].velX = movX;
					personVec[realIndex].velY = movY;
				}
				//ROS_INFO ("Cluster %d velocity is %f, %f", realIndex, personVec[realIndex].velX, personVec[realIndex].velY);
			}
			else{
				//ROS_INFO ("Cluster %d is a new candidate with index %d", i, realIndex);
				numPeople++;
				personSeen.push_back(true);
			}
		} //End for each person seen
		
		
		//For each candidate (seen or not)
		for(int i = 0; i < personVec.size(); i++) {
			//If person not seen this frame, update position with predicted position
			if(!personSeen[i]) {
				personVec[i].lastPosition.point.x = personVec[i].predictedPosition.point.x;
				personVec[i].lastPosition.point.y = personVec[i].predictedPosition.point.y;
				personVec[i].consecutiveFrames = 0;
				personVec[i].framesNotSeen++;
			}
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

