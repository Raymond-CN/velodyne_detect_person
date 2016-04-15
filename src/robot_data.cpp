#include "ros/ros.h"
#include <Ice/Ice.h>
#include "../include/AriaMapInformation.h"
#include "geometry_msgs/PointStamped.h"
#include <tf/transform_broadcaster.h>

using namespace RoboCompAriaMapInformation;
using namespace std;

geometry_msgs::PointStamped robotPose;


class AriaMapInformationI : public AriaMapInformation {
public:
	virtual void robotInformation(const mapPose& m, const Ice::Current&);
	virtual void mapInformation(const MapInfo& mi, const Ice::Current&);
	virtual void goalInformation(const GoalInfo& g, const Ice::Current&);
	tf::TransformBroadcaster broadcaster;
};

void AriaMapInformationI::robotInformation(const mapPose& m, const Ice::Current&)
{
    robotPose.point.x = m.X / 1000.0;
    robotPose.point.y = m.Y / 1000.0;
    cout << "Robot: " << robotPose.point.x << ", " << robotPose.point.y << endl;
    cout << "Robot in ARIA coords: " << m.X << ", " << m.Y << endl;
    
    //TODO: Not tracking rotation right now. Change idsl definition to include Th field
    
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.83, -4.75, 0.0)),
        ros::Time::now(),"world", "base"));
    
}

void AriaMapInformationI::mapInformation(const MapInfo& mi, const Ice::Current&)
{
    cout << " " << endl;
}

void AriaMapInformationI::goalInformation(const GoalInfo& g, const Ice::Current&)
{
    cout << " " << endl;
}


int main(int argc, char** argv){
  ros::init(argc, argv, "robot_data");
  ros::NodeHandle n;
  Ice::CommunicatorPtr ic;
  ros::Rate r(100);
  ros::Publisher pub = n.advertise<geometry_msgs::PointStamped> ("robot_position", 1);
  robotPose.point.x = 0.0;
  robotPose.point.y = 0.0;
  
	ic = Ice::initialize(argc, argv);
	try {      
      Ice::ObjectAdapterPtr adapter =
          ic->createObjectAdapterWithEndpoints("ariamapinformation", "tcp -p 19876");
      Ice::ObjectPtr object = new AriaMapInformationI;
      Ice::ObjectPrx mapinformation = adapter->addWithUUID(object)->ice_oneway();
      adapter->add(object, ic->stringToIdentity("AriaMapInformation.Endpoints"));
      adapter->activate();
    } catch (const Ice::Exception& e) {
      cerr << e << endl;
    } catch (const char* msg) {
      cerr << msg << endl;
    }

  while(n.ok()){   
  	robotPose.header.frame_id = "/world";
		robotPose.header.stamp = ros::Time();
		pub.publish(robotPose);
  	
  	r.sleep();
  }
}
