#include "ros/ros.h"
#include <Ice/Ice.h>
#include "../include/AriaMapInformation.h"
//#include <tf/transform_listener.h>

using namespace RoboCompAriaMapInformation;
using namespace std;

class AriaMapInformationI : public AriaMapInformation {
public:
	virtual void robotInformation(const mapPose& m, const Ice::Current&);
	virtual void mapInformation(const MapInfo& mi, const Ice::Current&);
	virtual void goalInformation(const GoalInfo& g, const Ice::Current&);
};

void AriaMapInformationI::robotInformation(const mapPose& m, const Ice::Current&)
{
    cout << "Robot: " << m.X << ", " << m.Y << endl;
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
	ic = Ice::initialize(argc, argv);

	try {
      
      Ice::ObjectAdapterPtr adapter =
          ic->createObjectAdapterWithEndpoints("ariamapinformation", "tcp -p 19876");
      Ice::ObjectPtr object = new AriaMapInformationI;
      //i
      Ice::ObjectPrx mapinformation = adapter->addWithUUID(object)->ice_oneway();
      //
      adapter->add(object, ic->stringToIdentity("AriaMapInformation.Endpoints"));
      //i
      cout << "rodaballo" << endl;
      //
      
      adapter->activate();

      //ic->waitForShutdown();
    } catch (const Ice::Exception& e) {
      cerr << e << endl;
    } catch (const char* msg) {
      cerr << msg << endl;
    }

  while(n.ok()){   
  	r.sleep();
  }
}
