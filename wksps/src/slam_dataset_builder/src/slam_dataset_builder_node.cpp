#include <cstdio>
#include <tf/transform_listener.h>
#include <ros/ros.h>

#define _USE_MATH_DEFINES

struct EchoListenerData
{
  tf::Vector3 position;
  ros::Time stamp;
};

std::ostream& operator<<(std::ostream& out, const EchoListenerData& data)
{
  out << "[" <<
    data.position.getX() << " " <<
    data.position.getY() << " " << 
    data.stamp.toSec() << "]";
  return out;
}

class EchoListener
{
public:  
  EchoListener(const std::string& sourceFrame, const std::string& targetFrame) :
     sourceFrame_(sourceFrame), targetFrame_(targetFrame) 
  {
    tf_.waitForTransform(sourceFrame_, targetFrame_, 
                         ros::Time(), ros::Duration(1.0));
  };
  ~EchoListener() {};

  EchoListenerData GetPosition() 
  {
    EchoListenerData data;

    tf::StampedTransform echoTransform;
    tf_.lookupTransform(sourceFrame_, targetFrame_, ros::Time(), echoTransform);
    data.position = echoTransform.getOrigin();
    data.stamp = echoTransform.stamp_;
    return data;
  }

private:
  tf::TransformListener tf_;

  std::string sourceFrame_;
  std::string targetFrame_;
};

void printEntry(const EchoListenerData& v1, const EchoListenerData& v2)
{
  std::cout.precision(3);
  std::cout.setf(std::ios::fixed, std::ios::floatfield);
  std::cout << v1 << " " << v2 << std::endl;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "slam_dataset_builder");
  
  // it is better to get arguments, but now it is value by default
  ros::NodeHandle nh;
  const double rate_hz = 10.0; // todo get from argv
  ros::Rate rate(rate_hz);
  
  const std::string slamSourceFrame = "/map"; // todo get from argv
  const std::string slamTargetFrame = "/base_link"; // todo get from argv
  const std::string beaconSourceFrame = "/beacon_base"; // todo get from argv
  const std::string beaconTargetFrame = "/beacon"; // todo get from argv

  EchoListener echoSlamListener(slamSourceFrame, slamTargetFrame);
  EchoListener echoBeaconListener(beaconSourceFrame, beaconTargetFrame);

  while(nh.ok())
  {
    try
    {
      EchoListenerData slamData = echoSlamListener.GetPosition();
      EchoListenerData beaconData = echoBeaconListener.GetPosition();
      printEntry(slamData, beaconData);
    }
    catch (const tf::TransformException& ex)
    {
      std::cerr << "Failure at " << ros::Time::now() << "\n";
      std::cerr << "Exception thrown:" << ex.what() << "\n" << std::flush; 
    }
    rate.sleep();
  } 


  return 0;
}
