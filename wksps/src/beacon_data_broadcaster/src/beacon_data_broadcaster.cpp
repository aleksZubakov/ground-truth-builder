#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_broadcaster.h>

#include "hedgehog_proxy.h"

// DEBUG
// geometry_msgs::Point simulate_beacons_data() {
// 	static geometry_msgs::Point::_x_type x;
// 	x += 0.5;

// 	geometry_msgs::Point pos;
// 	pos.x = x;
// 	pos.y = std::abs(2 * sin (x));
// 	pos.z = std::abs(7 * sin (x));

// 	return pos;
// }

bool terminateProgram=false;
void CtrlHandler(int signum)
{
	terminateProgram=true;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "beacon_data_broadcaster");

	HedgehogProxy hedge_proxy;

	signal (SIGINT, CtrlHandler);
	signal (SIGQUIT, CtrlHandler);
    

    tf::TransformBroadcaster br;

    tf::Transform transform;
    tf::Quaternion q;

    ros::Rate loop_rate(0.1);
	while ((!hedge_proxy.terminationRequired()) && ros::ok())
    // DEBUG
    // while (ros::ok())
	{
		PositionValue pos = hedge_proxy.get_data();

        // DEBUG
        // geometry_msgs::Point pos = simulate_beacons_data();
        
        if (pos.ready)
		{
            // DEBUG
			std::cout << "X: " << pos.x << " Y: " << pos.y << " Z: " << pos.z << std::endl;
            
            transform.setOrigin(tf::Vector3(pos.x, pos.y, pos.z));

            q.setRPY(0, 0, 0);
            transform.setRotation(q);

            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "beacon", "beacon_listener"));

			ros::spinOnce();
			loop_rate.sleep();
		}

	}
	return 0;
}

