#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_broadcaster.h>

#include "hedgehog_proxy.h"

// DEBUG
// geometry_msgs::Point simulate_beacons_data() {
// 	static geometry_msgs::Point::_x_type x;
// 	x += 0.01;

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

tf::Transform get_beacon_transform(HedgehogProxy& hedge_proxy) {
	tf::Transform base_coord;
	tf::Quaternion base_q;
	
	while((!terminateProgram) && (!hedge_proxy.terminationRequired())) {

		PositionValue pos = hedge_proxy.get_data();

		// DEBUG
		// geometry_msgs::Point pos = simulate_beacons_data();
		if (pos.ready) {
			base_coord.setOrigin(tf::Vector3(pos.x, pos.y, pos.z));
			base_q.setRPY(0, 0, 0);
			base_coord.setRotation(base_q);
			break;
		}
	}

	return base_coord;
}



int main(int argc, char** argv)
{
	ros::init(argc, argv, "beacon_data_broadcaster");

	HedgehogProxy hedge_proxy;

	signal (SIGINT, CtrlHandler);
	signal (SIGQUIT, CtrlHandler);
    

    tf::TransformBroadcaster br;

	tf::Transform base_coord = get_beacon_transform(hedge_proxy);

    ros::Rate loop_rate(0.5);
	// while ((!hedge_proxy.terminationRequired()) && ros::ok())
    // DEBUG
    while ((!terminateProgram) && (!hedge_proxy.terminationRequired()) && ros::ok())
	{
		// PositionValue pos = hedge_proxy.get_data();

        // DEBUG
        // geometry_msgs::Point pos = simulate_beacons_data();


        // if (pos.ready)
		// {
		
			br.sendTransform(tf::StampedTransform(base_coord, ros::Time::now(), "world", "beacon_base"));
            // DEBUG
            
			tf::Transform t = get_beacon_transform(hedge_proxy);
			
			tf::Vector3 pos = t.getOrigin();
			std::cout << "X: " << pos.x() << " Y: " << pos.y() << " Z: " << pos.z() << std::endl;

            // transform.setOrigin(tf::Vector3(pos.x, pos.y, pos.z));

            // q.setRPY(0, 0, 0);
            // transform.setRotation(q);
            br.sendTransform(tf::StampedTransform(t, ros::Time::now(), "beacon_base", "beacon"));

			ros::spinOnce();
			loop_rate.sleep();
		// }

	}
	return 0;
}

