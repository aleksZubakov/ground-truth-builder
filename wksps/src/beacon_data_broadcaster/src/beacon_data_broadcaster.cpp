#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_broadcaster.h>

#include "hedgehog_proxy.h"

// DEBUG
geometry_msgs::Point simulate_beacons_data() {
	static geometry_msgs::Point::_x_type x;
	x += 0.01;

	geometry_msgs::Point pos;
	pos.x = x;
	pos.y = std::abs(2 * sin (x));
	pos.z = std::abs(7 * sin (x));

	return pos;
}

// DEBUG
bool DEBUG = true;


bool terminateProgram=false;
void CtrlHandler(int signum)
{
	terminateProgram=true;
}

tf::Transform get_beacon_transform(HedgehogProxy& hedge_proxy) {
	tf::Transform base_coord;
	tf::Quaternion base_q;
	
	while((!terminateProgram) && (!hedge_proxy.terminationRequired())) {

		PositionValue pos = hedge_proxy.get_position();

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

	if (argc < 1 || (argc > 1 && argc != 4) ) {
		printf("Usage: beacon_data_broadcaster_node rate [base.x base.y base.z]");
		return -1;
	}

	HedgehogProxy hedge_proxy;

	signal (SIGINT, CtrlHandler);
	signal (SIGQUIT, CtrlHandler);
    

    tf::TransformBroadcaster br;

	// setting up base coordinate system
	tf::Transform base_coord;
	if (argc == 4) {
		base_coord.setOrigin(tf::Vector3(atof(argv[1]), atof(argv[2]), atof(argv[3])));
		
		tf::Quaternion q;
		q.setRPY(0, 0, 0);
		base_coord.setRotation(q);
	} else {
		tf::Transform base_coord = get_beacon_transform(hedge_proxy);
	}

	if(DEBUG) {
		tf::Vector3 pos = base_coord.getOrigin();
		std::cout << "X: " << pos.x() << " Y: " << pos.y() << " Z: " << pos.z() << std::endl;
	}

    ros::Rate loop_rate(atof(argv[0]));
    while ((!terminateProgram) && (!hedge_proxy.terminationRequired()) && ros::ok())
	{
			br.sendTransform(tf::StampedTransform(base_coord, ros::Time::now(), "world", "beacon_base"));
            
			tf::Transform t = get_beacon_transform(hedge_proxy);
			
			if (DEBUG) {
				tf::Vector3 pos = t.getOrigin();
				std::cout << "X: " << pos.x() << " Y: " << pos.y() << " Z: " << pos.z() << std::endl;
			}
            br.sendTransform(tf::StampedTransform(t, ros::Time::now(), "world", "beacon"));

			ros::spinOnce();
			loop_rate.sleep();

	}
	return 0;
}

