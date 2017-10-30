#include "ros/ros.h"
#include "geometry_msgs/Point.h"

#include <sstream>

geometry_msgs::Point simulate_beacons_data() {
	static geometry_msgs::Point::_x_type x;
	x += 0.5;

	geometry_msgs::Point pos;
	pos.x = x;
	pos.y = std::abs(2 * sin (x));
	pos.z = std::abs(7 * sin (x));

	return pos;
}


int main(int argc, char **argv) {
	ros::init(argc, argv, "beacon_data_publisher");

	ros::NodeHandle n;
	ros::Publisher pub = n.advertise<geometry_msgs::Point>("/beacon_data", 100);

	ros::Rate loop_rate(0.1);

	while (ros::ok()) {
		 geometry_msgs::Point pos = simulate_beacons_data();

		 ROS_INFO("%f %f %f", pos.x, pos.y, pos.z);
		 pub.publish(pos);

		 ros::spinOnce();
		 loop_rate.sleep();
	 }
}
