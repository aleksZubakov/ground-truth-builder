#include "ros/ros.h"
#include "geometry_msgs/Point.h"

#include "hedgehog_proxy.h"

//bool terminateProgram=false;
//void CtrlHandler(int signum)
//{
//	terminateProgram=true;
//}


//
//geometry_msgs::Point simulate_beacons_data() {
//	static geometry_msgs::Point::_x_type x;
//	x += 0.5;
//
//	geometry_msgs::Point pos;
//	pos.x = x;
//	pos.y = std::abs(2 * sin (x));
//	pos.z = std::abs(7 * sin (x));
//
//	return pos;
//}
//

int main()
{
	ros::init(argc, argv, "beacon_data_publisher");

	HedgehogProxy hedge_proxy;

//	signal (SIGINT, CtrlHandler);
//	signal (SIGQUIT, CtrlHandler);

	ros::NodeHandle n;
	ros::Publisher pub = n.advertise<geometry_msgs::Point>("/beacon_data", 100);

	ros::Rate loop_rate(0.1);

	while ((!hedge_proxy.terminationRequired()) && ros::ok())
	{
		PositionValue pos = hedge_proxy.get_data();
		if (pos.ready)
		{
//			std::cout << "X: " << pos.x << " Y: " << pos.y << " Z: " << pos.z << std::endl;
			ROS_INFO("X: %f  Y: %f  Z: %f \n", pos.x, pos.y, pos.z);
			geometry_msgs::Point point_to_send;

			point_to_send.x = pos.x;
			point_to_send.y = pos.y;
			point_to_send.z = pos.z;

			pub.publish(pos);

			ros::spinOnce();
			loop_rate.sleep();
		}

	}
	return 0;
}

