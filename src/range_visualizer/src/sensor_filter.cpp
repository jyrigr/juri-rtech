#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Range.h"

#include <sstream>
#include <cstdlib>
#include <math.h>

void sensorFilter(const sensor_msgs::Range::ConstPtr& msg) 
{
	std_msgs::Header header;
  header.seq  = msg->header.seq;
  header.frame_id = "sensor_filter";

	sensor_msgs::Range rmsg;
    
  rmsg.header = header;
  rmsg.radiation_type = sensor_msgs::Range::ULTRASOUND;
    
	rmsg.field_of_view = msg->field_of_view;
  rmsg.min_range = msg->min_range;
  rmsg.max_range = msg->max_range;

	rmsg.range = 1;

	pub.publish(rmsg);
}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "random_range");
  ros::NodeHandle n;

  ros::Publisher pub = n.advertise<sensor_msgs::Range>("distance/filter", 1000);
	ros::Subscriber sub = n.subscribe ("distance/raw", 1000, sensorFilter);

	ros::spin();

  return 0;
}
