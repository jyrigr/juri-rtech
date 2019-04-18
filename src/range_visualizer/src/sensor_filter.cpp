#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Range.h"

#include <sstream>
#include <vector>
#include <cstdlib>
#include <math.h>

class SensorFilter
{
public:
  SensorFilter()
  {
    //Topic you want to publish
    pub_ = n_.advertise<sensor_msgs::Range>("/distance/filter", 1000);

    //Topic you want to subscribe
    sub_ = n_.subscribe("/distance/raw", 1000, &SensorFilter::callback, this);
    
    //Initialize vector
    range_values = std::vector<float>(100);
    
  }

  void callback(const sensor_msgs::Range::ConstPtr& input)
  {
    std_msgs::Header header;
    header.seq  = input->header.seq;
    header.frame_id = "sensor_filter";

    sensor_msgs::Range rmsg;
        
    rmsg.header = header;
    rmsg.radiation_type = sensor_msgs::Range::ULTRASOUND;
    
    range_values.insert(range_values.begin(), input->range);
    range_values.pop_back();
    
    float range_sum = 0;
    for(std::vector<float>::iterator it = range_values.begin(); it != range_values.end(); ++it) {
        range_sum += *it;
    }
    
    rmsg.range = range_sum / range_values.size();
    
    rmsg.field_of_view = input->field_of_view;
    rmsg.min_range = input->min_range;
    rmsg.max_range = input->max_range;

    pub_.publish(rmsg); 
    
    return;
  }
  

private:
  ros::NodeHandle n_; 
  ros::Publisher pub_;
  ros::Subscriber sub_;
  
  std::vector<float> range_values;

};//End of class SensorFilter

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "sensor_filter");

  //Create an object of class SensorFilter that will take care of everything
  SensorFilter SFObject;

  ros::spin();

  return 0;
}
