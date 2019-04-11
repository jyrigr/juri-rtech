#include <ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Range.h>

int echoPin = A4;
int trigPin = A5;
int count = 0;

ros::NodeHandle  nh;

sensor_msgs::Range msg;
ros::Publisher distance("distance/raw", &msg);

void setup()
{
  pinMode(echoPin,INPUT);
  pinMode(trigPin,OUTPUT);
  
  nh.initNode();
  nh.advertise(distance);
}


long getSonarReadingMillimeters()
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration_us = pulseIn(echoPin, HIGH);
  long distance_mm = (duration_us / 58.0) * 10;
  return distance_mm;
}

void loop()
{
  std_msgs::Header header;
  header.seq  = count;
  header.frame_id = "sensor";
  
  msg.header = header;
  msg.radiation_type = sensor_msgs::Range::ULTRASOUND;

  float fov = M_PI/12;
  float min_range = 0.02;
  float max_range = 4.0;
  
  msg.field_of_view = fov;
  msg.min_range = min_range;
  msg.max_range = max_range;
  
  float range = getSonarReadingMillimeters() / 1000.0;
  msg.range = range;
  
  distance.publish( &msg );

  nh.spinOnce();
  delay(20);
  ++count;
}
