

#include <ros.h>
#include <std_msgs/Int16.h>

ros::NodeHandle  nh;

std_msgs::Int16 int_msg;
ros::Publisher chatter("chatter", &int_msg);

uint16_t dummy=180;

void setup()
{
  nh.initNode();
  nh.advertise(chatter);
}

void loop()
{
  int_msg.data = dummy;
  dummy=dummy*-1;
  chatter.publish( &int_msg );
  nh.spinOnce();
  delay(1000);
}
