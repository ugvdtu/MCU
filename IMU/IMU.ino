

#include <ros.h>
#include <std_msgs/Int16.h>

ros::NodeHandle  nh;

std_msgs::Int16 int_msg;
ros::Publisher chatter("IMU_Output", &int_msg);

int16_t dummy=0;

void setup()
{
  nh.initNode();
  nh.advertise(chatter);
}

void loop()
{
  int_msg.data = dummy;
  dummy=dummy+1;
  chatter.publish( &int_msg );
  nh.spinOnce();
  delay(1000);
}
