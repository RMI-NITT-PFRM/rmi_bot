/*  ------ver 1.0 by Ashwin Narayan@rmi------ :)
    This is code intended for the arduino that controls
    the wheels of the rmi_bot.
    
    It publishes rotary encoder data from the right and left
    wheels to the rwheel and lwheel topics.
    
    It contains some simple code to test if the publishing
    is working.
    
    To test this code you need to have rosserial_arduino
    installed.
    
    After uploading this code to the Arduino run 
    rosrun rosserial_python serial_node.py /dev/ttyACM0
    to connect to the arduino.
*/

#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>

ros::NodeHandle nh;

std_msgs::Int16 leftEncoder;
std_msgs::Int16 rightEncoder;
ros::Publisher lwheel("lwheel", &leftEncoder);
ros::Publisher rwheel("rwheel", &rightEncoder);

int i = 10030;

void setup()
{
  nh.initNode();
  nh.advertise(lwheel);
  nh.advertise(rwheel);
}

void loop()
{
  leftEncoder.data = i;
  rightEncoder.data = i;
  lwheel.publish(&leftEncoder);
  rwheel.publish(&rightEncoder);
  nh.spinOnce();
  delay(1000);
  i++;
}
