#include "ros/ros.h"
#include "std_msgs/String.h"

/* This tutorial demonstrates a simple receipt of messages over the ROS system.
 */

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{
  /* The ros::init() function as usual sees the argc and argv so that it can 
     perform any ROS arguments and name remapping that are provided at runtime.
     For porgrammatic remappings I can use a different version of the init()
     which takes remapping directly but for most command line programs, passing
     argc and argv is the easiest way to do it. The third argument to init() is
     the name of the node. I must call one of the versions of ros::init() before
     using any other part of the ROS system.
  */
  ros::init(argc, argv, "listener");
  
  /* NodeHandle is the main access point to communications with the ROS system.
     The first NodeHandle constructed will fully initialize this node, and the 
     last NodeHandle destructed will fully close down this node.
  */
  ros::NodeHandle n;
  
  /* The subscribe() call is how I tell ROS that I want to recieve messages on
     a given topic. This invokes a call to the ROS master node which keeps a 
     registry of who is publishing and who is subscribing. Messages are passed
     to a callback function, here called chatterCallback. The sunscribe() call
     returns a Subscriber object that I must hold on to until I want to 
     unsubscribe. When all copies of the subscriber object go out of scorpe, 
     this callback will automatically be unsubscribed from this topic.
     
     The second parameter to the subscribe() function is the size of the message
     queue. If messages are arriving faster than they are being processed, this
     is the number of messages that will be buffered up before the buffer starts
     throwing away the oldest ones.
  */

  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);

  /* ros::spin(0 will enter a loop, pumping callbacks. With this version, all
     callbacks will be called from within this thread (the main thread).
     ros::spin() will exit when C-c is pressed or when the node is shut down by
     the master.
  */
  ros::spin();

  return 0;
}
