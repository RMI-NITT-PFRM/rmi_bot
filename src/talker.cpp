#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

//Simple messaging over the ROS system.

int main(int argc, char** argv)
{
  /* The ros::init() function needs to see the argc and argv so that it can
     perform any ROS arguments and name remapping that are provided at the
     command line.

     Forprogammatic remapping I can use a different version of init() which
     takes remappings directly but for most command line program, passing
     argc and argv is the easiest way to do it. The third argument to init()
     is the name of the node. 

     I should call one of the variations of ros::init() before using any other
     part of the ROS system.
  */

  ros::init(argc,argv,"talker");

  /* Node handle is the main access point to communiations with the ROS system.
     The first NodeHandle construct will fully initialize this node and the 
     NodeHandle destructor will close down the node.
  */

  ros::NodeHandle n;
  
  /* The advertise() function is how I tell ROS that I want to publish on a
     given topic name. This invokes a call to the ROS master node which keeps
     a registry  of who is publishing anf who is subscribing. After this
     advertise() call is made, the master node will notify anyone who is trying
     to subscribe to this topic name and they will in turn negotiate a peer to
     peer connection with this node. advertise() returns a publisher object
     which allows you to publish messages on that topic through a call to
     publish(). Once all copies of the returned Publisher object are destroyed
     the topic will automatically be unadvertised.

     The second parameter to the advertise(0 function is the size of the
     message queue used for publishing messages. If messages are published more
     than we can send them, the number here specifies how many messages to
     buffer up before throwing some away.
  */

  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  
  ros::Rate loop_rate(10);

  /* A count of how many messages we have sent. This is used to create a unique
     string for each message.
  */

  int count =0;

  while(ros::ok())
    {
      //This is a message object. You stuff it with date, and then publish
      //it to a topic.
      std_msgs::String msg;

      std::stringstream ss;
      ss<<"Hello World"<<count;
      msg.data = ss.str();
      ROS_INFO("%s", msg.data.c_str());
      
      /* The publish() function is how I send messages. The parameter is the
	 message object. The type of this object must agree with the type given
	 as a template parameter to the advertise<>() call, as was done in the
	 constructor above.
      */

      chatter_pub.publish(msg);
      ros::spinOnce();

      loop_rate.sleep();
      ++count;
    }

  return 0;
}
