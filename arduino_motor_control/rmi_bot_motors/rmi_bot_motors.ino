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
	
	*Update*
	1. Code has been updated with interrupts to give the encoder reading.
	2. Does not respond to power levels on rmotor and lmotor topics yet.
	   but the code does contain functions for making the robot move back
	   forward, left and right.
*/

#include <ros.h>
#include <std_msgs/Int16.h>

ros::NodeHandle nh;

std_msgs::Int16 leftEncoder;
std_msgs::Int16 rightEncoder;
ros::Publisher lwheel("lwheel", &leftEncoder);
ros::Publisher rwheel("rwheel", &rightEncoder);

//Lefmomst wire is wire1 (For the motor driver)
//Wire 1 goes to 13
//wire 2 goes to 12
//Wire 3 goes to 11
//Wire 4 goes to 10

void goBack()
{
  digitalWrite(13, HIGH);
  digitalWrite(12, HIGH);
  digitalWrite(11, LOW);
  digitalWrite(10, LOW);
}

void goForward()
{
  digitalWrite(13, HIGH);
  digitalWrite(12, HIGH);
  digitalWrite(11, HIGH);
  digitalWrite(10, HIGH);
}

void turnRight()
{
  digitalWrite(13, HIGH);
  digitalWrite(12, HIGH);
  digitalWrite(11, LOW);
  digitalWrite(10, HIGH);
}

void turnLeft()
{
  digitalWrite(13, HIGH);
  digitalWrite(12, HIGH);
  digitalWrite(11, HIGH);
  digitalWrite(10, LOW);
}


int i = 10030;
int flag = 1;

void lmotorCB(const std_msgs::Int16& lmotorPower)
{
  //leftEncoder = lmotorPower;
}

void rmotorCB(const std_msgs::Int16& rmotorPower)
{
    //rightEncoder = rmotorPower;
}

ros::Subscriber<std_msgs::Int16> lmotor("lmotor", &lmotorCB);
ros::Subscriber<std_msgs::Int16> rmotor("rmotor", &rmotorCB);

void setup()
{
  nh.initNode();
  nh.advertise(lwheel);
  nh.advertise(rwheel);
  nh.subscribe(lmotor);
  nh.subscribe(rmotor);
  pinMode(13, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(10, OUTPUT);
  digitalWrite(13, HIGH);
  digitalWrite(12, HIGH);
  digitalWrite(11, HIGH);
  digitalWrite(10, HIGH);
  attachInterrupt(0,leftEnc, RISING);
  attachInterrupt(1, rightEnc, RISING);
  leftEncoder.data = 0;
  rightEncoder.data = 0;
}

void loop()
{
  lwheel.publish(&leftEncoder);
  rwheel.publish(&rightEncoder);
  nh.spinOnce();
  turnLeft();
  delay(1000);
}

void leftEnc()
{
  leftEncoder.data += 1;
}

void rightEnc()
{
  rightEncoder.data += 1;
}
