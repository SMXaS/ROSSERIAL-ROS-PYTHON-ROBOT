// Student Arnoldas Jurkus

// Student ID - 1801***

// University - Bedforshire Luton Campus

// Course - AI and Mobile Robots

// This is a ROSSERIAL ROBOT CONTROL PROGRAM

// TO CONTROL THIS:
// Open cmd -> roscore
// Open another cmd -> rosrun rosserial_python serial_node.py /dev/ttyACM0 
// (ACM0 is your USB port), (Will throw error if Bluetooth is connected)
// Open another cmd -> rostopic pub motor_forward std_msgs/Empty --once 
// (Publisher sends a message 'motor_forward to Subscriber)
// When Changes been made in Arduino, before uploading kill the roscore, rosrun USB topics

// Including Adafruit Motorshield V1
#include <AFMotor.h>

// Including ROS
#include <ros.h>

// Including Message Type
#include <std_msgs/Empty.h>

// Including Motor Declaration to use with Motor Shield V1
AF_DCMotor motor_Right(1); //M1
AF_DCMotor motor_Left(4); //M4

// Declaring ROS Node
ros::NodeHandle  nh;

// Functions for Subsribers

// Function 'MOVE FORWARD' in ROS Style ( Message, Type)
void message_forward( const std_msgs::Empty& toggle_msg){

  // Includes forward function
  forward();
  
}

// Function 'ROBOT MOVEMENT PAUSE' in ROS Style ( Message, Type)
void message_pause( const std_msgs::Empty& toggle_msgs){

  // Includes stop function
  stopping();
  
}

// Function 'MOVE BACKWARD' in ROS Style ( Message, Type)
void message_backward( const std_msgs::Empty& toggle_msg){

  // Includes backward function
  backward();
  
}

// Function 'MOVE LEFT' in ROS Style ( Message, Type)
void message_left( const std_msgs::Empty& toggle_msgs){

  // Includes left function
  left();
  
}

// Function 'MOVE RIGHT' in ROS Style ( Message, Type)
void message_right( const std_msgs::Empty& toggle_msg){

  // Includes right function
  right();
  
}

// Function 'SPIN LEFT' in ROS Style ( Message, Type)
void message_spin_left( const std_msgs::Empty& toggle_msgs){

  // Includes spin to the left function
  spinLeft();
  
}

// Function 'SPIN RIGHT' in ROS Style ( Message, Type)
void message_spin_right( const std_msgs::Empty& toggle_msgs){

  // Includes spin to the right function
  spinRight();
  
}

// You can change 'sub' to any other variable to declare more than one Subscriber
// "motor_forward" can be changed to any other variable

// ROS SUBSCRIBER, Subscribes to message forward and robot moves forward
ros::Subscriber<std_msgs::Empty> sub("motor_forward", &message_forward);

// ROS SUBSCRIBER, Subscribes to message pause and robot pauses
ros::Subscriber<std_msgs::Empty> sub1("motor_stop", &message_pause);

// ROS SUBSCRIBER, Subscribes to message backward and robot moves backward
ros::Subscriber<std_msgs::Empty> sub2("motor_backward", &message_backward);

// ROS SUBSCRIBER, Subscribes to message left and robot moves left
ros::Subscriber<std_msgs::Empty> sub3("motor_left", &message_left);

// ROS SUBSCRIBER, Subscribes to message right and robot moves right
ros::Subscriber<std_msgs::Empty> sub4("motor_right", &message_right);

// ROS SUBSCRIBER, Subscribes to message spin left and robot spins left
ros::Subscriber<std_msgs::Empty> sub5("motor_spin_left", &message_spin_left);

// ROS SUBSCRIBER, Subscribes to message spin right and robot spins right
ros::Subscriber<std_msgs::Empty> sub6("motor_spin_right", &message_spin_right);

void setup()
{ 
  Serial.begin(9600);

  // Initializing Node function
  nh.initNode();

  // Node Subsribes to 'sub' which is Motor Forward Message
  nh.subscribe(sub);

  // Node Subsribes to 'sub1' which is Motor Stop Message
  nh.subscribe(sub1);

  // Node Subsribes to 'sub2' which is Motor Backward Message
  nh.subscribe(sub2);

  // Node Subsribes to 'sub3' which is Motor Left Message
  nh.subscribe(sub3);

  // Node Subsribes to 'sub4' which is Motor Right Message
  nh.subscribe(sub4);

  // Node Subsribes to 'sub5' which is Motor Spin Left Message
  nh.subscribe(sub5);
  
  // Node Subsribes to 'sub6' which is Motor Spin Right Message
  nh.subscribe(sub6);
}

void loop()
{  
  // Node Spins once function
  nh.spinOnce();

  // Declaring 1s delay
  delay(1);
}

// Drive forward function
void forward()
{
  Serial.println("Driving forward");

  // You need to set a speed to work
  motor_Right.setSpeed(255);

  // Motor .run built in ADAFRUIT Motor Shield functionality
  motor_Right.run(FORWARD);

  // You need to set a speed to work
  motor_Left.setSpeed(255);

  // Motor .run built in ADAFRUIT Motor Shield functionality
  motor_Left.run(FORWARD);
}

// Drive backward function
void backward()
{
  Serial.println("Driving backwards");

  // You need to set a speed to work
  motor_Right.setSpeed(255);

  // Motor .run built in ADAFRUIT Motor Shield functionality
  motor_Right.run(BACKWARD);

  // You need to set a speed to work
  motor_Left.setSpeed(255);

  // Motor .run built in ADAFRUIT Motor Shield functionality
  motor_Left.run(BACKWARD);
}

// Drive left function
void left()
{
  Serial.println("Turning to the left");

  // You need to set a speed to work
  motor_Right.setSpeed(255);

  // Motor .run built in ADAFRUIT Motor Shield functionality
  motor_Right.run(FORWARD);

  // You need to set a speed to work
  motor_Left.setSpeed(255);

  // Motor .run built in ADAFRUIT Motor Shield functionality
  motor_Left.run(RELEASE);
}

// Drive right function
void right()
{
  Serial.println("Turning to the right...");
  
  // You need to set a speed to work
  motor_Right.setSpeed(255);
  
  // Motor .run built in ADAFRUIT Motor Shield functionality
  motor_Right.run(RELEASE);
  
  // You need to set a speed to work
  motor_Left.setSpeed(255);
  
  // Motor .run built in ADAFRUIT Motor Shield functionality
  motor_Left.run(FORWARD);
}

// Stop driving function
void stopping()
{
  Serial.println("Stopping");
  
  // You need to set a speed to work
  motor_Right.setSpeed(0);

  // Motor .run built in ADAFRUIT Motor Shield functionality
  motor_Right.run(RELEASE);

  // You need to set a speed to work
  motor_Left.setSpeed(0);

  // Motor .run built in ADAFRUIT Motor Shield functionality
  motor_Left.run(RELEASE);
}

// Spin around to the right function
void spinRight()
{
  Serial.println("I am spinning to the right");
  
  // You need to set a speed to work
  motor_Right.setSpeed(255);
  
  // Motor .run built in ADAFRUIT Motor Shield functionality
  motor_Right.run(BACKWARD);

  // You need to set a speed to work
  motor_Left.setSpeed(255);

  // Motor .run built in ADAFRUIT Motor Shield functionality
  motor_Left.run(FORWARD);
}

// Spin around to the left function
void spinLeft()
{
  Serial.println("Spining Left...");

  // You need to set a speed to work
  motor_Right.setSpeed(255);

  // Motor .run built in ADAFRUIT Motor Shield functionality
  motor_Right.run(FORWARD);

  // You need to set a speed to work
  motor_Left.setSpeed(255);
  
  // Motor .run built in ADAFRUIT Motor Shield functionality
  motor_Left.run(BACKWARD);
}
