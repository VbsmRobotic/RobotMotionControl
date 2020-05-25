

Start the Turtlesim Simulator

First, run the Turtlesim simulator using this command:

    > roscore
    > rosrun turtlesim turtlesim_node

You will develop a program to control this robot, STEP BY STEP following the instruction below. 

We want to make this robot move and display its location. So let us get started. 

1- Find the topic name of the pose (position and orientation) of turtlesim and its message type. 
Display the content of message of the pose.

$ rostopic list

/rosout
/turtle1/cmd_vel
/turtle1/color_sensor
/turtle1/pose

So, all the topics will be displayed. Among them, the topic /turtle1/pose represents the pose of the robot.

To know the message type, then we write:

$ rostopic info /turtle1/pose 

Type: turtlesim/Pose

Publishers: 
 * /turtlesim (http://vahid-G5-5590:40653/)

Subscribers: None


So, we understand that turtlesim/Pose is the message type of the topic. 

We can show the content of the message using the command

$ rosmsg show turtlesim/Pose


float32 x
float32 y
float32 theta
float32 linear_velocity
float32 angular_velocity

So, we understand that x, y represent the location of the robot, and theta representation the orientation of the robot. 

2- Find the topic name of the velocity command of turtlesim and its message type. Display the content of message of the velocity command.

Remember that velocity command (cmd_vel) is the topic that makes the robot move.

You need first to run 

$ rostopic list

/rosout
/turtle1/cmd_vel
/turtle1/color_sensor
/turtle1/pose

$ rostopic info /turtle1/cmd_vel 

Type: geometry_msgs/Twist

Publishers: None

Subscribers: 
 * /turtlesim (http://vahid-G5-5590:40653/)

So, we understand that geometry/Twist  is the message type of the topic. 
We can show the content of the message using the command

rosmsg show geometry_msgs/Twist
geometry_msgs/Vector3 linear
  float64 x
  float64 y
  float64 z
geometry_msgs/Vector3 angular
  float64 x
  float64 y
  float64 z

So, we understand that a Twist message has two main components: linear and angular.

linear, represents the linear velocity (velocity when the robot moves in straight line forward or backward)

angular, represents the angular velocity (velocity when the robot rotates left and right)

every velocity has three coordinates x, y and z, in three dimensional space. 


3- Write a simple ROS program called controller.cpp, 
which subscribes to the topic of the pose, and then prints the position of the robot in the callback function.

3-1 : #include<iostream>

#include "ros/ros.h"
#include "turtlesim/Pose.h"


using namespace  std ;


/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
// void poseCallback(const std_msgs::String::ConstPtr& msg)
// {
//   ROS_INFO("I heard: [%s]", msg->data.c_str());
// }
void poseCallback(const turtlesim::Pose::ConstPtr & pose_message){
    double turtlesim_posex=pose_message->x ;
    double turtlesim_posey=pose_message->y ;
    double turtlesim_posetheta=pose_message->theta ;
    cout <<"X: " << turtlesim_posex << endl ;
    cout <<"Y: " << turtlesim_posey << endl ;
    cout <<"theta: " << turtlesim_posetheta << endl ;
}

int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "robot_motion_controller");   //node name is robot_motion_controller

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called poseCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
  ros::Subscriber pose_sub = n.subscribe("/turtle1/pose", 1000, poseCallback);    //topic name 

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  return 0;
}

3-2 : add excutable node to CMakeLists    /src/control_motion/CMakeLists.txt

      #robot_motion_controller
      add_executable(robot_motion_controller src/controller.cpp)
      target_link_libraries(robot_motion_controller ${catkin_LIBRARIES})



3-3 : open three termminals  and run :
$ rosrun turtlesim turtlesim_node
$ rosrun control_motion robot_position
$ rosrun turtlesim turtle_teleop_key 

if you move turtle, you can monitor 
X: 5.54444
Y: 5.54444
theta: 0
X: 5.54444
Y: 5.54444
theta: 0
X: 5.54444
Y: 5.54444
theta: 0
X: 5.54444
Y: 5.54444
theta: 0

turtle is located at middile of simulator (5,5,0)

4- Complete the previous code to add a publisher to the velocity.

#include<iostream>

#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "geometry_msgs/Twist.h"


using namespace  std ;




void poseCallback(const turtlesim::Pose::ConstPtr & pose_message){
	  double turtlesim_posex=pose_message->x ;
	  double turtlesim_posey=pose_message->y ;
	  double turtlesim_posetheta=pose_message->theta ;
    cout <<"X: " << turtlesim_posex << endl ;
    cout <<"Y: " << turtlesim_posey << endl ;
    cout <<"theta: " << turtlesim_posetheta << endl ;
}


int main(int argc, char **argv)
{
 
  ros::init(argc, argv, "robot_motion_controller");   //node name is robot_motion_controller

 
  ros::NodeHandle n;


  ros::Subscriber pose_sub = n.subscribe("/turtle1/pose", 1000, poseCallback);    //topic name 

  ros::Publisher  velocity_pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);



  //Rate is class is used to define frequency for a loop. Here we send a message each 1 second
  ros::Rate loop_rate(1); 
  
  
  ros::spin();


  return 0;

} 

5- Complete the previous code make the robot move for a certain distance.
Develop a method called move(distance) which makes the robot moves a certain distance then stops.

5-1 : 

must define velocity_pub and pose_sub as global

ros::Publisher velocity_pub ;
ros::Subscriber pose_sub ;

/**
 *  makes the robot move with a certain linear velocity for a
 *  certain distance in a forward or backward straight direction.
 */
void move(double speed, double distance, char status){
	geometry_msgs::Twist vel_msg ;
	//set a random linear velocity in the x-axis
    switch(status)
    {
        case 'F' :
            vel_msg.linear.x =abs(speed) ;
            break;
        case 'B' : 
            vel_msg.linear.x =-abs(speed) ;
            break ;
    }		
	vel_msg.linear.y =0 ;
	vel_msg.linear.z =0 ;
	//set a random angular velocity in the y-axis
	vel_msg.angular.x = 0 ;
	vel_msg.angular.y = 0 ;
	vel_msg.angular.z = 0 ;

	double t0 = ros::Time::now().toSec() ;
	double current_distance = 0.0 ;
    //Rate is class is used to define frequency for a loop. Here we send a message each 100 second
	ros::Rate loop_rate(100) ;
	do{
        /**
         * The publish() function is how you send messages. The parameter
         * is the message object (velocity_pub). The type of this object must agree with the type
         * given as a template parameter to the advertise<>() call, as was done
         * in the constructor above.
         */
		velocity_pub.publish(vel_msg);     //put the message in the buffer 
		double t1 = ros::Time::now().toSec();
		current_distance = speed * (t1-t0);     //x = vt
		ros::spinOnce();
		loop_rate.sleep();
		cout<<(t1-t0)<<", "<<current_distance <<", "<<distance<<endl;
	}while(current_distance<distance);
	vel_msg.linear.x =0;
	velocity_pub.publish(vel_msg);
}




5-2:Run move function:

int main(int argc, char **argv)
{
 
  ros::init(argc, argv, "robot_motion_controller") ;   //node name is robot_motion_controller
 
  ros::NodeHandle n ;

  pose_sub = n.subscribe("/turtle1/pose", 1000, poseCallback) ;    //topic name 

  velocity_pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000) ;

  double S, D ;
  char status ;

  /** test your code here **/
  ROS_INFO("\n\n\n******START TESTING************\n") ;
  cout<<"enter speed: " ;
  cin>>S ;
  cout<<"enter distance: " ;
  cin>>D ;
  cout<<"Status : F/B " ;
  cin>>status ;
  move(S, D, status); 


  ros::spin() ;

  return 0 ;

} 



