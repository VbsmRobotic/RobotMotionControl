/*
    * subscribes to the topic that gives information about the robot position
    * publishes to the topic that makes the robot move. 
    * develop a method called move(distance) which makes the robot moves a certain distance then stops. 
    * develop a method called rotate(distance) which makes the robot rotate a certain angle then stops.
*/

#include<iostream>

#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include "geometry_msgs/Twist.h"


using namespace  std ;


ros::Publisher velocity_pub ;
ros::Subscriber pose_sub ;




void poseCallback(const turtlesim::Pose::ConstPtr & pose_message){
	double turtlesim_posex=pose_message->x ;
	double turtlesim_posey=pose_message->y ;
	double turtlesim_posetheta=pose_message->theta ;
    cout <<"X: " << turtlesim_posex << endl ;
    cout <<"Y: " << turtlesim_posey << endl ;
    cout <<"theta: " << turtlesim_posetheta << endl ;
}


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

