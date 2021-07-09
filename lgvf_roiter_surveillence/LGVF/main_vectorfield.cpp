#include "lib/vectorfield.h"


//----------------------------------------------------------------------------------------------------------------------------------------------------------

int main(int argc, char **argv)
{

//--------------------------------------------------------------------
// ROS subscribe and publish	
   ros::init(argc, argv, "main_vectorfield");
   ros::NodeHandle n;
   ros::Subscriber state_sub = n.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose",100,state_cb);
   ros::Subscriber speed_sub = n.subscribe<mavros_msgs::VFR_HUD>("/mavros/vfr_hud",100, state_speed);
   ros::Publisher dm_pub_attitude = n.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 100);
//-------------------------------------------------------------------
// 
   ros::Rate loop_rate(100); //ROS 0.01Hz 
   ros::spinOnce(); // adjust loop late
   int count = 1; 
//-------------------------------------------------------------------
// class call part 
   Altitude call; // setting integrater to 0 . from altitude class
   call.Altitude_parameter(-0.03, -0.0003, 0.01); // you can tune the PI gain valu

   Speed call1;   // setting integrater to 0 . from Speed class
   call1.Speed_parameter(0.65,0.025,0.01);

   Heading call2; // setting integrater to 0 . from Heading class
   call2.Heading_parameter(-0.85,-0.017,0.01);

   Vectorfield call3;
//--------------------------------------------------------------------


   while(ros::ok())
    {
      
       int arr[3];
       double current_angle[3];   
       double quater_angle[4];
       double Desired_angle[3];   // The desired angle we want to send

       quternionToeuler(current_angle);

       double yaw_c= call3.Vectorfield_Guidance(current_state.pose.position.x, current_state.pose.position.y, current_state.pose.position.z, current_state_speed.airspeed);    
       double dpsi = call3.Vectorfield_dpsi(current_state.pose.position.x, current_state.pose.position.y, current_state_speed.airspeed);

       double roll = call2.HeadingController(yaw_c, current_angle[2], dpsi); 
       double pitch = call.AltitudeController(50.0, current_state.pose.position.z);
       double thrust = call1.SpeedController(22.0, current_state_speed.airspeed);

       attitude_mode(Desired_angle,pitch,roll,yaw_c);
       eulerToquternion(quater_angle,Desired_angle); // chaned from euler angle to quterinon which pixhawk can subscribe
//----------------------------------------------------------------------
// ROS Topic publish part      
       attitudecommand.header.stamp = ros::Time::now();
       attitudecommand.header.seq=count;
       attitudecommand.header.frame_id = 1;
       attitudecommand.orientation.w = quater_angle[0];
       attitudecommand.orientation.x = quater_angle[1];
       attitudecommand.orientation.y = quater_angle[2];
       attitudecommand.orientation.z = quater_angle[3];
       attitudecommand.thrust = thrust;
     
       dm_pub_attitude.publish(attitudecommand); // Publish the topic 
     
       ros::spinOnce();
       count++;
       loop_rate.sleep();
//----------------------------------------------------------------------
    }
   
   return 0;
}






