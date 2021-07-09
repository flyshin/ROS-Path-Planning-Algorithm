#include <ros/ros.h>
#include <std_msgs/String.h> 
#include <stdio.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "mavros_msgs/Thrust.h"
#include "mavros_msgs/AttitudeTarget.h"
#include "mavros_msgs/VFR_HUD.h"


using boost::math::sign;


//------------------------------------------------------------------------------------------------------------------------------------------- 
//declaration

extern geometry_msgs::PoseStamped current_state;  // recieve the attitude data from pixhawk(px4-based gazebo)
extern mavros_msgs::VFR_HUD current_state_speed;  // recieve the current airspeed from pixhawk(px4-based gazebo)
extern mavros_msgs::AttitudeTarget attitudecommand; // to publish the attitude command to pixhawk(px4-based gazebo)

extern void state_cb(const geometry_msgs::PoseStamped::ConstPtr& msg); 
extern void state_speed(const mavros_msgs::VFR_HUD::ConstPtr& msg);

extern double* quternionToeuler(double *current_angle); // recieve the current attitude data  using ROS subscribe topic from pixhawk(px4_based gazebo) such as and change quaternion to euler angle
extern double* eulerToquternion(double *quater_angle, double *Desired_angle);   // I would like to send euler angle      
extern double* attitude_mode(double *Desired_angle, double pich, double roll, double yaw);


class Vectorfield
{
private:
  double target_position_x;
  double target_position_y;
  double target_velocity_x;
  double target_velocity_y;
  double desired_standoff_distance;
  double kl_gain;
  double wind_x;
  double wind_y;
  double f1;
  double f2;

public:

   Vectorfield()
   {
     target_position_x= 0;
     target_position_y= 0;
     target_velocity_x= 0;
     target_velocity_y= 0;
     desired_standoff_distance= 100;
     kl_gain = 0.42;
     wind_x = 0;
     wind_y = 0;
   };

   double Vectorfield_Guidance(double x_position, double y_position, double z_position, double Speed);
   double Vectorfield_dpsi(double x_position, double y_position, double Speed);

};



class Altitude
{
public:
	Altitude() //Constructor! you have to understand this!  it is for inlitial valu.
	{
		integrater = 0;
		pitch_wind_up =0;
	};

	void Altitude_parameter(double P, double I, double Time);
    double AltitudeController(double desired_altitude, double current_altitude);
  
private:
	double P_gain;
    double I_gain;
    double dt;
    double integrater;
    double pitch_wind_up;
};


class Speed
{
public:
	Speed() //Constructor! you have to understand this!  it is for inlitial valu.
	{
		integrater = 0;
		speed_wind_up = 0;
	};

	void Speed_parameter(double P, double I, double Time);
    double SpeedController(double desired_speed, double current_speed);
   
private:
	double P_gain;
    double I_gain;
    double dt;
    double integrater;
    double speed_wind_up;

};



class Heading
{
public:
	Heading() //Constructor! you have to understand this!  it is for inlitial valu.
	{
		integrater = 0;
		heading_wind_up =0;
	};


	//~Altitude(); somulsa
	void Heading_parameter(double P, double I, double Time);
    double HeadingController(double desired_heading, double current_heading, double psid);
   
private:
	double P_gain;
    double I_gain;
    double dt;
    double integrater;
    double heading_wind_up;

};


