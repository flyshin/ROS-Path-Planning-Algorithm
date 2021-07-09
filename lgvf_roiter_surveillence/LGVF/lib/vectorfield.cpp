#include "vectorfield.h"



 geometry_msgs::PoseStamped current_state;  // recieve the attitude data from pixhawk(px4-based gazebo)
 mavros_msgs::VFR_HUD current_state_speed;  // recieve the current airspeed from pixhawk(px4-based gazebo)
 mavros_msgs::AttitudeTarget attitudecommand; // to publish the attitude command to pixhawk(px4-based gazebo)



// definition

void state_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
  {
   current_state =  *msg;
  }

void state_speed(const mavros_msgs::VFR_HUD::ConstPtr& msg)
  { 
   current_state_speed =  *msg; 
  }



double Vectorfield::Vectorfield_Guidance(double x_position, double y_position, double z_position, double Speed)
   {
    double r = sqrt(pow(x_position - target_position_x,2.0) + pow(y_position - target_position_y,2.0) );

    double f1_p = (x_position - target_position_x) * ( pow(r,2) - pow(desired_standoff_distance,2) ) / kl_gain + (y_position - target_position_y) * (2*r*desired_standoff_distance);
    double f2_p = (y_position - target_position_y) * ( pow(r,2) - pow(desired_standoff_distance,2) ) / kl_gain - (x_position - target_position_x) * (2*r*desired_standoff_distance); 

    if(r<0.1)
       	{
         f1 = -Speed * f1_p;
         f2 = -Speed * f2_p; 

        }
    else
      	{
          f1 = -Speed * f1_p/r;
          f2 = -Speed * f2_p/r;
        }

      
       if(r==0)
         {
          double yaw_c = 0;
         }
         
      else
         {
           double Tx =  target_velocity_x -  wind_x;
           std::cout<<"Tx"<<Tx<<std::endl;

           double Ty =  target_velocity_y -  wind_y;
           std::cout<<"Ty"<<Ty<<std::endl;

           double a  =  pow(f1,2) + pow(f2,2);
           std::cout<<"a"<<a<<std::endl;

           double b  =  f1*Tx + f2*Ty;
           std::cout<<"b"<<b<<std::endl;

           double c  =  (pow(Tx,2) + pow(Ty,2))-pow(Speed,2);
            std::cout<<"c"<<c<<std::endl;

              if ( (pow(b,2) -a*c) > 0)
                {
                	double alpha =(-b + sqrt(pow(b,2)-a*c))/a;

                 	f1 =Tx + std::real(alpha)*f1;
                	f2 =Ty + std::real(alpha)*f2;
                 }
           
          }
 

       double yaw_c = atan2(f2,f1);
     

        std::cout<<"yaw_original"<<yaw_c<<std::endl;
       return  yaw_c;

   }


double Vectorfield::Vectorfield_dpsi(double x_position, double y_position, double Speed)
    {
        double r = sqrt(pow(x_position,2) + pow(y_position,2));
        double dpsi = 4 * Speed * (desired_standoff_distance + pow(r,2) )/ pow((pow(r,2)+pow(desired_standoff_distance,2)  ),2);

        return dpsi;
    }



void Altitude::Altitude_parameter(double P, double I, double Time)
	{
      P_gain =P;
      I_gain =I;
      dt =Time;
	}


double Altitude::AltitudeController(double desired_altitude, double current_altitude)
    {

       double error = desired_altitude-current_altitude;

       if(error>2.0)
       {
          error= 2;
       }
       else if(error<-5)
       { 
          error = -5;
       }

       double pout= error * P_gain;  
       integrater += error * dt;

       double iout= integrater * I_gain; 
       
       iout = iout + pitch_wind_up; //anti win dp to relize integrator

       double pitch = pout + iout;

       double pitch_pre_statuation = pitch;

        if(pitch>0.15)
       {
          pitch= 0.15;
      }
       else if(pitch<-0.15) 
       {
          pitch = -0.15;
       }
      
       double pitch_after_statuation = pitch;

       pitch_wind_up= pitch_after_statuation - pitch_pre_statuation;

       return pitch;
    }


void Speed::Speed_parameter(double P, double I, double Time)
	{

     P_gain =P;
     I_gain =I;
     dt =Time;
	}


 double Speed::SpeedController(double desired_speed, double current_speed)
    {

       double error = desired_speed-current_speed;

       if(error>2.0)
       {
          error= 2.0;
       }
       else if(error<-5)
       { 
          error = -5;
       }

       double pout= error * P_gain;  
       integrater += error * dt;

       double iout= integrater * I_gain; 

       iout = iout + speed_wind_up; //anti wind up

       double thrust = pout + iout;

       double thrust_pre_statuation = thrust;

        if(thrust > 0.8)
          {
           thrust= 0.8;
          }
       else if(thrust < -0.8) 
         {
          thrust = -0.8;
         }
      
       double thrust_after_statuation = thrust;
       speed_wind_up = thrust_after_statuation - thrust_pre_statuation;

       return thrust;
    }


void Heading::Heading_parameter(double P, double I, double Time)
	{

     P_gain =P;
     I_gain =I;
     dt =Time;
	}

double Heading::HeadingController(double desired_heading, double current_heading, double psid)
    {

       double error = desired_heading-current_heading;       
       error = remainder(error,2* M_PI);
       
       // Wrapped heading
       if (abs(error) > M_PI )
      {
      	error = error - 2*M_PI*sign(error);
      }
      

       if(error>1.0)
       {
          error= 1.0;
       }
       else if(error<-1)
       { 
          error = -1;
       }

       double pout= error * P_gain;  
       integrater += error * dt;
       double iout= integrater * I_gain; 

       iout = iout + heading_wind_up; // integrater antiwind up 
       double roll = pout + iout -psid;

       double roll_pre_statuation = roll;

       if(roll > 1)
          {
           roll= 1;
          }
       else if(roll < -1) 
         {
          roll = -1;
         }
       double roll_after_statuation = roll;
       
       heading_wind_up = roll_after_statuation - roll_pre_statuation;

       return roll;
    }




 double* quternionToeuler(double *current_angle)
       {
           
           //Current State
       	double x = current_state.pose.orientation.x;    
       	double y = current_state.pose.orientation.y;
       	double z = current_state.pose.orientation.z;
       	double w = current_state.pose.orientation.w;

           //Roll angle(x-axis rotation)
       	double sinr_cosp = 2*(w*x + y*z);
       	double cosr_cosp = 1 - 2*(x*x + y*y);
       	 current_angle[0] = std::atan2(sinr_cosp, cosr_cosp);  // roll angle

       	   //Pitch angle(y-axis rotation)
       	double sinp = 2*(w*y - z*x);
       	  
       	  if (std::abs(sinp) >= 1)
       	  	 current_angle[1] = std::copysign(M_PI/2, sinp); // use 90 degrees if out of range
       	  else
       	  	 current_angle[1] = std::asin(sinp);  //pitch angle

       	   // yaw ( y-axis rotation)
       	double siny_cosp = 2*(w*z + x*y);
       	double cosy_cosp = 1 -2*(y*y + z*z);
       	 current_angle[2] = std::atan2(siny_cosp, cosy_cosp); //yaw angle

     
         return current_angle;

       }




double* eulerToquternion(double *quater_angle, double *Desired_angle)   // I would like to send euler angle
          
          {

            double cy = cos(Desired_angle[2]*0.5);
            double sy = sin(Desired_angle[2]*0.5);
            double cp = cos(Desired_angle[1]*0.5);
            double sp = sin(Desired_angle[1]*0.5); 
            double cr = cos(Desired_angle[0]*0.5);
            double sr = sin(Desired_angle[0]*0.5);

            // quaternion q;
             quater_angle[0] =cy*cp*cr + sy*sp*sr;
             quater_angle[1] = cy*cp*sr - sy*sp*cr;
             quater_angle[2] = sy*cp*sr + cy*sp*cr;
             quater_angle[3] = sy*cp*cr - cy*sp*sr;            

             return quater_angle;
          }






double* attitude_mode(double *Desired_angle, double pich, double roll, double yaw)
          {
              	Desired_angle[0]= roll;
                Desired_angle[1]= pich;
                Desired_angle[2]= yaw;
              
                return Desired_angle;
          }

