/**
 * @file main.cpp
 * @author Gabriel Zucchi (gabrielzucchi43@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2024-06-07
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include <cstdio>

#include <cmath>

#include "vehicle_interface_SOCKET.h"
#include "config.h"

#include <signal.h>


#include <string>
#include "pid.h"

#include "model_free.h"

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

using namespace std::chrono_literals;
using std::placeholders::_1;


auto pid_accel_to_pwm = [](double pid_output){
        return 1.23*pid_output + 11.43;
    };
auto pid_steer_to_pwm = [](double pid_output){
    return 0.6*abs(pid_output) + 40;
    };
auto clamp = [](int num, int lower_lim, int upper_lim){
     if(num > upper_lim) return upper_lim;
      else if(num < lower_lim) return lower_lim;
      return num;
};


class Main_Node : public rclcpp::Node{
  public:
    Main_Node() : Node("main_node"){
        //Declaration of Parameters
        this -> declare_parameter<float>("Kp",0.0);
        this -> declare_parameter<float>("Ki",0.0);
        this -> declare_parameter<float>("Kd",0.0);
        this -> declare_parameter("Mode","NaN");
        this -> declare_parameter("Status","ON_");
        //Create variables to store the value of these parameteres
        kp_ = get_parameter("Kp").as_double();
        ki_ = get_parameter("Ki").as_double();
        kd_ = get_parameter("Kd").as_double();
        mode_ = get_parameter("Mode").as_string();
        status_ = get_parameter("Status").as_string();
        speed_cmd_val = 0; // 0 to 100
        steer_cmd_val = 0;// -100 to 100
        brake_cmd_val = 0;// 0 to 100


        //Callback and its lambda function .Taken from ROS2 Iron documentation
        auto pre_set_parameter_callback = [this](std::vector<rclcpp::Parameter> & parameters) {
          for (auto & param : parameters) {
            // Check for the changed param name and change the correct variable 
            if (param.get_name() == "Kp") {
                kp_ = param.as_double();
            }
            else if(param.get_name() == "Ki"){
              ki_ = param.as_double();
            }
            else if(param.get_name() == "Kd"){
              kd_ = param.as_double();
            }
            else if(param.get_name() == "Mode"){
              mode_ = param.as_string();
            }
            else if(param.get_name() == "Status"){
              status_ = param.as_string();
            }
          }
        };

      pre_set_parameters_callback_handle_ = this->add_pre_set_parameters_callback(pre_set_parameter_callback);


      //Prepare PWM Output
      //setup_PWM_ports();
      socket = connect_to_action_server();
      RCLCPP_INFO(this->get_logger(), "PWM Setup NOT Done!");
      //manual_auto(1); //TO BE REPLACED WITH THE BUTTON LOGIC ON THE TRUCK
      toggle_auto();
      

      //SUBSCRIBE TO FEEDBACK VALUES AND SETPOINT VALUES
      setpoint_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, std::bind(&Main_Node::setpoint_callback, this, _1));
      feedback_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>("speed_steer_feedback", 10, std::bind(&Main_Node::feedback_callback, this, _1));

      //SET SOME THINGS FOR THE PID CONTROLLER OBS WITH 0 VALUES IT IS IGNORING FOR NOW
      
        Speed_control_PID.setI_err_max(100);
        Steer_control_PID.setI_err_max(0);
        Speed_control_PID.setOutputRampRate(0);
        Steer_control_PID.setOutputRampRate(0);
      

      //SET SOME THINGS FOR THE PID MOdel Free CONTROLLER OBS WITH 0 VALUES IT IS IGNORING FOR NOW
        Speed_control_MF.setI_err_max(100);
        Steer_control_MF.setI_err_max(0);
        Speed_control_MF.setOutputRampRate(0);
        Steer_control_MF.setOutputRampRate(0);
      
      

      RCLCPP_INFO(this->get_logger(), "System Ready | Waiting for ROS2 msgs!");


    }
    //Callback function for setpoint msg (cmd_vel) coming from the planner
    void setpoint_callback(const geometry_msgs::msg::Twist::SharedPtr msg){
      //UPDATE SETPOINT OF THE CONTROLLER
      if(mode_ == "pid"){
      Speed_control_PID.setSetpoint(msg->linear.x);
      Steer_control_PID.setSetpoint(msg->angular.z);
      }

      else if(mode_ == "mf"){
      Speed_control_MF.setSetpoint(msg->linear.x);
      Steer_control_MF.setSetpoint(msg->angular.z);
      }


      //PLACEHOLDER
      RCLCPP_INFO(this->get_logger(), "Setpoint Twist msg: Speed-> %.3f  Steer-> %.3f",msg->linear.x, msg->angular.z);

    }
    //Callback function for feedback msg (cmd_vel) coming from the raspberry/CAN network

    void feedback_callback(const geometry_msgs::msg::Twist::SharedPtr msg){
      //UPDATE feedback value OF THE CONTROLLER
      if(msg->linear.z == 1.0) {
        //SEND PWM TO TRUCK SPEED
        if(mode_ == "pid"){
          speed_cmd_val = round( pid_accel_to_pwm( Speed_control_PID.getOutput( msg->linear.x ) ) ) ;
        }
        else if(mode_ == "mf"){
          speed_cmd_val = round( pid_accel_to_pwm( Speed_control_MF.getOutput( msg->linear.x ) ) ) ;
        }
        speed_cmd_val = clamp (speed_cmd_val,0,100);
      
      }

      else {  
        //SEND PWM TO TRUCK STEER
        if(mode_ == "pid"){
          steer_cmd_val = round( pid_steer_to_pwm( Steer_control_PID.getOutput( msg->angular.z ) ) );
        }

        else if(mode_ == "mf"){
          steer_cmd_val = round( pid_steer_to_pwm( Steer_control_MF.getOutput( msg->angular.z ) ) );
        }

        steer_cmd_val = clamp(steer_cmd_val,-100,100);
      }

      //pwm_update(speed_cmd_val, steer_cmd_val, brake_cmd_val);
      send_data(socket,speed_cmd_val, steer_cmd_val, brake_cmd_val);
      //PLACEHOLDER
      RCLCPP_INFO(this->get_logger(), "Sent msg to interface: Speed-> %d  Steer-> %d  Brake -> %d",speed_cmd_val, steer_cmd_val, brake_cmd_val);

    }
    

  private:
    rclcpp::node_interfaces::PreSetParametersCallbackHandle::SharedPtr pre_set_parameters_callback_handle_;
    rclcpp::TimerBase::SharedPtr timer_;
    float kp_ , ki_, kd_;
    int speed_cmd_val, steer_cmd_val, brake_cmd_val ,socket;// Values to send to interface
    std::string mode_, status_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr setpoint_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr feedback_subscription_;
    //Create and init the Controllers for Speed and Steer
    PID Speed_control_PID = PID(5,0.5,0.05);
    PID Steer_control_PID = PID(4,0.0,0.0);

//MF::MF(double P_,double I_,double D_,double Ts_, int N_,double alpha_)
    MF Speed_control_MF = MF(5,0.5,0.05 , 0.1, 5, 1.0);
    MF Steer_control_MF = MF(4,0.0,0.0 , 0.1, 5, 1.0);
  
};


void exit_handler(int s){
  //truck_sleep();
  //printf("EXITING, INTERFACE OFF");
  exit(0);
}



int main(int argc, char ** argv)
{
  signal (SIGINT, exit_handler);
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Main_Node>());
  rclcpp::shutdown();
  return 0;
}
