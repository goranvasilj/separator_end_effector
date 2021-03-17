
#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <sensor_msgs/MagneticField.h>
#include <std_msgs/Empty.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include "separator_end_effector/separator_service.h"
#include "dynamixel_workbench_msgs/DynamixelCommand.h"
ros::ServiceClient client;
int motor1_id, motor2_id;
int driving_torque;
int driving_speed;
int init_speed;
int init_torque;
int tightening_speed;
int tightening_torque;
enum end_effector_motor_state {idle, init, openning, closing, tightening};
end_effector_motor_state motor1_state = idle;
end_effector_motor_state motor2_state = idle;


std::string get_string_from_state(end_effector_motor_state state)
{
	switch(state)
	{
	case init:
		return "init";
		break;
	case openning:
		return "opening";
		break;
	case closing:
		return "closing";
		break;
	case tightening:
		return "tightening";
		break;
	}
	return "idle";
}


//setting torque to dynamixel motor
bool set_torque_limit_to_dynamixel(int motor_id, int torque)
{
	dynamixel_workbench_msgs::DynamixelCommand torque_limit_request;
	torque_limit_request.request.addr_name = "Torque_Limit";
	torque_limit_request.request.id = motor_id;
	torque_limit_request.request.value = torque;
	if (client.call(torque_limit_request))
	{
		if (torque_limit_request.response.comm_result == false)
		{
			return true; //false;
		}
		else
		{
			return true;
		}
	}
	else
	{
		ROS_ERROR("Failed to set torque to motor " + (char) (motor_id + 48));
	}
	return false;


}

//setting speed to dynamixel motor
bool set_speed_to_dynamixel(int motor_id, int speed)
{
	dynamixel_workbench_msgs::DynamixelCommand speed_request;
	speed_request.request.addr_name = "Moving_Speed";
	speed_request.request.id = motor_id;
	speed_request.request.value = speed;

	if (client.call(speed_request))
	{
		if (speed_request.response.comm_result == false)
		{
			return true; //false;
		}
		else
		{
			return true;
		}
	}
	else
	{
		ROS_ERROR("Failed to set speed to motor " + (char) (motor_id + 48));
	}
	return false;

}


bool set_speed_with_limited_torque_to_dynamixel(int motor_id, int speed, int torque)
{
	if (set_torque_limit_to_dynamixel(motor_id, torque))
	{
		if (set_speed_to_dynamixel(motor_id, speed))
		{
			return true;
		}
	}
	return false;
}


bool service_callback(separator_end_effector::separator_service::Request &req,
		separator_end_effector::separator_service::Response &res){
	if (req.req == "init1"){
		if (set_speed_with_limited_torque_to_dynamixel(motor1_id, init_speed, init_torque))
				motor1_state=init;
	}
	if (req.req ==  "init2"){
		if (set_speed_with_limited_torque_to_dynamixel(motor2_id, init_speed, init_torque))
				motor2_state=init;
	}
	if (req.req ==  "open1"){
		if (set_speed_with_limited_torque_to_dynamixel(motor1_id, driving_speed+1024, driving_torque))
			motor1_state=openning;
	}
	if (req.req ==  "open2"){
		if (set_speed_with_limited_torque_to_dynamixel(motor2_id, driving_speed+1024, driving_torque))
			motor2_state=openning;
	}
	if (req.req ==  "close1"){
		if (set_speed_with_limited_torque_to_dynamixel(motor1_id, driving_speed, driving_torque))
			motor1_state=closing;
	}
	if (req.req == "close2"){
		if (set_speed_with_limited_torque_to_dynamixel(motor2_id, driving_speed, driving_torque))
			motor2_state=closing;
	}
	if (req.req ==  "open_both"){
		if (set_speed_with_limited_torque_to_dynamixel(motor1_id, driving_speed+1024, driving_torque))
			motor1_state=openning;
		if (set_speed_with_limited_torque_to_dynamixel(motor2_id, driving_speed+1024, driving_torque))
			motor2_state=openning;
	}
	if (req.req ==  "close_both"){
		if (set_speed_with_limited_torque_to_dynamixel(motor1_id, driving_speed, driving_torque))
			motor1_state=closing;
		if (set_speed_with_limited_torque_to_dynamixel(motor2_id, driving_speed, driving_torque))
			motor2_state=closing;
	}
	if (req.req ==  "init_both"){
		if (set_speed_with_limited_torque_to_dynamixel(motor1_id, init_speed, init_torque))
				motor1_state=init;
		if (set_speed_with_limited_torque_to_dynamixel(motor2_id, init_speed, init_torque))
				motor2_state=init;
	}
	if (req.req ==  "stop"){
		if (set_speed_to_dynamixel(motor1_id, 0))
			motor1_state=idle;
		if (set_speed_to_dynamixel(motor2_id, 0))
			motor2_state=idle;
	}
	res.status = get_string_from_state(motor1_state) + " " + get_string_from_state(motor2_state);
	return true;

}



bool CheckState(end_effector_motor_state state, int id)
{
	switch(state)
	{
	case init:
		return "init";
		break;
	case openning:
		return "opening";
		break;
	case closing:
		return "closing";
		break;
	case tightening:
		return "tightening";
		break;
	}
	return true;
}



int main (int argc, char** argv){
    ros::init(argc, argv, "separator_end_effector");
    ros::NodeHandle nh;
    ros::NodeHandle nh_ns("~");

    int refresh_rate;
    std::string tool_service;
    std::string dynamixel_service;
    nh_ns.param("tool_service", tool_service, (std::string) "/tool_service");
    nh_ns.param("dynamixel_service", dynamixel_service, (std::string)"/dynamixel_workbench/dynamixel_command");
    nh_ns.param("motor1_id", motor1_id, 1);
    nh_ns.param("motor2_id", motor2_id, 6);
    nh_ns.param("driving_torque", driving_torque, 300);
    nh_ns.param("driving_speed", driving_speed, 100);
    nh_ns.param("init_speed", init_speed, 30);
    nh_ns.param("init_torque", init_torque, 300);
    nh_ns.param("tightening_speed", tightening_speed, 10);
    nh_ns.param("tightening_torque", tightening_torque, 300);

std::cout<<"motor id "<<motor1_id<<" "<<motor2_id<<std::endl;
    // client for dynamixel service
    client = nh.serviceClient<dynamixel_workbench_msgs::DynamixelCommand>(dynamixel_service);



    //set service callback
    ros::ServiceServer service = nh.advertiseService(tool_service, service_callback);


    ros::Rate loop_rate(100);
    while(ros::ok()){

        ros::spinOnce();
        CheckState(motor1_state, motor1_id);
        CheckState(motor2_state, motor2_id);
        loop_rate.sleep();


    }
}


