/**
 * @file position_setpoint_publisher.h
 * @author  Ashwin Varghese Kuruttukulam (ashwinvk94@gmail.com)
 * @brief	This class is an example of how to send position
 * 			setpoints to a px4 system using mavros
 * 		
 * @version 0.1
 * @date 2021-06-29
 * 
 */
#ifndef POSITION_SETPOINT_PUBLISHER_H_
#define POSITION_SETPOINT_PUBLISHER_H_
#include <ros/ros.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

class PositionSetpointPublisher {
private:
	// Ros node handle
	ros::NodeHandle nh_;

	//main while loop rate
	int main_while_rate_ = 50;

	// Suscribers
	// px4 state sub
	ros::Subscriber sub_px4_state_;

	// Publishers
	// Publishes setpoints to the flight controller
	ros::Publisher pub_mavros_setpoint_raw_;

    // ROS Services
	ros::ServiceClient set_mode_client_, arming_client_;

	// Publisher messages
	// mavros attitudetarget msg
	mavros_msgs::PositionTarget position_target_msg_;
    // mavros state message
    mavros_msgs::State current_state_;

	// ROS time variables
	ros::Time last_request_;
	ros::Time vehicle_arm_time_;

	// service messages
	mavros_msgs::SetMode offb_set_mode_, land_set_mode_;
	mavros_msgs::CommandBool arm_cmd_;

public:
	/**
	 * @brief Construct a new Attitude Thrust Publisher object
	 * 
	 * @param n 
	 */
	PositionSetpointPublisher(ros::NodeHandle& n, bool test_flag = false);
	/**
	 * @brief Destroy the Attitude Thrust Publisher object
	 * 
	 */
	~PositionSetpointPublisher();
	/**
	 * @brief Callback function for mavros state
	 * 
	 * @param msg state message from px4
	 */
	void stateCallback(const mavros_msgs::State::ConstPtr& msg);
	/**
	 * @brief move the px4 system to offboard mode and arm the drone
	 * 
	 */
	void movetoArmOffboard();

};
#endif // POSITION_SETPOINT_PUBLISHER_H_