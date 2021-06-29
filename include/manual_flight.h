/**
 * @file manual_flight.h
 * @author  Ashwin Varghese Kuruttukulam (ashwinvk94@gmail.com)
 * @brief	This class allows controlling a px4 drone using a keyboard
 *          it reads keyboard strokes using https://github.com/lrse/ros-keyboard
 * 		
 * @version 1.0
 * @date 2021-06-29
 * 
 */
#ifndef MANUAL_FLIGHT_PUBLISHER_H_
#define MANUAL_FLIGHT_PUBLISHER_H_
#include <ros/ros.h>
#include <vector>
#include <math.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <keyboard/Key.h>

class ManualFlight {
private:
	// Ros node handle
	ros::NodeHandle nh_;

	//main while loop rate
	int main_while_rate_ = 50;

	// Suscribers
	// px4 state sub
	ros::Subscriber sub_px4_state_;
    ros::Subscriber sub_keyboard_;

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

    // land trigger flag
    bool land_keyboard_trigger_;

public:
	/**
	 * @brief Construct a new Attitude Thrust Publisher object
	 * 
	 * @param n 
	 */
	ManualFlight(ros::NodeHandle& n, bool test_flag = false);
	/**
	 * @brief Destroy the Attitude Thrust Publisher object
	 * 
	 */
	~ManualFlight();
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
	/**
	 * @brief move the px4 system to land mode
	 * 
	 */
	void movetoLand();
    /**
     * @brief keyboard press callback
     * 
     */
    void keyboardCallback(const keyboard::Key::ConstPtr& msg);

};
#endif // MANUAL_FLIGHT_PUBLISHER_H_