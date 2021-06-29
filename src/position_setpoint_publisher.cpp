#include "position_setpoint_publisher.h"
PositionSetpointPublisher::PositionSetpointPublisher(ros::NodeHandle& n, bool test_flag):
	nh_(n)
{
	//ROS subscribers and publishers initialization
	sub_px4_state_ = nh_.subscribe<mavros_msgs::State>("mavros/state", 10, &PositionSetpointPublisher::stateCallback, this);
	pub_mavros_setpoint_raw_ = nh_.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10);
	
	// ROS Services initialization
	set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");

	// main loop Rate
	ros::Rate rate(main_while_rate_); // ROS Rate
    // wait for FCU connection
    while(ros::ok() && !current_state_.connected){
		ROS_INFO("Waiting to connect");
        ros::spinOnce();
        rate.sleep();
    }

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        pub_mavros_setpoint_raw_.publish(position_target_msg_);
        ROS_INFO("Publishing a few setpoints before starting");
        ros::spinOnce();
        rate.sleep();
    }

	// mavros setpoint message initialization
	position_target_msg_.header.frame_id = "odom";
	position_target_msg_.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
	position_target_msg_.type_mask = mavros_msgs::PositionTarget::IGNORE_VX | mavros_msgs::PositionTarget::IGNORE_VY |
                  mavros_msgs::PositionTarget::IGNORE_VZ | mavros_msgs::PositionTarget::IGNORE_AFX |
                  mavros_msgs::PositionTarget::IGNORE_AFY | mavros_msgs::PositionTarget::IGNORE_AFZ |
                  mavros_msgs::PositionTarget::IGNORE_YAW;
	position_target_msg_.position.x = 0.0f;
	position_target_msg_.position.y = 0.0f;
	position_target_msg_.position.z = 0.0f;
	position_target_msg_.acceleration_or_force.x = 0.0f;
	position_target_msg_.acceleration_or_force.y = 0.0f;
	position_target_msg_.acceleration_or_force.z = 0.0f;
	position_target_msg_.velocity.x = 0.0f;
	position_target_msg_.velocity.y = 0.0f;
	position_target_msg_.velocity.z = 0.0;
	position_target_msg_.yaw = 0.0f;
	position_target_msg_.yaw_rate = 0.0f;

	// mode messages intialization
    offb_set_mode_.request.custom_mode = "OFFBOARD";
    land_set_mode_.request.custom_mode = "AUTO.LAND";

	// arm messages intialization
    arm_cmd_.request.value = true;

	// ros time initialzation for timing the demo flight
    last_request_ = ros::Time::now();
	vehicle_arm_time_ = ros::Time::now();

	// main while loop
	if(!test_flag) {
		while(ros::ok()){
			
			// check and move to offboard mode and arm
			movetoArmOffboard();

			// Run mission if armed and in offboard mode
			if(current_state_.armed && current_state_.mode == "OFFBOARD"){

				// trigger landing if 10 seconds has passed after arming
				if((ros::Time::now() - last_request_ > ros::Duration(10.0))){
					// END mission after moving to land mode
					movetoLand();
					break;
				}
				// else hover at the start position
				else{
					ROS_INFO("Hovering at home location..");
					position_target_msg_.header.stamp = ros::Time::now();
					position_target_msg_.position.x = 0;
					position_target_msg_.position.y = 0;
					position_target_msg_.position.z = 2;
					position_target_msg_.yaw_rate = 0;
				}
			}
			else{
				// publish zero position setpoint when not armed and in offboard
				// to prevent failsafe trigger
				ROS_INFO("Not armed and in offboard yet...");
				position_target_msg_.header.stamp = ros::Time::now();
				position_target_msg_.position.x = 0;
				position_target_msg_.position.y = 0;
				position_target_msg_.position.z = 0;
				position_target_msg_.yaw_rate = 0;
			}
			// publish setpoint using mavros
			pub_mavros_setpoint_raw_.publish(position_target_msg_);
			ros::spinOnce();
			rate.sleep();
		}
	}
}

void PositionSetpointPublisher::movetoArmOffboard()
{	
	// move to offboard mode if not in offboard mode
	if( current_state_.mode != "OFFBOARD" &&
		(ros::Time::now() - last_request_ > ros::Duration(5.0))){
		if( set_mode_client_.call(offb_set_mode_) &&
			offb_set_mode_.response.mode_sent){
			ROS_INFO("Offboard enabled");
		}
		last_request_ = ros::Time::now();
	}
	// arm if in offboard mode and not armed
	else {
		if( !current_state_.armed &&
			(ros::Time::now() - last_request_ > ros::Duration(5.0))){
			if( arming_client_.call(arm_cmd_) &&
				arm_cmd_.response.success){
				ROS_INFO("Vehicle armed");
				vehicle_arm_time_ = ros::Time::now();
			}
			last_request_ = ros::Time::now();
		}
	}
}

void PositionSetpointPublisher::movetoLand()
{	
	// move to land mode
	ROS_INFO("Landing..");
	if( set_mode_client_.call(land_set_mode_) && land_set_mode_.response.mode_sent){
		ROS_INFO("Land Mode enabled");
	}
}

PositionSetpointPublisher::~PositionSetpointPublisher()
{
}

void PositionSetpointPublisher::stateCallback(const mavros_msgs::State::ConstPtr& msg)
{	
	current_state_ = *msg;
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "position_setpoint_publisher");
    ros::NodeHandle n;
    PositionSetpointPublisher rc_pub(n);
    ROS_INFO("position_setpoint_publisher node initiated");
    // ros::spin();
    return 0;
}