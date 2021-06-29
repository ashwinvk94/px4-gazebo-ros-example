#include "manual_flight.h"
ManualFlight::ManualFlight(ros::NodeHandle& n, bool test_flag):
	nh_(n)
{
	//ROS subscribers and publishers initialization
	sub_px4_state_ = nh_.subscribe<mavros_msgs::State>("mavros/state", 10, &ManualFlight::stateCallback, this);
	sub_keyboard_ = nh_.subscribe<keyboard::Key>("/keyboard/keydown", 10, &ManualFlight::keyboardCallback, this);
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
                  mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
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

    land_keyboard_trigger_ = false;

	// main while loop
	if(!test_flag) {
		while(ros::ok()){
			
			// check and move to offboard mode and arm
			movetoArmOffboard();

			// Run mission if armed and in offboard mode
			if(current_state_.armed && current_state_.mode == "OFFBOARD"){
				
                position_target_msg_.header.stamp = ros::Time::now();
				position_target_msg_.position.z = 2;

				// trigger landing if time has crossed last value in time_intervals
				if(land_keyboard_trigger_){
					// END mission after moving to land mode
					movetoLand();
					break;
				}
			}
			// publish setpoint using mavros
			pub_mavros_setpoint_raw_.publish(position_target_msg_);
			ros::spinOnce();
			rate.sleep();
		}
	}
}

void ManualFlight::movetoArmOffboard()
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

void ManualFlight::movetoLand()
{	
	// move to land mode
	ROS_INFO("Landing..");
	if( set_mode_client_.call(land_set_mode_) && land_set_mode_.response.mode_sent){
		ROS_INFO("Land Mode enabled");
	}
}

ManualFlight::~ManualFlight()
{
}

void ManualFlight::stateCallback(const mavros_msgs::State::ConstPtr& msg)
{	
	current_state_ = *msg;
}

void ManualFlight::keyboardCallback(const keyboard::Key::ConstPtr& msg)
{	
    switch(msg->code)
    {
        case 273: 
                // up key pressed
                position_target_msg_.position.x += 1;
                break;
        case 274:
                // down key pressed
                position_target_msg_.position.x -= 1;
                break;
        case 276:
                // down key pressed
                position_target_msg_.position.y += 1;
                break;
        case 275:
                // down key pressed
                position_target_msg_.position.y -= 1;
                break;
        case 122:
                // 'z' key pressed
                position_target_msg_.yaw += M_PI/4;
                break;
        case 120:
                // 'x' key pressed
                position_target_msg_.yaw -= M_PI/4;
                break;
        case 108:
                // 'l' key pressed
                land_keyboard_trigger_ = true;
                break;
        default:
                // code to be executed if n doesn't match any cases
                ROS_WARN_STREAM("unknown key pressed");
    }
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "position_setpoint_publisher");
    ros::NodeHandle n;
    ManualFlight rc_pub(n);
    ROS_INFO("position_setpoint_publisher node initiated");
    // ros::spin();
    return 0;
}