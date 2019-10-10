#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

float max_vel=1.00;
geometry_msgs:Twist setpoint_vel;

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg1){
    current_state = *msg1;
}

geometry_msgs::TwistStamped current_vel;
void vel_cb(const geometry_msgs::TwistStamped::ConstPtr& msg2){
	current_vel = *msg2;
}

geometry_msgs::Twist current_vel_control;
void vel_cb(const geometry_msgs::Twist::ConstPtr& msg3){
	current_vel_control = *msg3;
	setpoint_vel.linear.x = ((current_vel_control.linear.x%100)/100)*max_vel;
    setpoint_vel.linear.y = ((current_vel_control.linear.y%100)/100)*max_vel;
    setpoint_vel.linear.z = ((current_vel_control.linear.z%100)/100)*max_vel;
    setpoint_vel.angular.x = ((current_vel_control.angular.x%100)/100)*max_vel;
	setpoint_vel.angular.y = ((current_vel_control.angular.y%100)/100)*max_vel;
	setpoint_vel.angular.z = ((current_vel_control.angular.z%100)/100)*max_vel;
	
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "velocity_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber vel_sub = nh.subscribe<geometry_msgs::TwistStamped>
    	    ("/mavros/local_position/velocity", 100, vel_cb);
    ros::Subscriber vel_control_sub = nh.subscribe<geometry_msgs::Twist>
    	    ("/vel_control_topic", 100, vel_control_cb);
    ros::Publisher vel_pub = nh.advertise<geometry_msgs:Twist>
            ("/mavros/setpoint_velocity/cmd_vel_unstamped", 100);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    setpoint_vel.linear.x = 0;
    setpoint_vel.linear.y = 0;
    setpoint_vel.linear.z = 0;
    setpoint_vel.angular.x = 0;
	setpoint_vel.angular.y = 0;
	setpoint_vel.angular.z = 0;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        vel_pub.publish(initial_vel);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        vel_pub.publish(setpoint_vel);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
