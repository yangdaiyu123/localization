#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "state_publisher");
    ros::NodeHandle n;
    ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
    ros::Rate loop_rate(30);

    const double degree = 20*M_PI/180;

    // robot state
    double tilt = 0, tinc = degree, swivel=0, angle=0, height=0, hinc=0.005;

    // message declarations
    sensor_msgs::JointState joint_state;


    while (ros::ok()) {
        //update joint_state
        joint_state.header.stamp = ros::Time::now();
        joint_state.header.frame_id = "steer_fl";
        joint_state.name.resize(6);
        joint_state.position.resize(6);
        joint_state.name[0] ="wheel_rr";
        joint_state.position[0] = swivel;
        joint_state.name[1] ="wheel_rl";
        joint_state.position[1] = swivel;
        joint_state.name[2] ="wheel_fr";
        joint_state.position[2] = swivel;
        joint_state.name[3] ="wheel_fl";
        joint_state.position[3] = swivel;
        joint_state.name[4] ="steer_fr";
        joint_state.position[4] = (swivel-M_PI)/20;
        joint_state.name[5] ="steer_fl";
        joint_state.position[5] = (swivel-M_PI)/20;


        //send the joint state
        joint_pub.publish(joint_state);


        // Create new robot state
        swivel += degree;

        
        while(swivel>2*M_PI) swivel-=2*M_PI;

        // This will adjust as needed per iteration
        loop_rate.sleep();
    }


    return 0;
}
