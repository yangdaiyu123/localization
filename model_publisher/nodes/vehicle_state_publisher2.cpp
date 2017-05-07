#include <string>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int8MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <deque>

using namespace std;

const double ODOMETRY_FACTOR = 0.0210386;
const double wheel_radius = 0.3;
double roll_angle=0;

ros::Publisher joint_pub;
sensor_msgs::JointState joint_state;

double steer_angle=0.0;
double curr_odom = 0.0, last_odom = 0.0;
deque<double> steer_fifo;

double speed_factor;
double steer_factor;
double pub_period;


void timerCallback(const ros::TimerEvent&)
{
	float distance_inc = curr_odom - last_odom;
	float roll_inc = distance_inc / ( M_PI * 2 * wheel_radius ) * 2 * M_PI;
	roll_angle += roll_inc;
	while(roll_angle>2*M_PI) roll_angle-=2*M_PI;
	roll_angle *= speed_factor;
	
	//update joint_state
	joint_state.header.stamp = ros::Time::now();
	joint_state.name.resize(6);
	joint_state.position.resize(6);
	joint_state.name[0] ="wheel_rr";
	joint_state.position[0] = roll_angle;
	joint_state.name[1] ="wheel_rl";
	joint_state.position[1] = roll_angle;
	joint_state.name[2] ="wheel_fr";
	joint_state.position[2] = roll_angle;
	joint_state.name[3] ="wheel_fl";
	joint_state.position[3] = roll_angle;
	joint_state.name[4] ="steer_fr";
	joint_state.position[4] = steer_angle* steer_factor;
	joint_state.name[5] ="steer_fl";
	joint_state.position[5] = steer_angle* steer_factor;

	//send the joint state
	joint_pub.publish(joint_state);

	last_odom = curr_odom;
}

void steerAngleCallback(std_msgs::Float64 steer_in)
{
	steer_fifo.push_back( (steer_in.data-20) * M_PI / 180 );
	double sum=0.0,mean=0.0;
	if( steer_fifo.size() > 10 )
	{
		steer_fifo.pop_front();
		for(int i=0; i<steer_fifo.size(); i++)
			sum += steer_fifo[i];
		mean = sum/steer_fifo.size();
		steer_angle = mean;	
	}
	
}

void pulseCallback(const std_msgs::Int8MultiArray::ConstPtr& input)
{
	int pulse_inc = (int)input->data[0] + (int)input->data[0];
	float odom_inc = pulse_inc * ODOMETRY_FACTOR / 2.0;
	curr_odom += odom_inc;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "state_publisher2");
    ros::NodeHandle nh;
    
    ros::NodeHandle pnh("~");
    pnh.param("period", pub_period, 0.1);
	pnh.param("speed_factor", speed_factor, 1.0);
	pnh.param("steer_factor", steer_factor, 0.1);
    
    joint_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 1);
    ros::Timer timer = nh.createTimer(ros::Duration(pub_period), timerCallback);
    ros::Subscriber sub_steer = nh.subscribe("steer_angle", 2, steerAngleCallback);
    ros::Subscriber sub_pulse = nh.subscribe("pulse", 2, pulseCallback);
	

	ros::spin();

    return 0;
}



