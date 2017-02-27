#include "icp_localizer.h"
#include "ros/ros.h"

using namespace std;

ICPLocalizer::ICPLocalizer()
{
	sub_pulse = nh.subscribe("pulse", 2, &ICPLocalizer::pulseCallback, this);
	sub_gps = nh.subscribe("/gps_filtered", 2, &ICPLocalizer::gpsCallback, this);
	sub_yaw = nh.subscribe("/yaw_filtered", 2, &ICPLocalizer::yawCallback, this);

	ros::MultiThreadedSpinner spinner(4);
	spinner.spin();
}

void ICPLocalizer::pulseCallback(const std_msgs::Int8MultiArray::ConstPtr& input)
{
	cout<<"pulse0: "<<(int)input->data[0]<<"\tpulse1: "<<(int)input->data[1]<<endl;
}

void ICPLocalizer::gpsCallback(const sensor_msgs::NavSatFixConstPtr& input)
{
	cout<<"longitude: "<<input->longitude<<"\tlatitude: "<<input->latitude<<endl;
}

void ICPLocalizer::yawCallback(const std_msgs::Float64::ConstPtr& input)
{
	cout<<"yaw: "<<input->data<<endl;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "icp_localizer");

	ICPLocalizer localizer;

	return 0;
}
