#include "ros/ros.h"
#include <pcl/common/common_headers.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <cloud_operation.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int8MultiArray.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>


//0.0210386;
const double ODOMETRY_FACTOR = 0.0210386;

ros::Publisher pub_scan;
ros::Publisher pub_correct;
ros::Publisher pub_imu_pitch;

double curr_odom = 0.0, last_odom = 0.0;
bool rece_imu = false;
bool is_inited = false;
double turn_t = 0.0;
double curr_turn = 0.0, last_turn = 0.0;
double cloud_t = 0.0, last_cloud_t = 0.0;
double pose_x=0.0, pose_y=0.0, last_pose_x=0.0, last_pose_y=0.0;
double imu_odom=0.0;
double imu_pitch=0.0;
std::vector<int> fifo_size;
pcl::PointCloud<pcl::PointXYZRGB> cloud_sum;

int cal_angle(const pcl::PointXYZI pt)
{
    int angle = atan2(pt.y, pt.x)*180/M_PI;
    angle = angle < 0 ? angle += 360 : angle;
    return angle;
}

std::vector<pcl::PointCloud<pcl::PointXYZI> > get_sub_array(std::vector<pcl::PointCloud<pcl::PointXYZI> > input, int begin, int end)
{
    if(begin < 0 || end > input.size()-1) std::cout<<"error: function get_sub_array, out of index!"<<std::endl;
    std::vector<pcl::PointCloud<pcl::PointXYZI> > sub_array;
    for(int i=begin; i<end; i++)
        sub_array.push_back(input[i]);
    return sub_array;
}


std::vector<pcl::PointCloud<pcl::PointXYZI> > to_cir_array(pcl::PointCloud<pcl::PointXYZI> input)
{
    std::vector<pcl::PointCloud<pcl::PointXYZI> > cir_array;
    cir_array.clear();
    cir_array.resize(500);
    if(input.points.empty())
    {
        std::cout<<"warning: function to_cir_array() input empty cloud"<<std::endl;
        return cir_array;
    }

    int ang_offset = cal_angle(input.points[0]);
    int max_ang_idx = 360;

    for(int i=0; i<input.points.size(); i++ )
    {
        int ang_idx = cal_angle(input.points[i]);
        int _offset = ang_idx - ang_offset;
        int plus_360;
        if(_offset < 0) plus_360 = _offset + 360;
        else plus_360 = _offset;
        int reverse = plus_360 ? 360 - plus_360 : plus_360;
        if(i>input.points.size()/2 && reverse < 90) reverse+=360;
        if(max_ang_idx < reverse) max_ang_idx = reverse;

        cir_array[reverse].push_back(input.points[i]);
    }

    cir_array.resize(max_ang_idx+1);
    return cir_array;
}

std::vector<pcl::PointCloud<pcl::PointXYZI> > to_cir_array(std::vector<pcl::PointCloud<pcl::PointXYZI> > input)
{
    std::vector<std::vector<pcl::PointCloud<pcl::PointXYZI> > > temp_array;
    std::vector<pcl::PointCloud<pcl::PointXYZI> > ret_array;
    int max_size = 360;

    for(int i=0; i<input.size(); i++)
    {
        std::vector<pcl::PointCloud<pcl::PointXYZI> > cir_array = to_cir_array(input[i]);
        if(max_size<cir_array.size()) max_size = cir_array.size();
        temp_array.push_back(cir_array);
        std::cout<<"i: "<<i<<" size: "<<cir_array.size()<<std::endl;
    }
    ret_array.resize(max_size);
    for(int i=0; i<temp_array.size(); i++)
        for(int j=0; j<temp_array[i].size(); j++)
            ret_array[j] += temp_array[i][j];
    return ret_array;
}

template <typename PointT>
void cloud_transform(pcl::PointCloud<PointT>& cloud, double odom_inc, double yaw_inc)
{
    double inc_x = odom_inc * cos(yaw_inc);
    double inc_y = odom_inc * sin(yaw_inc);
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.translation() << -inc_x, -inc_y, 0.0;
    transform.rotate(Eigen::AngleAxisf(-yaw_inc, Eigen::Vector3f::UnitZ()));
    pcl::transformPointCloud(cloud, cloud, transform);
}

pcl::PointCloud<pcl::PointXYZRGB> to_color_cloud( std::vector<pcl::PointCloud<pcl::PointXYZI> > input )
{
    pcl::PointCloud<pcl::PointXYZRGB> color_cloud;
    color_cloud.clear();
    for (int i = 0; i < input.size()-1; ++i)
    {
        std::vector<pcl::PointCloud<pcl::PointXYZI> > sub_array = get_sub_array(input,i,i+1);
        pcl::PointCloud<pcl::PointXYZRGB> color = cloud_operation::arrayToColorCloud(sub_array,i%6);
        color_cloud += color;
    }
    return color_cloud;
}


void pointCloudCallback(const VPointCloud::ConstPtr &inMsg)
{
    if(!is_inited)
    {
        cloud_t = (double) ros::Time::now().toSec();
        last_cloud_t = cloud_t;
        is_inited = true;
        last_turn = curr_turn;
        last_odom = curr_odom;
        return;
    }

    double yaw_inc = (curr_turn - last_turn);
    double odom_inc = (curr_odom - last_odom);

//    imu_odom = odom_inc;
    odom_inc = imu_odom;

    last_odom = curr_odom;
    last_turn = curr_turn;

    cloud_t = inMsg->header.stamp/(1e6);
    double time_inc = cloud_t - last_cloud_t;
    last_cloud_t = cloud_t;

    std::vector<pcl::PointCloud<pcl::PointXYZI> > cloud_array = cloud_operation::cloudToArray(inMsg, 16);

    pcl::PointCloud<pcl::PointXYZI> sing_scan = cloud_operation::getOneRing(cloud_array, 5);
//    std::vector<pcl::PointCloud<pcl::PointXYZI> > cir_array = to_cir_array(sing_scan);
    std::vector<pcl::PointCloud<pcl::PointXYZI> > cir_array = to_cir_array(cloud_array);


    double yaw_inc_step = yaw_inc/cir_array.size();
    double odom_inc_step = odom_inc/cir_array.size();
    for(int i=0; i<cir_array.size(); i++)
    {
//        std::cout<<"cir_array_size: "<<cir_array.size()<<std::endl;
        if(cir_array[i].points.size())
            cloud_transform(cir_array[i], (cir_array.size()-i)*odom_inc_step, (cir_array.size()-i)*yaw_inc_step);
    }

    pcl::PointCloud<pcl::PointXYZI> cloud_corrected = cloud_operation::arrayToCloudXYZI(cir_array);

    std::vector<pcl::PointCloud<pcl::PointXYZI> > sub_array1 = get_sub_array(cir_array,0,20);
    std::vector<pcl::PointCloud<pcl::PointXYZI> > sub_array3 = get_sub_array(cir_array,20,360);
    std::vector<pcl::PointCloud<pcl::PointXYZI> > sub_array2 = get_sub_array(cir_array,360,cir_array.size()-1);
    pcl::PointCloud<pcl::PointXYZRGB> color_cloud1 = cloud_operation::arrayToColorCloud(sub_array1,0);
    pcl::PointCloud<pcl::PointXYZRGB> color_cloud2 = cloud_operation::arrayToColorCloud(sub_array2,3);
    pcl::PointCloud<pcl::PointXYZRGB> color_cloud3 = cloud_operation::arrayToColorCloud(sub_array3,2);
    pcl::PointCloud<pcl::PointXYZRGB> cloud_corrected2 = color_cloud1 + color_cloud2;
    cloud_corrected2 += color_cloud3;

    pcl::PointCloud<pcl::PointXYZRGB> color_cloud = to_color_cloud(cir_array);

    cloud_transform(cloud_sum,odom_inc, yaw_inc);
    fifo_size.push_back(cloud_corrected2.size());
    cloud_sum = cloud_sum + cloud_corrected2;

    if (fifo_size.size()>30)
    {
        cloud_sum.erase(cloud_sum.begin(), cloud_sum.begin() + fifo_size[0]);
        fifo_size.erase(fifo_size.begin());
    }


    sensor_msgs::PointCloud2 cloud_to_pub;
    sing_scan.header.frame_id = "base_link";
    pcl::toROSMsg(sing_scan,cloud_to_pub);
    pub_scan.publish(cloud_to_pub);

    cloud_corrected.header.frame_id = "base_link";
    cloud_corrected.header.stamp = inMsg->header.stamp;
    pcl::toROSMsg(cloud_corrected, cloud_to_pub);
    pub_correct.publish(cloud_to_pub);

}

void pulseCallback(const std_msgs::Int8MultiArray::ConstPtr& input)
{
    int pulse_inc = (int)input->data[0] + (int)input->data[1];
    float odom_inc = pulse_inc * ODOMETRY_FACTOR / 2.0;
    curr_odom += odom_inc;
}

void imuCallback(const sensor_msgs::Imu::ConstPtr& input)
{
    if(!rece_imu)
    {
        turn_t = (double) ros::Time::now().toSec();
        rece_imu = true;
        return;
    }
    double dt = (double) ros::Time::now().toSec() - turn_t;
    double yaw_inc = (double) (input->angular_velocity.z) * dt;
    curr_turn += yaw_inc;

    double pitch_inc = (double) (input->angular_velocity.y) * dt;
    imu_pitch += pitch_inc;
    std_msgs::Float64 pitch_deg;
    pitch_deg.data = imu_pitch * 180.0 / M_PI;
    pub_imu_pitch.publish(pitch_deg);

    turn_t = (double) ros::Time::now().toSec();
}

void rtkPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& input)
{
    pose_x = input->pose.position.x;
    pose_y = input->pose.position.y;

    double dist = std::sqrt( std::pow(pose_x-last_pose_x, 2) + std::pow(pose_y-last_pose_y, 2) );
    imu_odom = dist;
    std::cout<<"x:"<<input->pose.position.x<<"\ty: "<<input->pose.position.y<<"\tdist: "<<dist<<"\timu_odom: "<<imu_odom<<std::endl;

    last_pose_x = pose_x;
    last_pose_y = pose_y;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "motion_correct_node");
    ros::NodeHandle nh;
    ros::Subscriber sub_front_curb = nh.subscribe("points_in_base_link",2,pointCloudCallback);
    ros::Subscriber sub_pulse = nh.subscribe("pulse", 2, pulseCallback);
    ros::Subscriber sub_imu = nh.subscribe("imu_torso/xsens/data", 2, imuCallback);
    ros::Subscriber sub_rtk = nh.subscribe("fusion_pose", 2, rtkPoseCallback);
    pub_scan = nh.advertise<sensor_msgs::PointCloud2>("single_scan", 2);
    pub_correct = nh.advertise<sensor_msgs::PointCloud2>("corrected_scan", 2);
    pub_imu_pitch = nh.advertise<std_msgs::Float64>("pitch_deg",2);
    cloud_sum.clear();
    fifo_size.clear();

    ros::spin();
    return 0;
}