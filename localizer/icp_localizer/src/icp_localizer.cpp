#include "icp_localizer.h"

using namespace std;



ICPLocalizer::ICPLocalizer()
{
	sub_map = nh.subscribe("point_map",2, &ICPLocalizer::mapCallback, this);
	sub_points = nh.subscribe("feature_points_sum",2, &ICPLocalizer::featurePointsCallback, this);
	sub_pose = nh.subscribe("estimate_pose", 2, &ICPLocalizer::poseCallback, this);
	sub_pulse = nh.subscribe("pulse", 2, &ICPLocalizer::pulseCallback, this);
	sub_imu = nh.subscribe("/imu_torso/xsens/data",2,&ICPLocalizer::imuCallback,this);\
	sub_rtk = nh.subscribe("rtk_pose", 2, &ICPLocalizer::rtkCallback, this);

	pub_icp_pose = nh.advertise<geometry_msgs::PoseStamped>("icp_pose", 2);
//	pub_pose_fixed = nh.advertise<geometry_msgs::PoseStamped>("fix")
	
	maximum_iterations = 500;
	transformation_epsilon = 0.01;
	max_correspondence_distance = 2.0;
	euclidean_fitness_epsilon = 0.1;
	ransac_outlier_rejection_threshold = 1.0;
	fitness_score = 0;

	map_loaded = false;
	is_inited = false;
	rece_imu = false;
	is_drift = false;
	curr_turn = 0.0;
	last_turn = 0.0;
	curr_yaw = 0.0;
	curr_odom = 0.0;
	

	ros::MultiThreadedSpinner spinner(8);
	spinner.spin();
}

void ICPLocalizer::mapCallback(const sensor_msgs::PointCloud2::ConstPtr& input)
{
	pcl::PointCloud<pcl::PointXYZ> map;
    pcl::fromROSMsg(*input, map);

    pcl::PointCloud<pcl::PointXYZ>::Ptr map_ptr(new pcl::PointCloud<pcl::PointXYZ>(map));

    icp.setInputTarget(map_ptr);
//    std::cout << "setInputTarget finished." << std::endl;

    map_loaded = true;
}

void ICPLocalizer::featurePointsCallback(const sensor_msgs::PointCloud2::ConstPtr& input)
{
//初始化定位
	if(is_inited == false)
	{
		cout<<"init!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
		cout<<"init!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
		cout<<"init!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
		
		
		curr_pose = rtk_pose;
		cout<<"init_x: "<<curr_pose.pose.position.x<<" "<<"last_y: "<<curr_pose.pose.position.y<<" "<<"last_z: "<<curr_pose.pose.position.z<<" "<<"last_yaw: "<<curr_yaw<<endl;
		last_pose = rtk_pose;
		dr_pose = rtk_pose;
		last_turn = curr_turn;
		last_odom = curr_odom;
		
		curr_pose.header.frame_id = "icp_pose";
		pub_icp_pose.publish(curr_pose);
		
		is_inited = true;
		return;
	}

//计算位移增量与角度增量
	odom_inc = curr_odom - last_odom;
	turn_inc = curr_turn - last_turn;
	last_odom = curr_odom;
	last_turn = curr_turn;
	
	double inc_x = odom_inc * cos( curr_yaw );
	double inc_y = odom_inc * sin( curr_yaw );
	
	dr_pose.pose.position.x += inc_x;
	dr_pose.pose.position.y += inc_y;
	dr_pose.pose.position.z = 0;
	
	tf::Quaternion dr_tfq;
	dr_tfq.setRPY(0, 0, curr_yaw);
	geometry_msgs::Quaternion dr_quat;
	tf::quaternionTFToMsg(dr_tfq, dr_quat);
	dr_pose.pose.orientation = dr_quat;
	
	
//上帧定位＋航位推算＝预测初始匹配位姿
	//pose
	pred_pose.pose.position.x = last_pose.pose.position.x + inc_x;
	pred_pose.pose.position.y = last_pose.pose.position.y + inc_y;
	pred_pose.pose.position.z = last_pose.pose.position.z = 0;
	
	//orientation
	double last_roll, last_pitch, last_yaw;
	tf::Quaternion last_q;
    tf::quaternionMsgToTF(last_pose.pose.orientation, last_q);
    tf::Matrix3x3(last_q).getRPY(last_roll, last_pitch, last_yaw);
    
	double pred_roll, pred_pitch, pred_yaw;
	pred_roll = last_roll;
	pred_pitch = last_pitch;
	pred_yaw = last_yaw + turn_inc;
  	
	tf::Quaternion pred_tfq;
	pred_tfq.setRPY(0, 0, pred_yaw);
	geometry_msgs::Quaternion pred_quat;
	tf::quaternionTFToMsg(pred_tfq, pred_quat);
	pred_pose.pose.orientation = pred_quat;
	
	
	//init_guess
	Eigen::Translation3f init_translation(inc_x, inc_y, 0);
	Eigen::AngleAxisf init_rotation_x(pred_roll, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf init_rotation_y(pred_pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf init_rotation_z(turn_inc, Eigen::Vector3f::UnitZ());
    Eigen::Matrix4f init_guess = (init_translation * init_rotation_z ).matrix();
    
	
//icp点云匹配
	pcl::PointCloud<pcl::PointXYZ> feature_points;
    pcl::fromROSMsg(*input, feature_points);
    pcl::PointCloud<pcl::PointXYZ>::Ptr feature_points_ptr(new pcl::PointCloud<pcl::PointXYZ>(feature_points));
    
    icp.setInputSource(feature_points_ptr);
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    
    icp.setMaximumIterations(maximum_iterations);
    icp.setTransformationEpsilon(transformation_epsilon);
    icp.setMaxCorrespondenceDistance(max_correspondence_distance);
    icp.setEuclideanFitnessEpsilon(euclidean_fitness_epsilon);
    icp.setRANSACOutlierRejectionThreshold(ransac_outlier_rejection_threshold);

	icp.align(*output_cloud);  
	
	fitness_score = icp.getFitnessScore();
	Eigen::Matrix4f t = icp.getFinalTransformation();
	
	fix_matrix = t;
	tf::Matrix3x3 mat_l;  // localizer
	mat_l.setValue(static_cast<double>(t(0, 0)), static_cast<double>(t(0, 1)), static_cast<double>(t(0, 2)),
	               static_cast<double>(t(1, 0)), static_cast<double>(t(1, 1)), static_cast<double>(t(1, 2)),
	               static_cast<double>(t(2, 0)), static_cast<double>(t(2, 1)), static_cast<double>(t(2, 2)));

	double fix_x,fix_y,fix_z;
	double fix_roll,fix_pitch,fix_yaw;
	// Update localizer_pose
	fix_x = t(0, 3);
	fix_y = t(1, 3);
	fix_z = 0;
	
	mat_l.getRPY(fix_roll, fix_pitch, fix_yaw, 1);
	
	
//预测初始匹配位姿×修正矩阵＝当前定位结果
	curr_pose.pose.position.x = pred_pose.pose.position.x + fix_x;
	curr_pose.pose.position.y = pred_pose.pose.position.y + fix_y;	
	curr_pose.pose.position.z = pred_pose.pose.position.z + fix_z;
	
//	curr_yaw = pred_yaw + fix_yaw;
//	curr_pose.pose.orientation = tf::createQuaternionMsgFromYaw(pred_yaw + fix_yaw);
	tf::Quaternion cur_tfq;
	cur_tfq.setRPY(0, 0, curr_yaw);
	geometry_msgs::Quaternion cur_quat;
	tf::quaternionTFToMsg(cur_tfq, cur_quat);
	curr_pose.pose.orientation = cur_quat;	
	
//	if( fabs(fix_x) > 2 || fabs(fix_y) > 2)	curr_pose.pose.position = pred_pose.pose.position;
	if(fabs(fix_x) < 0.5 && fabs(fix_y) < 0.5 && fitness_score < 0.03)
		dr_pose = curr_pose;
	else
		curr_pose = dr_pose;
		
//	if( is_drift == false && fitness_score < 0.1 )
//	{
//		dr_pose = curr_pose;
//		is_drift = false;
//	}
//	
//	
//	if( is_drift == false && fitness_score > 1 )
//	{
//		curr_pose = dr_pose;
//		is_drift = true;
//	}
//	
//	if( is_drift == true && fitness_score < 20 )
//	{
//		dr_pose = curr_pose;
//		is_drift = false;
//	}
//	else if( is_drift == true )
//	{
//		curr_pose = dr_pose;
//	}

	
	

//上帧定位＝当前定位
	last_pose.pose.position = curr_pose.pose.position;	
	last_pose.pose.orientation = curr_pose.pose.orientation; 
	
	
//发布定位	
	curr_pose.header.frame_id = "icp_pose";
	pub_icp_pose.publish(curr_pose);
	
	static tf::TransformBroadcaster br_filtered_curr;
    tf::Transform transform_filtered;  
    tf::Quaternion q_filtered(curr_pose.pose.orientation.x,curr_pose.pose.orientation.y,curr_pose.pose.orientation.z,curr_pose.pose.orientation.w);

    transform_filtered.setOrigin(tf::Vector3(curr_pose.pose.position.x,curr_pose.pose.position.y,curr_pose.pose.position.z));

    transform_filtered.setRotation(q_filtered);
    br_filtered_curr.sendTransform( tf::StampedTransform(transform_filtered, ros::Time::now(), "map" , "icp_pose"));
	
	cout<<"pred_x: "<<pred_pose.pose.position.x<<"\t"<<"pred_y: "<<pred_pose.pose.position.y<<"\t"<<"pred_z: "<<pred_pose.pose.position.z<<"\t"<<"pred_yaw: "<<pred_yaw<<endl;
	cout<<"curr_x: "<<curr_pose.pose.position.x<<"\t"<<"curr_y: "<<curr_pose.pose.position.y<<"\t"<<"curr_z : "<<curr_pose.pose.position.z<<"\t"<<"curr_yaw: "<<curr_yaw<<endl;
	cout<<"rtk_x : "<<rtk_pose.pose.position.x<<"\t"<<"rtk_y : "<<rtk_pose.pose.position.y<<"\t"<<"rtk_z : "<<rtk_pose.pose.position.z<<"\t"<<"rtk_yaw : "<<curr_yaw<<endl;
	cout<<"fix_x : "<<fix_x<<"\t"<<"fix_y: "<<fix_y<<"\t"<<"fix_z : "<<fix_z<<"\t"<<"fix_yaw : "<<fix_yaw<<endl;
	cout<<"inc_x : "<<inc_x<<"\t"<<"inc_y: "<<fix_y<<"\t"<<"inc_z : "<<fix_z<<"\t"<<"inc_yaw : "<<turn_inc<<endl;
	cout<<"fitness_score: "<<fitness_score<<endl;

	cout<<endl;

}

void ICPLocalizer::pulseCallback(const std_msgs::Int8MultiArray::ConstPtr& input)
{
	int pulse_inc = (int)input->data[0] + (int)input->data[0];
	float odom_inc = pulse_inc * ODOMETRY_FACTOR / 2.0;
	curr_odom += odom_inc;
//	cout<<"curr_odom: "<<curr_odom<<endl;
}


void ICPLocalizer::imuCallback(const sensor_msgs::Imu::ConstPtr& input)
{
	if(!rece_imu)
	{
		turn_t = (double) ros::Time::now().toSec();
		rece_imu = true;
		return;
	}
	double dt = (double) ros::Time::now().toSec() - turn_t;
	double yaw_inc = double(input->angular_velocity.z) * dt;
	curr_turn += yaw_inc;
	
	turn_t = (double) ros::Time::now().toSec();
//	
//	cout<<"dt: "<<dt<<endl;
//	cout<<"yaw_vel: "<<input->angular_velocity.z<<endl;
//	cout<<"yaw_inc: "<<yaw_inc<<endl;
//	cout<<"curr_turn: "<<curr_turn<<endl;
}


void ICPLocalizer::rtkCallback(const geometry_msgs::PoseStamped::ConstPtr& input)
{
	rtk_pose = *input;
	

	if(!is_inited)
	{
		curr_pose = rtk_pose;
		curr_pose.header.frame_id = "icp_pose";
		pub_icp_pose.publish(curr_pose);

		
		static tf::TransformBroadcaster br_filtered_rtk;
		tf::Transform transform_filtered;  
		tf::Quaternion q_filtered(curr_pose.pose.orientation.x,curr_pose.pose.orientation.y,curr_pose.pose.orientation.z,curr_pose.pose.orientation.w);

		transform_filtered.setOrigin(tf::Vector3(curr_pose.pose.position.x,curr_pose.pose.position.y,curr_pose.pose.position.z));

		transform_filtered.setRotation(q_filtered);
		br_filtered_rtk.sendTransform( tf::StampedTransform(transform_filtered, input->header.stamp, "map" , "icp_pose"));
	}
}

void ICPLocalizer::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& input)
{
//	cout<<fix_matrix<<endl;	

	geometry_msgs::PoseStamped pose_fixed;
	pose_fixed.pose.position.x = input->pose.position.x + fix_matrix(0, 3);
	pose_fixed.pose.position.y = input->pose.position.y + fix_matrix(1, 3);
	pose_fixed.pose.position.z = input->pose.position.z + fix_matrix(2, 3);
	pose_fixed.pose.orientation = input->pose.orientation;
	
	
	
	double roll, pitch, yaw;
	tf::Quaternion q;
    tf::quaternionMsgToTF(input->pose.orientation, q);
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    curr_yaw = yaw;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "icp_localizer");

	ICPLocalizer localizer;

	return 0;
}
