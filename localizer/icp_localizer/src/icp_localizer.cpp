#include "icp_localizer.h"


using namespace std;



ICPLocalizer::ICPLocalizer()
{
	sub_map = nh.subscribe("point_map",2, &ICPLocalizer::mapCallback, this);
	sub_points = nh.subscribe("feature_points_sum",2, &ICPLocalizer::featurePointsCallback, this);
	sub_pose = nh.subscribe("estimate_pose", 2, &ICPLocalizer::poseCallback, this);
	sub_pulse = nh.subscribe("pulse", 2, &ICPLocalizer::pulseCallback, this);
	sub_imu = nh.subscribe("/imu_torso/xsens/data",2,&ICPLocalizer::imuCallback,this);
	sub_rtk = nh.subscribe("rtk_pose", 2, &ICPLocalizer::rtkCallback, this);
	
	sub_front_curb = nh.subscribe("front_curb_raw",2, &ICPLocalizer::frontCurbCallback, this);
	sub_sign = nh.subscribe("sign_points",2, &ICPLocalizer::signCallback, this);

	pub_icp_pose = nh.advertise<geometry_msgs::PoseStamped>("icp_pose", 2);
	pub_lcp_pose = nh.advertise<geometry_msgs::PoseStamped>("lcp_pose", 2);
	pub_cloud_dr = nh.advertise<sensor_msgs::PointCloud2>("cloud_dr_sum", 2);
	pub_cloud_sample = nh.advertise<sensor_msgs::PointCloud2>("feature_points_sample", 2);

	cb = boost::bind(&ICPLocalizer::configCallback, this, _1, _2);
	dr_srv.setCallback(cb);
	
	maximum_iterations = 500;
	transformation_epsilon = 0.01;
	max_correspondence_distance = 4.0;
	euclidean_fitness_epsilon = 0.2;
	ransac_outlier_rejection_threshold = 3.0;
	fitness_score = 0;

	map_loaded = false;
	is_inited = false;
	rece_imu = false;
	is_drift = false;
	save_point = false;
	save_cnt = 0;
	curr_turn = 0.0;
	last_turn = 0.0;
	curr_yaw = 0.0;
	curr_odom = 0.0;
	last_size = 0;
	
	cloud_dr_sum.clear();
	cloud_dr_new.clear();
	
	PointXYZO po;
	po.x = -20;
	po.y = 0;
	po.z = 0;
	cloud_dr_sum.push_back(po);
	

	ros::MultiThreadedSpinner spinner(8);
	spinner.spin();
}

void ICPLocalizer::mapCallback(const sensor_msgs::PointCloud2::ConstPtr& input)
{
	pcl::PointCloud<pcl::PointXYZ> map;
    pcl::fromROSMsg(*input, map);
    
//    for(long int i=0; i<map.points.size(); i++)
//    	map.points[i].z = 0.0;

    pcl::PointCloud<pcl::PointXYZ>::Ptr map_ptr(new pcl::PointCloud<pcl::PointXYZ>(map));

    icp.setInputTarget(map_ptr);
//    std::cout << "setInputTarget finished." << std::endl;

	if(save_point)
		pcl::io::savePCDFileASCII ("/home/wlh/map/dbg-icp/map.pcd", map);
    map_loaded = true;
    
    
    if(map_ptr->empty())	return;
    pcl::search::KdTree<pcl::PointXYZ> kdtree;
	kdtree.setInputCloud(map_ptr);
	

	pcl::PointXYZ point;
	point.x = rtk_pose.pose.position.x;
	point.y = rtk_pose.pose.position.y;
	point.z = rtk_pose.pose.position.z;
	pcl::PointIndices::Ptr pointIndices(new pcl::PointIndices);
	std::vector<float> squaredDistances(5);
	if (kdtree.radiusSearch(point, 20.0 , pointIndices->indices , squaredDistances) > 0)
	{
		pcl::ExtractIndices<pcl::PointXYZ> extract;
		extract.setInputCloud(map_ptr);
		extract.setIndices(pointIndices);
		extract.setNegative(false);
		extract.filter(cut_map);	
	}
	
	sign_map.clear();
    for(long int i=0; i<map.points.size(); i++)
	{
		if( map.points[i].z>3.0 )
			sign_map.push_back(map.points[i]);
	}
		
	sign_map.header.frame_id = "map";
	pcl_conversions::toPCL(ros::Time::now(), sign_map.header.stamp);
	sensor_msgs::PointCloud2 cloud_to_pub;
	pcl::toROSMsg(sign_map, cloud_to_pub);
	pub_cloud_dr.publish(cloud_to_pub);
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


	
//icp点云匹配
	pcl::PointCloud<pcl::PointXYZ> feature_points;
    pcl::fromROSMsg(*input, feature_points);
    pcl::PointCloud<pcl::PointXYZ>::Ptr feature_points_ptr(new pcl::PointCloud<pcl::PointXYZ>(feature_points));
    
	if(save_point)
	{
		std::stringstream ss;
		ss<<save_cnt++;
        pcl::io::savePCDFileASCII ("/home/wlh/map/dbg-icp/"+ss.str()+".pcd", feature_points);
	}
    
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
	

	if( feature_points.points.size() == last_size )
	{
		curr_pose = pred_pose;
		cout<<"feature_points not update"<<endl;
	}
	else
		pred_pose = curr_pose;
	
	if(fitness_score<0.1)
		dr_pose = curr_pose;
	if(fitness_score>1)
		curr_pose = dr_pose;
		
//	if( abs(curr_pose.pose.position.x-dr_pose.pose.position.x)<1 && abs(curr_pose.pose.position.y-dr_pose.pose.position.y)<1 )
//		dr_pose = curr_pose;
//	else
//		curr_pose = dr_pose;
		
//最近邻距离修正
	if(!front_curb.points.empty() && !cut_map.empty())
	{
		pcl::search::KdTree<pcl::PointXYZ> kdt2;
		kdt2.setInputCloud(cut_map.makeShared());
		
		Eigen::Affine3f trans = Eigen::Affine3f::Identity();
		trans.translation() << curr_pose.pose.position.x, curr_pose.pose.position.y, 0;
		trans.rotate(Eigen::AngleAxisf(curr_yaw, Eigen::Vector3f::UnitZ()));
		pcl::PointCloud<pcl::PointXYZ> temp_cloud;
		pcl::transformPointCloud(front_curb, temp_cloud, trans);	
		
		
		double distance_sum = 0.0;
		double distance_mean = 0.0;

		for(int i=0; i<temp_cloud.points.size(); i++)
		{
			pcl::PointXYZ pt;
			pt.x = temp_cloud.points[i].x;
			pt.y = temp_cloud.points[i].y;
			pt.z = temp_cloud.points[i].z;

			std::vector<int> ptIndices(1);
			std::vector<float> squared(1);
			if (kdt2.nearestKSearch(pt, 1, ptIndices, squared) > 0)
			{			
				for (size_t i = 0; i < ptIndices.size(); ++i)
					distance_sum += sqrt(squared[i]);
			}
		}
		
		distance_mean = distance_sum / (temp_cloud.points.size()*1.0) ;
//		cout<<"distance_mean: "<<distance_mean<<endl;
		
		
		double lcp_x = 0.0;
		double lcp_y = 0.0;
		lcp_x = distance_mean * cos( curr_yaw + M_PI/2.0 );
		lcp_y = distance_mean * sin( curr_yaw + M_PI/2.0 );
		
		Eigen::Affine3f trans3 = Eigen::Affine3f::Identity();
		trans3.translation() << curr_pose.pose.position.x + lcp_x , curr_pose.pose.position.y + lcp_y , 0;
		trans3.rotate(Eigen::AngleAxisf(curr_yaw, Eigen::Vector3f::UnitZ()));
		temp_cloud.clear();
		pcl::transformPointCloud(front_curb, temp_cloud, trans3);	
		
		double distance_sum3 = 0.0;
		double distance_mean3 = 0.0;
		
		for(int i=0; i<temp_cloud.points.size(); i++)
		{
			pcl::PointXYZ pt;
			pt.x = temp_cloud.points[i].x;
			pt.y = temp_cloud.points[i].y;
			pt.z = temp_cloud.points[i].z;

			std::vector<int> ptIndices3(1);
			std::vector<float> squared3(1);
			if (kdt2.nearestKSearch(pt, 1, ptIndices3, squared3) > 0)
			{			
				for (size_t i = 0; i < ptIndices3.size(); ++i)
					distance_sum3 += sqrt(squared3[i]);
			}
		}
		distance_mean3 = distance_sum3 / (temp_cloud.points.size()*1.0) ;
//		cout<<"distance_mean3: "<<distance_mean3<<endl;
		
		
		Eigen::Affine3f trans4 = Eigen::Affine3f::Identity();
		trans4.translation() << curr_pose.pose.position.x - lcp_x , curr_pose.pose.position.y - lcp_y , 0;
		trans4.rotate(Eigen::AngleAxisf(curr_yaw, Eigen::Vector3f::UnitZ()));
		temp_cloud.clear();
		pcl::transformPointCloud(front_curb, temp_cloud, trans4);	
		
		double distance_sum4 = 0.0;
		double distance_mean4 = 0.0;
		
		for(int i=0; i<temp_cloud.points.size(); i++)
		{
			pcl::PointXYZ pt;
			pt.x = temp_cloud.points[i].x;
			pt.y = temp_cloud.points[i].y;
			pt.z = temp_cloud.points[i].z;

			std::vector<int> ptIndices4(1);
			std::vector<float> squared4(1);
			if (kdt2.nearestKSearch(pt, 1, ptIndices4, squared4) > 0)
			{			
				for (size_t i = 0; i < ptIndices4.size(); ++i)
					distance_sum4 += sqrt(squared4[i]);
			}
		}
		distance_mean4 = distance_sum4 / (temp_cloud.points.size()*1.0) ;
//		cout<<"distance_mean4: "<<distance_mean4<<endl;
		
		if(distance_mean3 < distance_mean4)
		{
			curr_pose.pose.position.x += lcp_x;
			curr_pose.pose.position.y += lcp_y;
		}
		else
		{
			curr_pose.pose.position.x -= lcp_x;
			curr_pose.pose.position.y -= lcp_y;
		}
		
		
		front_curb.clear();
	}	
	
	if(!sign_points.points.empty() && !sign_map.empty())
	{
		pcl::search::KdTree<pcl::PointXYZ> kdt2;
		kdt2.setInputCloud(sign_map.makeShared());
		
		Eigen::Affine3f trans = Eigen::Affine3f::Identity();
		trans.translation() << curr_pose.pose.position.x, curr_pose.pose.position.y, 0;
		trans.rotate(Eigen::AngleAxisf(curr_yaw, Eigen::Vector3f::UnitZ()));
		pcl::PointCloud<pcl::PointXYZ> temp_cloud;
		pcl::transformPointCloud(sign_points, temp_cloud, trans);
		
		double distance_sum = 0.0;
		double distance_mean = 0.0;

		for(int i=0; i<temp_cloud.points.size(); i++)
		{
			pcl::PointXYZ pt;
			pt.x = temp_cloud.points[i].x;
			pt.y = temp_cloud.points[i].y;
			pt.z = temp_cloud.points[i].z;

			std::vector<int> ptIndices(1);
			std::vector<float> squared(1);
			if (kdt2.nearestKSearch(pt, 1, ptIndices, squared) > 0)
			{			
				for (size_t i = 0; i < ptIndices.size(); ++i)
					distance_sum += sqrt(squared[i]);
			}
		}
		
		distance_mean = distance_sum / (temp_cloud.points.size()*1.0+0.001) ;
		cout<<"distance_mean5: "<<distance_mean<<endl;
		
		
		double lcp_x = 0.0;
		double lcp_y = 0.0;
		lcp_x = distance_mean * cos( curr_yaw );
		lcp_y = distance_mean * sin( curr_yaw );
		
		Eigen::Affine3f trans3 = Eigen::Affine3f::Identity();
		trans3.translation() << curr_pose.pose.position.x + lcp_x , curr_pose.pose.position.y + lcp_y , 0;
		trans3.rotate(Eigen::AngleAxisf(curr_yaw, Eigen::Vector3f::UnitZ()));
		temp_cloud.clear();
		pcl::transformPointCloud(sign_points, temp_cloud, trans3);	
		
		double distance_sum3 = 0.0;
		double distance_mean3 = 0.0;
		double nearest_cnt3 = 0;
		
		for(int i=0; i<temp_cloud.points.size(); i++)
		{
			pcl::PointXYZ pt;
			pt.x = temp_cloud.points[i].x;
			pt.y = temp_cloud.points[i].y;
			pt.z = temp_cloud.points[i].z;

			std::vector<int> ptIndices3(1);
			std::vector<float> squared3(1);
			if (kdt2.nearestKSearch(pt, 1, ptIndices3, squared3) > 0)
			{			
				for (size_t i = 0; i < ptIndices3.size(); ++i)
					distance_sum3 += sqrt(squared3[i]);
			}
		}
		distance_mean3 = distance_sum3 / (temp_cloud.points.size()*1.0+0.001) ;
		cout<<"distance_mean6: "<<distance_mean3<<endl;
		
		
		Eigen::Affine3f trans4 = Eigen::Affine3f::Identity();
		trans4.translation() << curr_pose.pose.position.x - lcp_x , curr_pose.pose.position.y - lcp_y , 0;
		trans4.rotate(Eigen::AngleAxisf(curr_yaw, Eigen::Vector3f::UnitZ()));
		temp_cloud.clear();
		pcl::transformPointCloud(sign_points, temp_cloud, trans4);	
		
		double distance_sum4 = 0.0;
		double distance_mean4 = 0.0;
		double nearest_cnt4 = 0;
		
		for(int i=0; i<temp_cloud.points.size(); i++)
		{
			pcl::PointXYZ pt;
			pt.x = temp_cloud.points[i].x;
			pt.y = temp_cloud.points[i].y;
			pt.z = temp_cloud.points[i].z;

			std::vector<int> ptIndices4(1);
			std::vector<float> squared4(1);
			if (kdt2.nearestKSearch(pt, 1, ptIndices4, squared4) > 0)
			{			
				for (size_t i = 0; i < ptIndices4.size(); ++i)
					distance_sum4 += sqrt(squared4[i]);
			}
			
//			kdt2.radiusSearch(pt, 0.5, ptIndices4, squared4)
			
		}
		distance_mean4 = distance_sum4 / (temp_cloud.points.size()*1.0+0.001) ;
		cout<<"distance_mean7: "<<distance_mean4<<endl;
		
		if(distance_mean3 < distance_mean4)
		{
			curr_pose.pose.position.x += lcp_x;
			curr_pose.pose.position.y += lcp_y;
		}
		else
		{
			curr_pose.pose.position.x -= lcp_x;
			curr_pose.pose.position.y -= lcp_y;
		}
	
	
		sign_points.clear();
	}
	
	

//上帧定位＝当前定位
	last_pose.pose.position = curr_pose.pose.position;	
	last_pose.pose.orientation = curr_pose.pose.orientation; 
	
	last_size = feature_points.points.size();

	
//发布定位	
	curr_pose.header.frame_id = "icp_pose";
	pub_icp_pose.publish(curr_pose);
	
	static tf::TransformBroadcaster br_filtered_curr;
    tf::Transform transform_filtered;  
    tf::Quaternion q_filtered(curr_pose.pose.orientation.x,curr_pose.pose.orientation.y,curr_pose.pose.orientation.z,curr_pose.pose.orientation.w);
    transform_filtered.setOrigin(tf::Vector3(curr_pose.pose.position.x,curr_pose.pose.position.y,curr_pose.pose.position.z));
    transform_filtered.setRotation(q_filtered);
    br_filtered_curr.sendTransform( tf::StampedTransform(transform_filtered, ros::Time::now(), "map" , "icp_pose"));
    
//    static tf::TransformBroadcaster br_filtered_dr;
//    tf::Transform transform_dr;  
//    tf::Quaternion q_dr(dr_pose.pose.orientation.x,dr_pose.pose.orientation.y,dr_pose.pose.orientation.z,dr_pose.pose.orientation.w);
//    transform_dr.setOrigin(tf::Vector3(dr_pose.pose.position.x,dr_pose.pose.position.y,dr_pose.pose.position.z));
//    transform_dr.setRotation(q_dr);
//    br_filtered_dr.sendTransform( tf::StampedTransform(transform_dr, ros::Time::now(), "map" , "dr_pose"));
	
//	cout<<"pred_x: "<<pred_pose.pose.position.x<<"\t"<<"pred_y: "<<pred_pose.pose.position.y<<"\t"<<"pred_z: "<<pred_pose.pose.position.z<<"\t"<<"pred_yaw: "<<pred_yaw<<endl;
//	cout<<"curr_x: "<<curr_pose.pose.position.x<<"\t"<<"curr_y: "<<curr_pose.pose.position.y<<"\t"<<"curr_z : "<<curr_pose.pose.position.z<<"\t"<<"curr_yaw: "<<curr_yaw<<endl;
//	cout<<"rtk_x : "<<rtk_pose.pose.position.x<<"\t"<<"rtk_y : "<<rtk_pose.pose.position.y<<"\t"<<"rtk_z : "<<rtk_pose.pose.position.z<<"\t"<<"rtk_yaw : "<<curr_yaw<<endl;
//	cout<<"fix_x : "<<fix_x<<"\t"<<"fix_y: "<<fix_y<<"\t"<<"fix_z : "<<fix_z<<"\t"<<"fix_yaw : "<<fix_yaw<<endl;
//	cout<<"inc_x : "<<inc_x<<"\t"<<"inc_y: "<<inc_y<<"\t"<<"inc_z : "<<0<<"\t"<<"inc_yaw : "<<turn_inc<<endl;
//	cout<<"fitness_score: "<<fitness_score<<endl;

//	cout<<endl;

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

void ICPLocalizer::configCallback(icp_localizer::saveConfig &config, uint32_t level)
{
	save_point = config.save_point;
}

void ICPLocalizer::frontCurbCallback(const sensor_msgs::PointCloud2::ConstPtr& input)
{
    pcl::fromROSMsg(*input, front_curb);
    
}

void ICPLocalizer::signCallback(const sensor_msgs::PointCloud2::ConstPtr& input)
{
	pcl::fromROSMsg(*input, sign_points);
	
//	if(sign_points.points.empty())	return;
//	
//	LcpMatching<pcl::PointXYZ> LCP;
//	pcl::PointCloud<pcl::PointXYZ> cloud_trans;
//	
//	pcl::PointCloud<pcl::PointXYZ> cloud_sum;
//	cloud_sum.clear();
//	
//	Eigen::Affine3f trans = Eigen::Affine3f::Identity();
//	trans.translation() << curr_pose.pose.position.x, curr_pose.pose.position.y, 0;
//	trans.rotate(Eigen::AngleAxisf(curr_yaw, Eigen::Vector3f::UnitZ()));
//	pcl::PointCloud<pcl::PointXYZ> cloud_new;
//	cloud_new.clear();
//	pcl::transformPointCloud(sign_points, cloud_new, trans);
//	
//	cloud_trans = LCP.cloud_translation_step( cloud_new , curr_yaw , 5.0 );
//	cloud_sum += cloud_trans;
//	cloud_trans = LCP.cloud_translation_step( cloud_new , curr_yaw , -5.0 );
//	cloud_sum += cloud_trans;
//	cloud_trans = LCP.cloud_translation_step( cloud_new , curr_yaw , 10.0 );
//	cloud_sum += cloud_trans;
//	cloud_trans = LCP.cloud_translation_step( cloud_new , curr_yaw , -10.0 );
//	cloud_sum += cloud_trans;
//	
//	pcl::search::KdTree<pcl::PointXYZ> kdt2;
//	if(sign_map.empty())	return;
//	kdt2.setInputCloud(sign_map.makeShared());
//	for(int i=0; i<cloud_new.points.size(); i++)
//	{
//		pcl::PointXYZ pt;
//		pt.x = cloud_new.points[i].x;
//		pt.y = cloud_new.points[i].y;
//		pt.z = cloud_new.points[i].z;

//		std::vector<int> ptIndices(1);
//		std::vector<float> squared(1);
//		if (kdt2.nearestKSearch(pt, 1, ptIndices, squared) > 0)
//		{			
//			for (size_t i = 0; i < ptIndices.size(); ++i)
//			std::cout << "\t" << cloud_new.points[ptIndices[i]].x
//					  << " " << cloud_new.points[ptIndices[i]].y
//					  << " " << cloud_new.points[ptIndices[i]].z
//					  << " (squared distance: " << sqrt(squared[i]) << ")" << std::endl;
//			std::cout <<endl;
//		}
//	}
//	
//	
//	sign_points.clear();
}

void ICPLocalizer::points_dr_fifo(OPointCloud& cloud_sum, OPointCloud cloud_new, double odom_inc, double yaw_inc)
{
	double inc_x_ = odom_inc * cos( yaw_inc );
	double inc_y_ = odom_inc * sin( yaw_inc );
	
	Eigen::Affine3f trans = Eigen::Affine3f::Identity();
	trans.translation() << -inc_x_, -inc_y_, 0;
	trans.rotate(Eigen::AngleAxisf(-yaw_inc, Eigen::Vector3f::UnitZ()));
	
	pcl::transformPointCloud(cloud_sum, cloud_sum, trans);
	
	cloud_sum = cloud_sum + cloud_new;
	
	
	for (pcl::PointCloud<PointXYZO>::iterator it = cloud_sum.begin(); it != cloud_sum.end();)
	{
	    if ( (*it).x > 1 || (*it).x < -10 ) it = cloud_sum.erase(it);
	    else ++it;
	}
	
	
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "icp_localizer");

	ICPLocalizer localizer;

	return 0;
}
