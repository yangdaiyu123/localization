#include "lcp_matching.h"

ros::Publisher pub_cloud_tr;


void frontCurbCallback(const OPointCloud::ConstPtr &input)
{
	if(input->empty())
		return;
	
	PointXYZO po;
	pcl::PointCloud<PointXYZO> cloud_new;
	cloud_new.clear();
	for(int i=0; i<input->points.size(); i++)
	{
		po.x = input->points[i].x;
		po.y = input->points[i].y;
		po.z = 0;
		po.orientation = input->points[i].orientation;
		cloud_new.push_back(po);
	}
	
	pcl::search::KdTree<PointXYZO> kdtree;
	kdtree.setInputCloud(cloud_new.makeShared());

	PointXYZO point;
	point.x = 0.0;
	point.y = 0.0;
	point.z = 0.0;
	std::vector<int> pointIndices(5);
	std::vector<float> squaredDistances(5);
	if (kdtree.nearestKSearch(point, 2, pointIndices, squaredDistances) > 0)
	{
		std::cout << "5 nearest neighbors of the point:" << std::endl;
		for (size_t i = 0; i < pointIndices.size(); ++i)
			std::cout << "\t" << cloud_new.points[pointIndices[i]].x
					  << " " << cloud_new.points[pointIndices[i]].y
					  << " " << cloud_new.points[pointIndices[i]].z
					  << " (squared distance: " << squaredDistances[i] << ")" << std::endl;
	}	
	LcpMatching<PointXYZO> LCP;
	pcl::PointCloud<PointXYZO> cloud_trans;
	
	pcl::PointCloud<PointXYZO> cloud_sum;
	cloud_sum.clear();
	
	cloud_trans = LCP.cloud_translation_step( cloud_new , 0 , 10.0 );
	cloud_sum += cloud_trans;
	cloud_trans = LCP.cloud_translation_step( cloud_new , 0 , -10.0 );
	cloud_sum += cloud_trans;
	cloud_trans = LCP.cloud_translation_step( cloud_new , 0 , 20.0 );
	cloud_sum += cloud_trans;
	cloud_trans = LCP.cloud_translation_step( cloud_new , 0 , -20.0 );
	cloud_sum += cloud_trans;
	
	cloud_sum.header.frame_id = "base_link";
	pcl_conversions::toPCL(ros::Time::now(), cloud_sum.header.stamp);
	sensor_msgs::PointCloud2 cloud_to_pub;
	pcl::toROSMsg(cloud_sum, cloud_to_pub);
	pub_cloud_tr.publish(cloud_to_pub);
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "lcp_matching");
	
	ros::NodeHandle nh;
	ros::Subscriber sub_front_curb = nh.subscribe("front_curb_raw",2, &frontCurbCallback);
	pub_cloud_tr = nh.advertise<sensor_msgs::PointCloud2>("front_curb_trans", 2);
	

	ros::spin();

	return 0;
}
