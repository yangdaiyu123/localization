#include "ros/ros.h"
#include <pcl/io/pcd_io.h>
#include <pcl/common/common_headers.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/io.h>
#include <pcl/common/transforms.h>

#include <iostream>
#include <assert.h>

//pcl
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

//octomap
#include <octomap/octomap.h>


using namespace std;

int main( int argc, char** argv )
{
	ros::init(argc, argv, "octomap_viewer");

	ros::NodeHandle nh;
	ros::Rate loop_rate(20);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sum(new pcl::PointCloud<pcl::PointXYZ>);

	int cnt=0;
	for(int i=0; i<1526; i++)
	{
		std::stringstream ss;
		ss<<cnt++;

		if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/wlh/point_map_xuanhuai/"+ss.str()+".pcd", *cloud_in) == -1)
			std::cout<<"read error\n";


		*cloud_sum += *cloud_in;
		cloud_sum->header.frame_id = "map";
		loop_rate.sleep();
    }

    cout<<"point cloud loaded, piont size = "<<cloud_sum->points.size()<<endl;

    //声明octomap变量
    cout<<"copy data into octomap..."<<endl;
    // 创建八叉树对象，参数为分辨率，这里设成了0.05
    octomap::OcTree tree( 0.05 );

    for( int i=0; i < cloud_sum->points.size(); i++)
    {
        // 将点云里的点插入到octomap中
        tree.updateNode( octomap::point3d(cloud_sum->points[i].x, cloud_sum->points[i].y, cloud_sum->points[i].z), true );
    }

    // 更新octomap
    tree.updateInnerOccupancy();
    // 存储octomap
    tree.writeBinary( "/home/wlh/octomap.bt" );
    cout<<"done."<<endl;

    return 0;
}
