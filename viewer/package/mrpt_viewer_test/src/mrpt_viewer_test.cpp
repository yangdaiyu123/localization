#include "mrpt_viewer_test.h"
#include <sstream>

//pcl::visualization::CloudViewer MyViewer("cloud");



//mrpt::gui::CDisplayWindow3D win3D;



int main(int argc, char **argv)
{
	ros::init(argc, argv, "mrpt_viewer_test");

	MrptViewer viewer;

	ros::NodeHandle nh;
	ros::Rate loop_rate(20);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sum(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::PointXYZ pt;
	pt.x=0; pt.y=0;pt.z=0;;
	cloud_sum->push_back(pt);

	int cnt=0;

//	win3D.setWindowTitle("CyberTiggo_View");
//    win3D.resize(600, 600);
//    win3D.setCameraAzimuthDeg(270);//方向沿y轴由负方向向正方向看
//    win3D.setCameraElevationDeg(60);//俯角20°
//    win3D.setCameraPointingToPoint(0, 10, 0);//指向(0,10,0)点
//    win3D.setCameraZoom(100);


	while (ros::ok())
	{
		std::stringstream ss;
		ss<<cnt++;

		if (pcl::io::loadPCDFile<pcl::PointXYZ> ("/home/wlh/point_map/"+ss.str()+".pcd", *cloud_in) == -1)
			std::cout<<"read error\n";


		*cloud_sum += *cloud_in;
		cloud_sum->header.frame_id = "map";

		mrpt::maps::CColouredPointsMap cloud_transformed;
    	cloud_transformed.setFromPCLPointCloud(*cloud_sum);
    	viewer.display(cloud_transformed);
//		MyViewer.showCloud(cloud_sum , "cloud");

//		mrpt::opengl::COpenGLScenePtr	scene;
//		mrpt::opengl::CGridPlaneXYPtr groundPlane;
//		mrpt::opengl::CAxisPtr  axis;

//		groundPlane = mrpt::opengl::CGridPlaneXY::Create(-300,300,-300,300, 0,5); //地平线网格间隔
//		groundPlane->setColor(0.0f, 0.0f, 0.0f);
//		groundPlane->setLineWidth(0.5);

//		axis = mrpt::opengl::CAxis::Create(0,0,0,2,4,2,0.25,3,false);
//		axis->setColor(mrpt::utils::TColorf(0,0,1,1));

//		scene = mrpt::opengl::COpenGLScene::Create();
//		scene->insert(axis);
//		scene->insert(groundPlane);

//		mrpt::opengl::CPointCloudPtr cloud_to_view = opengl::CPointCloud::Create();
//		cloud_to_view->loadFromPointsMap(&cloud_transformed);
//		scene->insert(cloud_to_view);

//		mrpt::opengl::COpenGLScenePtr &ptrScene = win3D.get3DSceneAndLock();
//		ptrScene = scene;
//		win3D.unlockAccess3DScene();
//		win3D.forceRepaint();

		ros::spinOnce();
		loop_rate.sleep();
	}


	return 0;
}
