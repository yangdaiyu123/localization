#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Int8MultiArray.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Float64MultiArray.h"

#include <pcl/common/common_headers.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/io.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/contrib/contrib.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <custom_msgs/BoundingBox2d.h>
#include <custom_msgs/BoundingBox2dMultiArray.h>


using namespace std;
using namespace cv;



int main(int argc, char **argv)
{
	ros::init(argc, argv, "road_marking_process");
	ros::NodeHandle nh;

	Mat img = imread("/home/wlh/road_marking.png");
	Mat raw_img;
	cvtColor( img, raw_img, COLOR_BGR2GRAY );
	
	Mat kernelDilateL = getStructuringElement(MORPH_RECT, Size(3, 21));
	Mat kernelErode = getStructuringElement(MORPH_RECT, Size(3, 21));
	dilate(raw_img, raw_img, kernelDilateL);
	erode(raw_img, raw_img, kernelErode);
	
	std::vector<std::vector<cv::Point> > contours ;  
    cv::findContours(raw_img , contours , CV_RETR_EXTERNAL , CV_CHAIN_APPROX_NONE) ;  
    cv::Mat box_img(raw_img.size() , CV_8UC1 , cv::Scalar(255)) ;  
    cv::drawContours(box_img , contours , -1 , cv::Scalar(0) , 1) ; 
    Mat box_img_color;
    cvtColor(box_img,box_img_color,cv::COLOR_GRAY2RGB);

	if(!contours.empty())
    {
    	for(int i=0; i<contours.size(); i++)
    	{
    		cv::Rect rect = cv::boundingRect(cv::Mat(contours[i])); 
    		cv::rectangle(box_img_color,rect,cv::Scalar(200,200,200),CV_FILLED); 
    		cv::rectangle(img,rect,cv::Scalar(200,200,200),CV_FILLED); 
    	}
    }
	
	cv::imshow("road_marking",img);
	cv::imshow("box_img",box_img_color);
	
	
	cv::waitKey(0);
    
	return 0;
}
