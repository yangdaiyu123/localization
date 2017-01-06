#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int8.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Int8MultiArray.h"
#include <string>
#include <iostream>
#include <fstream>
#include <strstream>
#include <ican/canframe.h>

using namespace std;

class CanFrameDecoder
{
public:

	CanFrameDecoder();
    void chatterCallback(ican::canframe can_msg);

private:

    ros::NodeHandle n;
    ros::Subscriber sub;
    ros::Publisher pub_ang;
    ros::Publisher pub_pulse1;
    ros::Publisher pub_pulse2;
    ros::Publisher pub_pulse;
    ros::Publisher pub_trigger;
    std_msgs::Float64 steerAngle_;
    std_msgs::Int8 pulses_;
    std_msgs::Int8 pulses2_;

    int id;
    int len;
    int data[4];
    short a,b;//这里要注意，必须定义为整形变量，否则会丢失数据符号！
    float steerAngle;
    int pulses,pulses2;
    int speed;
    int mileage;
    int brakePosition,throttlePosition;
    int keys,remoteKeys;
    int camera_trigger;
};

CanFrameDecoder::CanFrameDecoder()
{
	camera_trigger = 0;

    sub = n.subscribe("can_frame", 10, &CanFrameDecoder::chatterCallback,this);
    pub_ang = n.advertise<std_msgs::Float64>("steer_angle",10);
    pub_pulse1 = n.advertise<std_msgs::Int8>("pulse1",10);
    pub_pulse2 = n.advertise<std_msgs::Int8>("pulse2",10);
    pub_pulse = n.advertise<std_msgs::Int8MultiArray>("pulse",10);
    pub_trigger = n.advertise<std_msgs::Int32>("camera_trigger",10);
    ros::spin();
}

void CanFrameDecoder::chatterCallback(ican::canframe can_msg)
{
    id = can_msg.id;
    len = can_msg.len;
    for(int i=0;i<len;i++)
    	data[i] = can_msg.data[i];
	std_msgs::Int8MultiArray pulse;
	std_msgs::Int32 trigger_out;

    switch(id)
    {
	 case 0x181://转角
        a=data[0]<<8;
        b=a+data[1];
	    steerAngle = b*1.0/10;


        steerAngle_.data = steerAngle;
        pub_ang.publish(steerAngle_);

//        cout<<"frame_id: "<<std::hex<<id<<"\t"
//	        <<"data_len: "<<len<<"\t"
//	        <<"steerAngle: "<<steerAngle<<endl;
    break;

    	case 0x182://脉冲
		pulses = data[0];
		pulses2 = data[1];

        pulses_.data = pulses;
        pulses2_.data = pulses2;
        pub_pulse1.publish(pulses_);
        pub_pulse2.publish(pulses2_);

        pulse.data.push_back(pulses);
        pulse.data.push_back(pulses2);
        pub_pulse.publish(pulse);

//		cout<<"frame_id: "<<std::hex<<id<<"\t"
//	        <<"data_len: "<<len<<"\t"
//	        <<"pulse1: "<<pulses<<"\t"
//	        <<"pulse2: "<<pulses2<<endl;
	break;

	case 0x183://里程(cm)
		mileage = ((can_msg.data[0])<<24) + ((can_msg.data[1])<<16) + ((can_msg.data[2])<<8) + (can_msg.data[3]);
//		cout<<"frame_id: "<<std::hex<<id<<"\t"
//	        <<"data_len: "<<len<<"\t"
//	        <<"mileage: "<<mileage<<endl;
	break;

	case 0x184://刹车油门位置
		brakePosition = ((can_msg.data[0])<<8)+(can_msg.data[1]);
		throttlePosition = ((can_msg.data[2])<<8)+(can_msg.data[3]);
//		cout<<"frame_id: "<<std::hex<<id<<"\t"
//	        <<"data_len: "<<len<<"\t"
//	        <<"brakePosition: "<<brakePosition<<"\t"
//	        <<"throttlePosition: "<<throttlePosition<<endl;
	break;

	case 0x185://按键
		keys = can_msg.data[0];
//		cout<<"frame_id: "<<std::hex<<id<<"\t"
//	        <<"data_len: "<<len<<"\t"
//	        <<"keys: "<<keys<<endl;
	break;

	case 0x186://遥控
                remoteKeys = ((can_msg.data[0])<<8)+can_msg.data[1];
//                cout<<"frame_id: "<<std::hex<<id<<"\t"
//	        <<"data_len: "<<len<<"\t"
//	        <<"remoteKeys: "<<remoteKeys<<endl;
	break;

	case 0x702://摄像头触发
		trigger_out.data = camera_trigger++;
		pub_trigger.publish(trigger_out);
        cout<<"frame_id: "<<std::hex<<id<<"\t"
	        <<"data_len: "<<len<<"\t"
	        <<"trigger_cnt: "<<std::oct<<camera_trigger<<endl;
	break;

	default:
	break;
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "canframe_decoder");
    CanFrameDecoder DecoderObject;

    return 0;
}
